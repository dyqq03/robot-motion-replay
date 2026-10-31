#!/usr/bin/env python3
"""全身回放引擎 —— 从 openarm_ui/monitor_node.py 搬移 (2026-09-05 版本, 2985 行)。

搬移纪律 (规格 §5.2/§5.3): 搬移而不是重写。接近段 / eyou 零阶保持 /
BEST_EFFORT 双发对冲都是真机调出来的, 每个函数与源文件逐行对应
(源行号见各函数 docstring 的 "源:" 标注)。

与源文件的差异 (仅此 6 类, 其余逐字一致):
  1. 反馈读取: monitor 从 SharedStore 读 → 本类从 FeedbackStore 读
     (server 进程自己订阅 4 路反馈话题, QoS 与面板一致, 见 FeedbackStore)
  2. 手遥操链子进程管理 (_hand_chain_kill/start/restart) 不搬 ——
     那是面板进程职责 (核对报告 §12); 相关位置留注释说明
  3. goal 执行路径禁止现转: load() 遇未转换录制直接报错; 转换由
     warmup_conversions()/convert_recording() 在启动与 list_actions 时预热
     (规格 §4.3)
  4. neck_joint_names 由构造参数注入 (monitor 原从 ROS 参数读)
  5. estop 增加脖子 t=0 抱住 (裁决 §13-3); resume/set_speed 为新增方法
     (裁决 §13-5, 复用源引擎的 on-track 换向路径)
  6. 接近段时长: 固定 2s → 按固定峰值限速反推 (π/2)·max|Δq|/v_peak
     (2026-10-10 改; 之前短暂用过"录制实测 vmax"自适应, 那个统计被 dt
     毛刺污染已废弃)。峰值上限 APPROACH_VMAX=0.5 rad/s (腰腿
     EYOU_VMAX 同值的度值), 另加逐帧 slew 钳制做纯兜底; 见
     _fbp_publish_approach; 脖子接近段同步用该时长 (源硬编码 2s)
  7. 2026-10-17 双臂起播: 从新鲜 JTC 保持目标接入，至少 1s 余弦过渡，
     避免把承重后的实测位置直接用作新轨迹起点；其他链保留原最短时长。
  8. 正常结束归位: 从新鲜 JTC 输出目标衔接，默认 1s；距离较大时仍按
     原有峰值限速延长，不再把承重后的实测位置作为双臂归位轨迹第一点。

publishers 由调用方注入 (构造参数 pubs), QoSProfile 由 server 侧"原样复制"
monitor_node._make_control_publishers (核对报告 §3):
    arm_traj:  {side: pub}  BEST_EFFORT + VOLATILE d10  ← 唯一不能改错
    neck_traj / hand / eyou_cmd: 默认 RELIABLE d10
"""

import json
from openarm_ui.arm_home import home_plan
import math
import os
import subprocess
import sys
import threading
import time
from pathlib import Path

from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

try:
    import h5py as _h5py
except ImportError:  # 缺 h5py 时回放加载报错 (与 monitor 面板同款兜底)
    _h5py = None

# 断流缝合 (2026-09-12, 与 openarm_ui.convert_bag 共享逻辑): 源流高负载下
# 周期性断流, 恢复后首条是断流前的陈旧值 — 原样回放手臂在断流处猛抽,
# vmax 统计也会被陈旧跳变污染 (接近段时长失真)。转换器缺依赖时退化为不缝合
try:
    from openarm_ui.convert_bag import GAP_THRESHOLD_S, stitch_gaps
except ImportError:  # pragma: no cover
    GAP_THRESHOLD_S = {"arm_cmd_l": 0.2, "arm_cmd_r": 0.2, "arm_js": 0.2,
                       "neck": 0.2, "hand": 0.2, "hand_left": 0.2,
                       "hand_right": 0.2, "eyou": 0.25}
    def stitch_gaps(t, data, threshold, stale_tol=0.02):
        return t, data, []

# 数据集根目录 (monitor KER_DATASET_DIR 的 parent/"fullbody", L168-169)
FULLBODY_ROOT = Path("/ros_ws/src/vla_collection/datasets/fullbody")

# 双臂 JTC 关节序号 (monitor L135): joint6 已物理移除
ARM_JOINT_NUMS = (1, 2, 3, 4, 5, 7)


class FeedbackStore:
    """monitor SharedStore 的最小替代 (核对报告 §12): 只缓存回放引擎需要的 4 路反馈。

    订阅 QoS 与面板一致 (monitor_node L538-551, 全部默认 RELIABLE d10):
        /joint_states                   JointState           (joint_state_broadcaster)
        /neck_joint_states              JointState           (neck_node)
        /hand/joint_states              JointState           (实测关节角，度)
        /Lower_limbs_Waist_motor_state  Float64MultiArray    (eyou_motor_node, 度)
    """

    def __init__(self, node):
        self._lock = threading.Lock()
        self._js = None
        self._neck = None
        self._hand = None
        self._hand_left = None       # 双手链 namespace 反馈 (2026-09-10 起)
        self._hand_right = None
        self._hand_received = {}
        self._eyou = None
        self._arm_commands = {}
        # 反馈订阅挪独立 executor 线程 (2026-09-12): 这 6 条 (250Hz joint_states
        # 领衔) 原先与 action/服务(estop)/定时器挤同一根主 executor 线程, 每秒
        # 700+ 次唤醒 × O(全部实体) 的等待集重建 ≈ 节点 59% CPU 全是簿记
        # (py-spy 实测 92.7% 采样在 spin 机制里)。拆到专用小等待集后主 executor
        # 只剩 action/服务 —— 急停不再排在订阅回调后面。回调仍只写快照 (单写者),
        # 读侧 getter 带 _lock, 跨线程语义不变。
        # 注: 本机 Humble rclpy 的 Executor 没有 add_callback_group (Iron 才有),
        # 同一节点拆不到两个 executor, 所以给 6 条快照订阅一个专用私有节点
        # —— 同进程共享 DDS participant, 额外开销可忽略; 外部看到的订阅者
        # 只是节点名变成 body_feedback_store, 话题/QoS/数量都不变。
        self._fb_node = Node("body_feedback_store", namespace=node.get_namespace())
        self._fb_node.create_subscription(JointState, "/joint_states", self._on_js, 10)
        self._fb_node.create_subscription(JointState, "/neck_joint_states", self._on_neck, 10)
        self._fb_node.create_subscription(JointState, "/hand/joint_states", self._on_hand, 10)
        self._fb_node.create_subscription(
            JointState, "/left/hand/joint_states", self._on_hand_left, 10)
        self._fb_node.create_subscription(
            JointState, "/right/hand/joint_states", self._on_hand_right, 10)
        self._fb_node.create_subscription(
            Float64MultiArray, "/Lower_limbs_Waist_motor_state", self._on_eyou, 10)
        # Read the target actually sent by each JTC. Replacing that target with
        # loaded-arm feedback at playback entry can abruptly remove PD support.
        for side in ("left", "right"):
            self._fb_node.create_subscription(
                JointTrajectoryControllerState,
                f"/{side}_joint_trajectory_controller/controller_state",
                lambda msg, side=side: self._on_arm_command(side, msg),
                qos_profile_sensor_data)
        self._fast_exec = SingleThreadedExecutor()
        self._fast_exec.add_node(self._fb_node)
        threading.Thread(target=self._fast_exec.spin, daemon=True,
                         name="feedback-fast-exec").start()
        node.get_logger().info(
            "[FeedbackStore] 8 路反馈/控制目标订阅 → 私有节点+独立 executor 线程"
            " (簿记瘦身, 主 executor 只留 action/服务)")

    def _on_js(self, m: JointState):
        with self._lock:
            self._js = m
            self._js_received = time.monotonic()

    def _on_neck(self, m: JointState):
        with self._lock:
            self._neck = m
            self._neck_received = time.monotonic()

    def _on_arm_command(self, side: str, msg):
        names = list(msg.joint_names)
        # output is the applied command; reference/desired support older JTCs.
        positions = None
        for field in ("output", "reference", "desired"):
            point = getattr(msg, field, None)
            values = list(getattr(point, "positions", ()))
            if len(values) == len(names) and values:
                positions = [float(v) for v in values]
                break
        if positions is None or not all(math.isfinite(v) for v in positions):
            return
        with self._lock:
            self._arm_commands[side] = (names, positions, time.monotonic())

    def arm_command_pos(self, side: str) -> list | None:
        """Fresh JTC output in hardware joint order; never substitute feedback."""
        with self._lock:
            sample = self._arm_commands.get(side)
        if sample is None:
            return None
        names, positions, received = sample
        if time.monotonic() - received > 0.25:
            return None
        try:
            return [positions[names.index(f"openarm_{side}_joint{j}")]
                    for j in ARM_JOINT_NUMS]
        except (ValueError, IndexError):
            return None

    def _on_hand(self, m: JointState):
        with self._lock:
            self._hand = m
            self._hand_received["hand"] = time.monotonic()

    def _on_hand_left(self, m: JointState):
        with self._lock:
            self._hand_left = m
            self._hand_received["hand_left"] = time.monotonic()

    def _on_hand_right(self, m: JointState):
        with self._lock:
            self._hand_right = m
            self._hand_received["hand_right"] = time.monotonic()

    def _on_eyou(self, m: Float64MultiArray):
        # 面板入口消毒同款 (monitor L505-515): NaN → 0, round 4
        vals = [round(float(x), 4) if math.isfinite(float(x)) else 0.0
                for x in m.data]
        with self._lock:
            self._eyou = vals

    def _pos_by_names(self, msg: JointState | None, names: list) -> list | None:
        if msg is None:
            return None
        try:
            idx = [msg.name.index(n) for n in names]
        except ValueError:
            return None
        # monitor _js() 对 pos round 4 (L365-371), 回放路径读到的即 4 位值
        return [round(float(msg.position[i]), 4) for i in idx]

    def arm_pos(self, side: str) -> list | None:
        """一臂 6 关节当前位姿 (rad, ARM_JOINT_NUMS 序) —— monitor _arm_current_pos."""
        with self._lock:
            js = self._js
        return self._pos_by_names(js, [f"openarm_{side}_joint{n}"
                                       for n in ARM_JOINT_NUMS])

    def arm_pos_12(self) -> list | None:
        """双臂 12 关节当前位姿 (左 6 + 右 6) —— monitor _arm_current_pos_12."""
        parts = []
        for side in ("left", "right"):
            p = self.arm_pos(side)
            if p is None:
                return None
            parts.extend(p)
        return parts

    def neck_pos(self, names: list) -> list | None:
        """脖子当前位姿 (按 neck_joint_names 映射) —— monitor _neck_current_pos."""
        with self._lock:
            js = self._neck
        return self._pos_by_names(js, names)

    def hand_pos(self, key: str = "hand") -> tuple:
        """Measured calibrated degrees; never substitute target commands.

        Both receipt age and acquisition timestamp must be fresh. Legacy single
        hand recordings may use the measured left hand on a bimanual chain.
        """
        with self._lock:
            m = {"hand": self._hand, "hand_left": self._hand_left,
                 "hand_right": self._hand_right}.get(key)
            received = self._hand_received.get(key, 0.0)
            if m is None and key == "hand":
                m = self._hand_left
                received = self._hand_received.get("hand_left", 0.0)
        if m is None or time.monotonic() - received > 0.5:
            return None, None
        stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        age = (self._fb_node.get_clock().now().nanoseconds - stamp) / 1e9
        if stamp <= 0 or not -0.05 <= age <= 0.5:
            return None, None
        names, positions = list(m.name), list(m.position)
        if (not names or len(names) != len(positions) or len(set(names)) != len(names)
                or not all(math.isfinite(v) for v in positions)):
            return None, None
        return positions, names

    def eyou_deg(self) -> list | None:
        """eyou 反馈最新一帧 (度, 已消毒); 长度 <4 返回 None —— monitor 接近段检查."""
        with self._lock:
            cur = self._eyou
        if not cur or len(cur) < 4:
            return None
        return list(cur)


HAND_DISABLE_FLAG = '/ros_ws/config/disable_hand_actions'

def _hands_disabled():
    return Path(HAND_DISABLE_FLAG).is_file()


class FullbodyPlayer:
    """全身回放引擎 (monitor_node 全身回放段搬移, 主状态机原样)。

    回放时钟由外部驱动: server 每 100Hz 调一次 tick() (monitor 原为
    create_timer(1.0/SCRUB_HZ, self._fbp_tick), SCRUB_HZ=100)。
    """

    FB_APPROACH_S = 2.0                     # monitor L1401; 无目标/退化时的兜底接近时长
    APPROACH_VMAX = 0.5                     # arm/neck rad/s
    HAND_APPROACH_VMAX_DEG_S = math.degrees(APPROACH_VMAX)  # 28.65 deg/s
    APPROACH_FLOOR_S = 0.2                  # 退化最小接近时长 (距离≈0, 防 0 点轨迹)
    ARM_APPROACH_FLOOR_S = 1.0              # 起播前双臂至少平滑接近 1 秒
    HOME_RETURN_S = 1.0                    # 回放结束后的归位过渡默认 1 秒
    EYOU_VMAX = math.degrees(APPROACH_VMAX)  # 腰腿链单位是度 → 同一条限速的度值 28.65
    EYOU_HOLD_GAP_S = 0.25                  # monitor L1731

    def __init__(self, node, pubs: dict, neck_joint_names: list):
        """
        :param node: rclpy Node (只用于日志; 订阅在 FeedbackStore 内部建)
        :param pubs: {"arm_traj": {side: Publisher}, "neck_traj": Publisher,
                      "hand": Publisher, "eyou_cmd": Publisher}
                     QoS 由调用方原样复制 monitor (见模块 docstring)
        :param neck_joint_names: ["neck_joint1", "neck_joint2"] (monitor 参数默认)
        """
        self._node = node
        self._log = node.get_logger()
        # 属性名与 monitor 一致, 函数体逐字对应
        self._pub_arm_traj = pubs["arm_traj"]
        self._pub_neck_traj = pubs["neck_traj"]
        self._pub_hand = pubs["hand"]
        # 双手链回放话题 (可选: 老调用方只传 "hand" 也能跑)
        self._pub_hand_left = pubs.get("hand_left")
        self._pub_hand_right = pubs.get("hand_right")
        self._pub_eyou_cmd = pubs["eyou_cmd"]
        self._neck_names = [str(n) for n in neck_joint_names]
        self._fb = FeedbackStore(node)
        # 全身回放内部状态 (monitor __init__ L454-459 同构; fbplay 快照改由
        # snapshot() 对外提供, 不再写 SharedStore)
        self._fbp = {"loaded": False, "name": None, "T": 0.0, "chains": {},
                     "playing": False, "mode": "idle", "speed": 1.0, "t": 0.0,
                     "approach_target": "start", "approach_start": 0.0,
                     "approach_dur": None,   # None=自动 (按峰值限速反推)
                     "last_stream": 0.0, "last_approach_tick": 0.0,
                     "t0": 0.0, "play_wall": 0.0, "prebuilt": {}}
        self._snap = None
        self._last_approach_done = []   # NEW: 最近一次接近段实际执行链 (scheduler 诊断用)

    # ------------------------------------------------------------------
    # 对外快照 (monitor _fbp_store 的替代: 面板/查询走 query_state)
    # ------------------------------------------------------------------
    def _snapshot(self) -> None:
        st = self._fbp
        self._snap = {
            "loaded": st["loaded"], "name": st["name"],
            "playing": st["playing"],
            "approaching": st["mode"] == "approach",
            "t": round(st["t"], 3), "T": round(st["T"], 3),
            "speed": st["speed"], "chains": list(st["chains"]),
        }

    def snapshot(self) -> dict:
        """monitor _fbp_store 同形快照 (面板曲线游标同款字段)。"""
        return dict(self._snap or {})

    # ------------------------------------------------------------------
    # 加载 (fb_list / _fbp_load; 臂链按原始录制时间戳加载, 不做重采样)
    # ------------------------------------------------------------------
    def fb_list(self) -> dict:
        """datasets/fullbody 下所有录制 (含转换状态与元信息) —— monitor fb_list。"""
        root = FULLBODY_ROOT
        items = {}
        if root.is_dir():
            for d in sorted(root.iterdir()):
                if not d.is_dir() or not d.name.startswith("fb_"):
                    continue
                info = {"name": d.name, "converted": (d / "fullbody.hdf5").is_file()}
                mp = d / "fullbody_meta.json"
                if mp.is_file():
                    try:
                        with open(mp) as f:
                            meta = json.load(f)
                        info["duration_sec"] = meta.get("duration_sec")
                        chains = meta.get("chains", {})
                        info["chains"] = [k for k, v in chains.items()
                                          if v.get("frames")]
                    except (OSError, ValueError):
                        pass
                items[d.name] = info
        return {"ok": True, "items": items}

    def _fbp_load(self, name: str) -> dict:
        """加载 fullbody 录制 (monitor _fbp_load L1530-1619)。

        与源差异: 未转换直接报错 —— goal 执行路径禁止现转 (规格 §4.3),
        预热见模块级 warmup_conversions()/convert_recording()。
        播放中换加载的"先停"保留; 手遥操链恢复不搬 (面板进程职责)。
        """
        root = FULLBODY_ROOT
        d = root / name
        if not d.is_dir():
            return {"ok": False, "error": f"录制不存在: {name}"}
        if self._fbp.get("playing"):
            self._fbp["playing"] = False
            self._fbp["mode"] = "idle"
            # 手遥操链恢复属面板职责 (核对报告 §12), 此处不搬 _hand_chain_*
        h5p = d / "fullbody.hdf5"
        if not h5p.is_file():
            return {"ok": False,
                    "error": f"录制未转换: {name} (预热转换未完成, 请稍后重试)"}
        if _h5py is None:
            return {"ok": False, "error": "h5py 不可用"}
        mp = d / "fullbody_meta.json"
        if mp.is_file():
            try:
                with open(mp) as f:
                    missing = (json.load(f).get("missing_joints") or [])
                if missing:
                    return {"ok": False,
                            "error": f"录制缺关节 (回放会命令到 0, 拒绝): {missing}"}
            except (OSError, ValueError):
                pass
        chains = {}
        with _h5py.File(h5p, "r") as f:
            arm_mode = str(f.attrs.get("arm_mode", "none"))
            # 臂链按【原始录制时间戳】加载 (不再 100Hz 重采样): 回放忠实复刻
            # 录制时的指令流, 倍速通过 time_from_start/|speed| 实现
            if arm_mode in ("ker", "mixed") and f["arms/cmd_left"].shape[0] > 0:
                chains["arms"] = {
                    "mode": "ker",
                    "left": {"t": [float(x) for x in f["arms/t_arm_cmd_l"][...]],
                             "q": [list(r) for r in f["arms/cmd_left"][...]]},
                    "right": {"t": [float(x) for x in f["arms/t_arm_cmd_r"][...]],
                              "q": [list(r) for r in f["arms/cmd_right"][...]]},
                }
            if "arms" not in chains and arm_mode in ("teach", "mixed") \
                    and f["arms/joint_position"].shape[0] > 0:
                chains["arms"] = {
                    "mode": "teach",
                    "t": [float(x) for x in f["arms/t_arm_js"][...]],
                    "q": [list(r) for r in f["arms/joint_position"][...]],
                }
            if f["neck/q"].shape[0] > 0:
                chains["neck"] = {"t": list(f["neck/t"][...]),
                                  "q": [list(r) for r in f["neck/q"][...]]}
            if f["hand/q"].shape[0] > 0:
                chains["hand"] = {"t": list(f["hand/t"][...]),
                                  "q": [list(r) for r in f["hand/q"][...]],
                                  "names": [str(n) for n in
                                            f["hand"].attrs.get("joint_names", [])]}
            # 双手链 (2026-09-10 起; 老 h5 无这两个组 → 跳过)
            for hk in ("hand_left", "hand_right"):
                if hk in f and f[hk + "/q"].shape[0] > 0:
                    chains[hk] = {"t": list(f[hk + "/t"][...]),
                                  "q": [list(r) for r in f[hk + "/q"][...]],
                                  "names": [str(n) for n in
                                            f[hk].attrs.get("joint_names", [])]}
            if f["eyou/q"].shape[0] > 0:
                chains["eyou"] = {"t": list(f["eyou/t"][...]),
                                  "q": [list(r) for r in f["eyou/q"][...]]}
        if _hands_disabled():
            ignored = [key for key in ("hand", "hand_left", "hand_right") if key in chains]
            for key in ignored:
                chains.pop(key)
            if ignored:
                self._log.info("[hands disabled] Ignoring recorded chains: " + ", ".join(ignored))
        if not chains:
            return {"ok": False, "error": "录制里没有启用的部件数据（手指动作已禁用时会跳过手部链）"}
        # 断流缝合 (2026-09-12): 载入时清洗内存副本 (不改 h5 文件); 必须在
        # 预建首尾帧之前 (陈旧跳变 0.7rad/8ms ≈ 87rad/s 会把首尾帧和接近段
        # 目标位算歪)。带值特征判定所以幂等, 已清洗的新 h5 再跑一遍是无操作
        if "arms" in chains:
            ac = chains["arms"]
            if ac["mode"] == "ker":
                for side in ("left", "right"):
                    ac[side]["t"], ac[side]["q"], _ = stitch_gaps(
                        ac[side]["t"], ac[side]["q"], GAP_THRESHOLD_S["arm_cmd_l"])
            else:
                ac["t"], ac["q"], _ = stitch_gaps(
                    ac["t"], ac["q"], GAP_THRESHOLD_S["arm_js"])
        for key in ("neck", "hand", "hand_left", "hand_right"):
            if key in chains:
                chains[key]["t"], chains[key]["q"], _ = stitch_gaps(
                    chains[key]["t"], chains[key]["q"], GAP_THRESHOLD_S["hand"])
        if "eyou" in chains:
            chains["eyou"]["t"], chains["eyou"]["q"], _ = stitch_gaps(
                chains["eyou"]["t"], chains["eyou"]["q"], GAP_THRESHOLD_S["eyou"])
        # 2026-10-10: 接近段不再按"录制实测 vmax"自适应。那个 vmax 用单帧
        # |Δq|/Δt 统计, 被 dt<1ms 的毛刺抬高 30-60 倍 (实测真实 2.2 rad/s →
        # 算成 74-139 rad/s), need=max|Δq|/vmax 于是恒 <1s、被下限夹住 ——
        # 自适应等于没写, 而且方向反了 (vmax 越大接近段越快)。现改为固定
        # 峰值限速反推时长, 见 APPROACH_VMAX / _chain_approach_need。
        T = 0.0
        for c in chains.values():
            if c.get("t"):
                T = max(T, c["t"][-1])
            elif c.get("left"):      # ker 双臂子流
                T = max(T, c["left"]["t"][-1], c["right"]["t"][-1])
        prebuilt = {"arms": {"fwd": {}, "rev": {}}, "neck": {"fwd": None, "rev": None}}
        if "arms" in chains:
            prebuilt["arms"]["fwd"] = self._fbp_build_arm_msgs(chains["arms"], 0.0, 1.0)
            prebuilt["arms"]["rev"] = self._fbp_build_arm_msgs(chains["arms"], T, -1.0)
        if "neck" in chains:
            prebuilt["neck"]["fwd"] = self._fbp_build_neck_msg(chains["neck"], 0.0, 1.0)
            prebuilt["neck"]["rev"] = self._fbp_build_neck_msg(chains["neck"], T, -1.0)
        self._fbp = {
            "loaded": True, "name": name, "T": T, "chains": chains,
            "playing": False, "mode": "idle", "speed": 1.0,
            "t": T, "approach_target": "start", "approach_start": 0.0,
            "approach_dur": None,   # None=自动 (距离/实测速度自适应)
            "last_stream": 0.0, "t0": 0.0, "play_wall": 0.0,
            "prebuilt": prebuilt,
        }
        self._snapshot()
        self._log.info(
            f"[全身回放] 加载 {name}: 链={list(chains)}, T={T:.1f}s")
        return {"ok": True, "name": name, "T": round(T, 2),
                "chains": list(chains)}

    # ------------------------------------------------------------------
    # 取帧 / 构建消息 (monitor _fbp_frame L1621 / _fbp_build_arm_msgs L1639 /
    #                _fbp_build_neck_msg L1679 / _fbp_frame_hold L1733)
    # ------------------------------------------------------------------
    def _fbp_frame(self, chain, t: float):
        """取链在时刻 t 的帧 (线性插值), 返回 (帧, 有效)。"""
        ts, qs = chain["t"], chain["q"]
        if not ts or t < ts[0] or t > ts[-1]:
            return None, False
        lo, hi = 0, len(ts) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        if ts[hi] <= t:
            return list(qs[hi]), True
        a = (t - ts[lo]) / max(ts[hi] - ts[lo], 1e-9)
        return [qs[lo][k] + a * (qs[hi][k] - qs[lo][k])
                for k in range(len(qs[lo]))], True

    def _fbp_build_arm_msgs(self, c: dict, t: float, speed: float) -> dict:
        """从时刻 t 起按方向拼双臂 JTC 整段消息 (原始录制时间戳, 原样回放)。

        每臂用各自的录制流组点 (ker 模式左右臂时间轴独立 — 与录制一致);
        倍速 = 时间轴压缩: time_from_start = |ts[i] - t| / |speed| —
        采样率不变, 相邻帧沿轨迹的 delta 随倍速变大 (发布频率不提高)。
        """
        out = {}
        for side, off in (("left", 0), ("right", 6)):
            if c["mode"] == "ker":
                st_ = c[side]
                ts, qs, off = st_["t"], st_["q"], 0
            else:  # teach: 单张 12 宽流
                ts, qs = c["t"], c["q"]
            if not ts:
                continue
            if speed > 0:
                idx = [i for i in range(len(ts)) if ts[i] >= t - 1e-9]
                dur = [(ts[i] - t) / speed for i in idx]
            else:
                idx = [i for i in range(len(ts) - 1, -1, -1) if ts[i] <= t + 1e-9]
                dur = [(t - ts[i]) / (-speed) for i in idx]
            if not idx:
                continue
            names = [f"openarm_{side}_joint{n}" for n in ARM_JOINT_NUMS]
            msg = JointTrajectory()
            msg.joint_names = names
            for i, d in zip(idx, dur):
                msg.points.append(JointTrajectoryPoint(
                    positions=[float(qs[i][off + k])
                               for k in range(len(ARM_JOINT_NUMS))],
                    time_from_start=Duration(seconds=d).to_msg()))
            out[side] = msg
        return out

    def _fbp_build_neck_msg(self, c: dict, t: float, speed: float):
        """从时刻 t 起按方向拼脖子 JTC 整段消息 (原始时间戳), 无点返回 None.

        time_from_start = |ts[i] - t|: 以"当前播放时刻 t"为基准 (monitor 原注释,
        否则脖子动作与臂/手脱同步).
        """
        ts, qs = c["t"], c["q"]
        if speed > 0:
            idx = [i for i in range(len(ts)) if ts[i] >= t - 1e-9]
            dur = [(ts[i] - t) / speed for i in idx]
        else:
            idx = [i for i in range(len(ts) - 1, -1, -1) if ts[i] <= t + 1e-9]
            dur = [(t - ts[i]) / (-speed) for i in idx]
        if not idx:
            return None
        names = self._neck_names
        msg = JointTrajectory()
        msg.joint_names = names
        for i, d in zip(idx, dur):
            msg.points.append(JointTrajectoryPoint(
                positions=[float(x) for x in qs[i]],
                time_from_start=Duration(seconds=d).to_msg()))
        return msg

    def _fbp_frame_hold(self, chain, t: float):
        """同 _fbp_frame, 但间隔 >EYOU_HOLD_GAP_S 按零阶保持 + 结束前 50ms 短斜坡.

        腰腿录制是 10Hz 键盘指令流 (monitor 原注释): 长间隔是真实停顿,
        跨停顿线性插值会把停顿变成缓慢漂移.
        """
        ts, qs = chain["t"], chain["q"]
        if not ts or t < ts[0] or t > ts[-1]:
            return None, False
        lo, hi = 0, len(ts) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if ts[mid] <= t:
                lo = mid
            else:
                hi = mid
        if ts[hi] <= t:
            return list(qs[hi]), True
        gap = ts[hi] - ts[lo]
        if gap <= self.EYOU_HOLD_GAP_S:
            a = (t - ts[lo]) / max(gap, 1e-9)
        else:
            ramp = 0.05
            a = max(0.0, (t - (ts[hi] - ramp)) / ramp)
        return [qs[lo][k] + a * (qs[hi][k] - qs[lo][k])
                for k in range(len(qs[lo]))], True

    # ------------------------------------------------------------------
    # 发布 (monitor _fbp_publish_full L1704 / _fbp_stream_tick L1761)
    # ------------------------------------------------------------------
    def _hand_pub(self, key: str):
        """手链 key → 对应回放 publisher (hand_left/right 未配置返回 None)."""
        if _hands_disabled():
            return None
        return {"hand": self._pub_hand,
                "hand_left": self._pub_hand_left,
                "hand_right": self._pub_hand_right}.get(key)
    def _fbp_publish_full(self, t: float, speed: float) -> None:
        """发整段剩余轨迹: 臂(JTC)/脖(JTC) 一次整段. 手/腰腿走 tick 流.

        正放起点 t=0 / 倒放起点 t=T 用 _fbp_load 预构建的消息; 播放中换向
        (任意 t) 现拼.
        """
        st = self._fbp
        chains = st["chains"]
        pre = st.get("prebuilt", {})
        entry = ((speed > 0 and t <= 0.0)
                 or (speed < 0 and t >= st["T"] - 1e-9))
        # 预构建消息只对 1 倍速有效 (时间轴未经 /speed 压缩), 其余倍速现拼
        use_pre = entry and abs(abs(speed) - 1.0) < 1e-9
        key = "fwd" if speed > 0 else "rev"
        if "arms" in chains:
            if use_pre and pre.get("arms", {}).get(key):
                msgs = pre["arms"][key]
            else:
                msgs = self._fbp_build_arm_msgs(chains["arms"], t, speed)
            for side, msg in msgs.items():
                self._pub_arm_traj[side].publish(msg)
        if "neck" in chains:
            if use_pre and pre.get("neck", {}).get(key) is not None:
                msg = pre["neck"][key]
            else:
                msg = self._fbp_build_neck_msg(chains["neck"], t, speed)
            if msg is not None:
                self._pub_neck_traj.publish(msg)

    def _fbp_stream_tick(self, t: float, speed: float) -> None:
        """play 阶段每 tick: 手/腰腿按墙钟时刻 t 流式插值发布."""
        st = self._fbp
        chains = st["chains"]
        now = time.monotonic()
        if now - st["last_stream"] < 0.02:   # 50Hz 足够, 减轻负载
            return
        st["last_stream"] = now
        for hk in ("hand", "hand_left", "hand_right"):
            if _hands_disabled():
                continue
            if hk not in chains:
                continue
            q, ok = self._fbp_frame(chains[hk], t)
            if not ok:
                continue
            msg = JointState()
            msg.name = chains[hk].get("names") or [f"j{i}" for i in range(len(q))]
            msg.position = [float(x) for x in q]
            pub = self._hand_pub(hk)
            if pub is not None:
                pub.publish(msg)
            if hk == "hand" and self._pub_hand_left is not None:
                self._pub_hand_left.publish(msg)   # 方案 A 桥: 老录制是左手数据
        if "eyou" in chains:
            q, ok = self._fbp_frame_hold(chains["eyou"], t)
            if ok:
                self._pub_eyou_cmd.publish(
                    Float64MultiArray(data=[float(x) for x in q]))

    # ------------------------------------------------------------------
    # 接近段 (monitor _fbp_publish_approach L1782 / _fbp_approach_tick L1851)
    # ------------------------------------------------------------------
    @staticmethod
    def _clamp_slew(prev: list, tgt: list, max_step: float) -> list:
        """逐关节把一步位移限制在 max_step 内 (兜底钳制)。

        正常情况时长按同一个峰值限速反推, 余弦峰值恰好 = 限速, 这里永不
        触发; 一旦触发 (时长算错/异常录制/NaN) 就牺牲到位精度换取不超速
        —— 慢是安全方向。
        """
        return [p + max(-max_step, min(max_step, t - p))
                for p, t in zip(prev, tgt)]

    def _chain_approach_need(self, cur: list, tgt: list, v_peak) -> float:
        """单链所需接近时长 = (π/2)·max|Δq| / v_peak。

        余弦插值 s=0.5(1-cos(πi/n)) 的峰值是平均速度的 π/2 倍, 所以按峰值
        反推: 峰值 = v_peak 时 T = (π/2)·d/v_peak。v_peak 用链自身单位
        (臂/脖 APPROACH_VMAX rad/s，手 HAND_APPROACH_VMAX_DEG_S deg/s，腰腿 EYOU_VMAX deg/s), 单位自洽。
        v_peak 未知/非正时退兜底时长 FB_APPROACH_S。
        """
        if not v_peak or v_peak <= 0:
            return self.FB_APPROACH_S
        d = max(abs(a - b) for a, b in zip(cur, tgt))
        return (math.pi / 2.0) * d / v_peak

    def _hand_measured_start(self, key, chain):
        """Align measured feedback by name before planning any trajectory."""
        if self._hand_pub(key) is None:
            raise ValueError(f"{key} 未连接手部回放发布器")
        q, names = self._fb.hand_pos(key)
        expected = chain.get("names") or []
        if (q is None or not expected or len(set(expected)) != len(expected)
                or len(names) != len(q) or len(set(names)) != len(names)):
            raise ValueError(f"{key} 缺少新鲜、有效的手部实测角度，拒绝起播")
        values = dict(zip(names, q))
        if not set(expected).issubset(values) or not all(math.isfinite(values[n]) for n in expected):
            raise ValueError(f"{key} 实测关节名称与动作包不匹配")
        if any(len(row) != len(expected) or not all(math.isfinite(v) for v in row)
               for row in chain["q"]):
            raise ValueError(f"{key} 动作包关节角无效")
        return [values[n] for n in expected]

    def _fbp_publish_approach(self, target: str) -> list:
        """全身统一接近段: 各链从当前位姿插值到起点/终点帧. 返回实际执行链.

        2026-10-10: 时长不再按"录制实测 vmax"自适应 (那个统计被 dt 毛刺
        污染, 见 _chain_approach_need 与 _stream_vmax 删除说明), 改为固定
        峰值限速反推: 每链 need = (π/2)·max|Δq|/v_peak, 全局
        dur = max(各链 need, APPROACH_FLOOR_S)。峰值限速本身由
        APPROACH_VMAX (臂/脖)，HAND_APPROACH_VMAX_DEG_S (手, 度) 与 EYOU_VMAX (腰腿, 度) 给定。
        显式 transition_duration 只能把接近段**放慢**, 不能加快; 明显快于
        限速所需时记 warn 并按限速放慢。另加逐帧 slew 钳制作纯兜底。
        """
        st = self._fbp
        chains = st["chains"]
        end = target == "end"
        idx = -1 if end else 0
        # ① 各链当前反馈 + 目标帧 + 所需时长
        plan = {}   # chain -> {"cur", "tgt", "need", ...}
        if "arms" in chains:
            cur = self._fb.arm_pos_12()
            if cur is not None:
                if (time.monotonic() - getattr(self._fb, "_js_received", 0.0) > 0.5
                        or not all(math.isfinite(v) for v in cur)):
                    raise ValueError("起播过渡缺少新鲜、有效的双臂反馈")
                commands = {side: self._fb.arm_command_pos(side)
                            for side in ("left", "right")}
                if any(q is None for q in commands.values()):
                    raise ValueError("起播过渡缺少新鲜双臂控制目标，请等待控制器状态恢复")
                # Keep the first commanded position continuous. Actual feedback
                # remains required above, but is not a replacement for a target.
                cur = commands["left"] + commands["right"]
                c = chains["arms"]
                if c["mode"] == "ker":
                    tgt = {side: c[side]["q"][idx][:6]
                           for side in ("left", "right")}
                    plan["arms"] = {
                        "cur": {"left": cur[:6], "right": cur[6:]}, "tgt": tgt,
                        "need": max(
                            self._chain_approach_need(cur[:6], tgt["left"],
                                                      self.APPROACH_VMAX),
                            self._chain_approach_need(cur[6:], tgt["right"],
                                                      self.APPROACH_VMAX)),
                        "mode": "ker"}
                else:
                    plan["arms"] = {
                        "cur": cur, "tgt": c["q"][idx],
                        "need": self._chain_approach_need(cur, c["q"][idx],
                                                          self.APPROACH_VMAX),
                        "mode": "teach"}
        if "neck" in chains:
            cur = self._fb.neck_pos(self._neck_names)
            if cur is not None:
                plan["neck"] = {
                    "cur": cur, "tgt": chains["neck"]["q"][idx],
                    "need": self._chain_approach_need(cur, chains["neck"]["q"][idx],
                                                      self.APPROACH_VMAX)}
        for hk in ("hand", "hand_left", "hand_right"):
            if _hands_disabled():
                continue
            if hk not in chains:
                continue
            hs = self._hand_measured_start(hk, chains[hk])
            plan[hk] = {
                "cur": hs, "tgt": chains[hk]["q"][idx],
                "need": self._chain_approach_need(hs, chains[hk]["q"][idx],
                                                  self.HAND_APPROACH_VMAX_DEG_S)}
        if "eyou" in chains:
            ec = self._fb.eyou_deg()
            if ec is not None:
                plan["eyou"] = {
                    "cur": ec, "tgt": chains["eyou"]["q"][idx],
                    "need": self._chain_approach_need(ec, chains["eyou"]["q"][idx],
                                                      self.EYOU_VMAX)}
        if not plan:
            return []
        # ② 全局时长: 各链最慢者 (至少 APPROACH_FLOOR_S, 防距离≈0 时 0 点轨迹);
        #    显式 transition_duration 只能把接近段放慢, 不能快于峰值限速。
        dur = max(self.APPROACH_FLOOR_S, max(p["need"] for p in plan.values()))
        if "arms" in plan:
            dur = max(dur, self.ARM_APPROACH_FLOOR_S)
        explicit = st.get("approach_dur")
        if explicit is not None and explicit > 0:
            if explicit + 0.25 < dur:
                self._log.warn(
                    f"[接近段] transition_duration={explicit:.1f}s 快于"
                    f"{self.APPROACH_VMAX} rad/s 峰值限速所需 {dur:.1f}s, "
                    f"已按限速放慢")
            dur = max(dur, explicit)
        st["approach_dur"] = dur
        # ③ 发臂/脖整段 JTC (点数随 dur; 脖不再硬编码 2s) + 记手/腰腿起止
        done = []
        if "arms" in plan:
            p = plan["arms"]
            dt = 0.01
            n = max(1, int(math.ceil(dur / dt)))
            dt = dur / n
            step = self.APPROACH_VMAX * dt        # 逐帧兜底: 单帧位移上限
            for side, off in (("left", 0), ("right", 6)):
                if p["mode"] == "ker":
                    start, tgt = p["cur"][side], p["tgt"][side]
                else:
                    start, tgt = p["cur"][off:off + 6], p["tgt"][off:off + 6]
                names = [f"openarm_{side}_joint{jn}" for jn in ARM_JOINT_NUMS]
                msg = JointTrajectory()
                msg.joint_names = names
                prev = [float(x) for x in start]
                for i in range(n + 1):
                    s = 0.5 * (1.0 - math.cos(math.pi * i / n))
                    pts = [start[k] + s * (tgt[k] - start[k])
                           for k in range(6)]
                    if i == n:
                        pts = [float(x) for x in tgt]    # 末点精确到位
                    else:
                        pts = self._clamp_slew(prev, pts, step)
                    prev = pts
                    msg.points.append(JointTrajectoryPoint(
                        positions=pts,
                        time_from_start=Duration(seconds=i * dt).to_msg()))
                self._pub_arm_traj[side].publish(msg)
            done.append("arms")
        if "neck" in plan:
            cur, q0 = plan["neck"]["cur"], plan["neck"]["tgt"]
            names = self._neck_names
            msg = JointTrajectory()
            msg.joint_names = names
            dt = 0.05
            n = max(1, int(math.ceil(dur / dt)))
            step = self.APPROACH_VMAX * dt
            prev = [float(x) for x in cur]
            for i in range(n + 1):
                s = 0.5 * (1.0 - math.cos(math.pi * i / n))
                pts = [cur[k] + s * (q0[k] - cur[k]) for k in range(2)]
                if i == n:
                    pts = [float(x) for x in q0]
                else:
                    pts = self._clamp_slew(prev, pts, step)
                prev = pts
                msg.points.append(JointTrajectoryPoint(
                    positions=pts,
                    time_from_start=Duration(seconds=i * dt).to_msg()))
            self._pub_neck_traj.publish(msg)
            done.append("neck")
        for hk in ("hand", "hand_left", "hand_right"):
            if _hands_disabled():
                continue
            if hk in plan:
                st[hk + "_from"] = list(plan[hk]["cur"])
                st[hk + "_to"] = plan[hk]["tgt"]
                st[hk + "_last"] = list(plan[hk]["cur"])   # 逐帧兜底基准
                done.append(hk)
        if "eyou" in plan:
            st["eyou_from"] = plan["eyou"]["cur"]
            st["eyou_to"] = plan["eyou"]["tgt"]
            st["eyou_last"] = list(plan["eyou"]["cur"])
            done.append("eyou")
        st["approach_target"] = target
        st["approach_start"] = time.monotonic()
        st["last_approach_tick"] = st["approach_start"]   # 首拍从 0 时刻起算
        return done

    def _fbp_approach_tick(self) -> None:
        """接近段期间: 手/腰腿流式插值 (20Hz) + 逐帧 slew 兜底钳制。

        钳制步长用**实际墙钟间隔** (tick 抖动不会让单帧位移被放大):
        单帧位移上限 = 峰值限速 × 实际间隔。与时长参数解耦 —— 时长算错、
        录制异常都封得住, 代价只是慢一点 (慢是安全方向)。
        """
        st = self._fbp
        now = time.monotonic()
        prev_t = st.get("last_approach_tick", 0.0)
        if now - prev_t < 0.05:
            return
        st["last_approach_tick"] = now
        elapsed = max(now - prev_t, 1e-3)
        p = min(1.0, (now - st["approach_start"])
                / st.get("approach_dur", self.FB_APPROACH_S))  # NEW
        s = 0.5 * (1.0 - math.cos(math.pi * p))
        chains = st["chains"]
        for hk in ("hand", "hand_left", "hand_right"):
            if _hands_disabled():
                continue
            if hk + "_from" not in st or hk not in chains:
                continue
            q = [a + s * (b - a)
                 for a, b in zip(st[hk + "_from"], st[hk + "_to"])]
            q = self._clamp_slew(st.get(hk + "_last", st[hk + "_from"]), q,
                                 self.HAND_APPROACH_VMAX_DEG_S * elapsed)
            st[hk + "_last"] = q
            msg = JointState()
            msg.name = chains[hk].get("names") or [f"j{i}" for i in range(len(q))]
            msg.position = [float(x) for x in q]
            pub = self._hand_pub(hk)
            if pub is not None:
                pub.publish(msg)
            if hk == "hand" and self._pub_hand_left is not None:
                self._pub_hand_left.publish(msg)   # 方案 A 桥: 老录制是左手数据
        if "eyou_from" in st and "eyou" in chains:
            q = [a + s * (b - a)
                 for a, b in zip(st["eyou_from"], st["eyou_to"])]
            q = self._clamp_slew(st.get("eyou_last", st["eyou_from"]), q,
                                 self.EYOU_VMAX * elapsed)
            st["eyou_last"] = q
            self._pub_eyou_cmd.publish(
                Float64MultiArray(data=[round(x, 3) for x in q]))

    # ------------------------------------------------------------------
    # 主时钟 (monitor _fbp_tick L1872; 100Hz, 由 server 驱动)
    # ------------------------------------------------------------------
    def _begin_home_return(self):
        st = self._fbp
        if not ({"arms", "neck"} & set(st["chains"])):
            st["playing"], st["mode"] = False, "idle"
            return
        messages = {}
        duration = self.HOME_RETURN_S
        starts = None
        if "arms" in st["chains"]:
            if time.monotonic() - getattr(self._fb, "_js_received", 0.) > .5:
                raise ValueError("回放归位缺少新鲜双臂反馈")
            feedback = {side: self._fb.arm_pos(side) for side in ("left", "right")}
            if any(q is None or len(q) != 6 or not all(math.isfinite(v) for v in q)
                   for q in feedback.values()):
                raise ValueError("回放归位缺少完整、有效的双臂反馈")
            starts = {side: self._fb.arm_command_pos(side) for side in ("left", "right")}
            if any(q is None for q in starts.values()):
                raise ValueError("回放归位缺少新鲜双臂控制目标")
            _, duration = home_plan(starts, duration)
        neck_msg = None
        if "neck" in st["chains"]:
            if time.monotonic() - getattr(self._fb, "_neck_received", 0.) > .5:
                raise ValueError("回放归位缺少新鲜脖子反馈")
            cur = self._fb.neck_pos(self._neck_names)
            if cur is None or len(cur) != 2 or not all(math.isfinite(v) for v in cur):
                raise ValueError("回放归位缺少完整脖子反馈")
            target = [math.radians(4.5), math.radians(-8.3)]
            duration = max(duration, self._chain_approach_need(cur, target, self.APPROACH_VMAX))
            neck_msg = JointTrajectory()
            neck_msg.joint_names = list(self._neck_names)
            steps = max(1, math.ceil(duration * 50))
            for i in range(steps + 1):
                f = .5 * (1 - math.cos(math.pi * i / steps))
                neck_msg.points.append(JointTrajectoryPoint(
                    positions=[a + f * (b - a) for a, b in zip(cur, target)],
                    time_from_start=Duration(seconds=i * duration / steps).to_msg()))
        if starts is not None:
            messages, duration = home_plan(starts, duration)
        # Remain busy until the return trajectory finishes, so another action
        # cannot take ownership while the arms are still returning home.
        st["playing"], st["mode"] = True, "home_return"
        st["home_started"] = time.monotonic()
        st["home_duration"] = duration
        st["home_paused"] = False
        for side, msg in messages.items():
            self._pub_arm_traj[side].publish(msg)
        if neck_msg is not None:
            self._pub_neck_traj.publish(neck_msg)
        neck_note = "脖子目标 +4.5°, -8.3°" if neck_msg is not None else "本动作未发送脖子归位轨迹"
        self._log.info(f"[全身回放] 正常结束归位 ({neck_note}): {duration:.1f}s")

    def tick(self):
        """100Hz: 全身回放时钟 (monitor _fbp_tick L1872-1902, 逐字搬移).

        播放时刻按墙钟推进: 臂/脖 JTC 跑在硬件自己的钟上, tick 抖动若按
        次数累加会让流式链 (手/腰腿) 落后且永不追回.
        """
        st = self._fbp
        if not st["playing"]:
            return
        if st["mode"] == "home_return":
            if time.monotonic() - st["home_started"] >= st["home_duration"]:
                st["playing"], st["mode"] = False, "idle"
            self._snapshot()
            return
        if st["mode"] == "approach":
            self._fbp_approach_tick()
            if time.monotonic() - st["approach_start"] \
                    >= st.get("approach_dur", self.FB_APPROACH_S):  # NEW
                st["mode"] = "play"
                st["t"] = 0.0 if st["approach_target"] == "start" else st["T"]
                st["t0"] = st["t"]
                st["play_wall"] = time.monotonic()
                self._fbp_publish_full(st["t"], st["speed"])   # 全部链到位后才播
            self._snapshot()
            return
        t = st["t0"] + (time.monotonic() - st["play_wall"]) * st["speed"]
        completed = ((st["speed"] < 0 and t <= 0.)
                     or (st["speed"] > 0 and t >= st["T"]))
        st["t"] = max(0., min(t, st["T"]))
        if completed:
            self._fbp_stream_tick(st["t"], st["speed"])
            self._begin_home_return()
        else:
            self._fbp_stream_tick(st["t"], st["speed"])
        # 自然播完: 手遥操链恢复属面板职责 (核对报告 §12), 此处不搬
        self._snapshot()

    # ------------------------------------------------------------------
    # 入口 (monitor cmd_fb_play L1904 / cmd_fb_pause L1963 / cmd_estop L1990)
    # ------------------------------------------------------------------
    def play(self, speed: float, approach_dur: float = 0.0,
             skip_approach: bool = False) -> dict:
        """全身回放: speed ±1 语义 (正放=接近起点后播, 倒放=接近终点后倒).

        源 monitor cmd_fb_play L1904-1953; 与源差异:
          - 试教/手部动作库守卫不搬 (面板进程职责; server 单飞调度负责
            自身互斥, 面板侧与手部回放的互斥由 B5 query_state 把关)
          - 手遥操链停/起不搬 (面板进程职责, 核对报告 §12)
          - speed 范围保持源的 ±[0.05, 3.0] 作内部安全线; 对外契约
            [0.25, 2.0] 由 server 侧夹 (裁决 §13-1)
        NEW (接口契约 §4.1, 源无此概念):
          - approach_dur > 0: 显式指定接近段时长 (transition_duration,
            只能放慢: 快于峰值限速所需会被拦并记 warn); =0/None: 自动
            (各链 (π/2)·max|Δq|/v_peak, 见 _fbp_publish_approach)
          - skip_approach=True: smooth_transition=false, 不走接近段直接开播
            (身体仍在原处, JTC 首个目标点=录制首帧, 由控制器自行移动过去)
        """
        st = self._fbp
        if st["mode"] == "home_return":
            return {"ok": False, "error": "双臂正在回零，请等待完成或暂停"}
        if not st["loaded"]:
            return {"ok": False, "error": "未加载全身录制 (先加载)"}
        if abs(speed) < 0.05 or abs(speed) > 3.0:
            return {"ok": False, "error": "speed 需在 ±[0.05, 3.0]"}
        # Validate all requested hands before ANY arm/neck publication, including
        # skip-approach and direction changes. No fallback to target messages.
        try:
            for key in ("hand", "hand_left", "hand_right"):
                if key in st["chains"] and not _hands_disabled():
                    self._hand_measured_start(key, st["chains"][key])
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        st["speed"] = speed
        # 每次播放都重置: 显式覆盖只在本次 goal 生效, 不污染后续播放
        st["approach_dur"] = (float(approach_dur)
                              if approach_dur and approach_dur > 0 else None)
        if st["playing"]:
            # 安全: 接近段期间再点正放/倒放, 身体不在轨迹上而 st["t"] 是陈旧值,
            # 直接发整段会让手臂跳到终点/起点帧 (真机踩过: 连点两次正放
            # 机器人打自己) — 一律拒绝, 等接近段走完
            if st["mode"] != "play":
                return {"ok": False,
                        "error": "接近段进行中 (身体正插值到轨迹起点/终点), "
                                 "稍候再操作"}
            # 播放中换向: 身体在轨迹上, 直接按当前 t 发新方向整段
            st["mode"] = "play"
            st["t0"] = st["t"]
            st["play_wall"] = time.monotonic()
            self._fbp_publish_full(st["t"], speed)
            self._snapshot()
            return {"ok": True, "action": "play", "speed": speed,
                    "approach": False, "on_track": True}
        if skip_approach:                          # NEW: 不开接近段直接开播
            st["playing"] = True
            st["mode"] = "play"
            st["t"] = 0.0 if speed > 0 else st["T"]
            st["t0"] = st["t"]
            st["play_wall"] = time.monotonic()
            self._fbp_publish_full(st["t"], speed)
            self._snapshot()
            self._log.info(f"[全身回放] 直接开播 (skip_approach) speed={speed}")
            return {"ok": True, "action": "play", "speed": speed,
                    "approach": False, "skip_approach": True}
        # (源在此停手套源 _hand_chain_kill; 面板进程职责, 不搬)
        target = "start" if speed > 0 else "end"
        self._last_approach_done = []
        try:
            done = self._fbp_publish_approach(target)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        self._last_approach_done = list(done)      # NEW
        if not done:
            return {"ok": False,
                    "error": "各链当前无反馈, 无法接近 (拒绝盲播)"}
        st["playing"] = True
        st["mode"] = "approach"
        self._snapshot()
        self._log.info(
            f"[全身回放] speed={speed}, 接近段链={done}, 目标={target}")
        return {"ok": True, "action": "play", "speed": speed,
                "approach": True, "chains": done}

    def pause(self) -> dict:
        """全身暂停: 臂 t=0 抱住点, 脖 t=0 单点, 手/腰腿停流 (monitor cmd_fb_pause L1963)."""
        st = self._fbp
        st["home_paused"] = st["mode"] == "home_return"
        st["playing"] = False
        st["mode"] = "idle"
        for side in ("left", "right"):
            cur = self._fb.arm_pos(side)
            if cur is None:
                continue
            names = [f"openarm_{side}_joint{n}" for n in ARM_JOINT_NUMS]
            msg = JointTrajectory()
            msg.joint_names = names
            msg.points = [JointTrajectoryPoint(
                positions=cur, time_from_start=Duration(seconds=0.0).to_msg())]
            self._pub_arm_traj[side].publish(msg)
        cur = self._fb.neck_pos(self._neck_names)
        if cur is not None:
            names = self._neck_names
            msg = JointTrajectory()
            msg.joint_names = names
            msg.points = [JointTrajectoryPoint(
                positions=cur, time_from_start=Duration(seconds=0.0).to_msg())]
            self._pub_neck_traj.publish(msg)
        # (源在此恢复手遥操链; 面板进程职责, 不搬)
        self._snapshot()
        return {"ok": True, "action": "pause"}

    def resume(self) -> dict:
        """原位续播 (裁决 §13-5): 从暂停时的 t 继续, 不走接近段.

        复用源的 on-track 换向路径 (monitor cmd_fb_play L1924-1932):
        playing=True, t0=t, 按当前 t 重发整段剩余轨迹.
        """
        st = self._fbp
        if not st["loaded"]:
            return {"ok": False, "error": "未加载全身录制 (先加载)"}
        if st.get("home_paused"):
            self._begin_home_return()
            self._snapshot()
            return {"ok": True, "action": "resume", "returning_home": True}
        if st["playing"]:
            return {"ok": True, "action": "resume", "already_playing": True}
        st["playing"] = True
        st["mode"] = "play"
        st["t0"] = st["t"]
        st["play_wall"] = time.monotonic()
        self._fbp_publish_full(st["t"], st["speed"])
        self._snapshot()
        self._log.info(f"[全身回放] 原位续播 t={st['t']:.2f}s speed={st['speed']}")
        return {"ok": True, "action": "resume", "t": round(st["t"], 3)}

    def set_speed(self, speed: float) -> dict:
        """播放中调速 (on-track 换向同款路径, 不重新接近)."""
        st = self._fbp
        if st["mode"] == "home_return":
            return {"ok": False, "error": "双臂正在回零，请等待完成或暂停"}
        if not st["loaded"]:
            return {"ok": False, "error": "未加载全身录制 (先加载)"}
        if abs(speed) < 0.05 or abs(speed) > 3.0:
            return {"ok": False, "error": "speed 需在 ±[0.05, 3.0]"}
        st["speed"] = speed
        if st["playing"] and st["mode"] != "approach":
            st["t0"] = st["t"]
            st["play_wall"] = time.monotonic()
            self._fbp_publish_full(st["t"], speed)
        self._snapshot()
        return {"ok": True, "action": "set_speed", "speed": speed}

    def estop(self) -> dict:
        """急停: 停一切 + 双臂抱住当前位姿 (monitor cmd_estop L1990-2023).

        与源差异 (裁决 §13-3): 增加脖子 t=0 抱住 —— 规格"停一切";
        面板原版不碰脖子 (脖子轨迹已在控制器上整段执行).
        录制保存属面板职责 (server 无录制), 不搬。
        """
        self._fbp["home_paused"] = False
        if self._fbp["playing"]:
            self._fbp["playing"] = False
            self._fbp["mode"] = "idle"
            self._snapshot()
        held = []
        for side in ("left", "right"):
            cur = self._fb.arm_pos(side)
            if cur is None:
                continue
            names = [f"openarm_{side}_joint{n}" for n in ARM_JOINT_NUMS]
            msg = JointTrajectory()
            msg.joint_names = names
            msg.points = [JointTrajectoryPoint(
                positions=cur,
                time_from_start=Duration(seconds=0.0).to_msg())]
            self._pub_arm_traj[side].publish(msg)
            self._pub_arm_traj[side].publish(msg)   # BEST_EFFORT 双发对冲
            held.append(side)
        # NEW (裁决 §13-3): 脖子 t=0 抱住 —— 有反馈才发, 无反馈宁缺不盲发
        cur = self._fb.neck_pos(self._neck_names)
        if cur is not None:
            names = self._neck_names
            msg = JointTrajectory()
            msg.joint_names = names
            msg.points = [JointTrajectoryPoint(
                positions=cur, time_from_start=Duration(seconds=0.0).to_msg())]
            self._pub_neck_traj.publish(msg)
            held.append("neck")
        self._log.warn(f"[急停] 回放已停, 抱住: {held or '无反馈'}")
        return {"ok": True, "held": held,
                "note": f"已停回放并抱住当前位姿 ({','.join(held) or '无反馈'}); "
                        "estop 后无需手工复位, 直接发新 goal 即可正常播"}

    # ------------------------------------------------------------------
    # scheduler 用 helper (NEW, 源无: 供 B3 调度器查询而不直碰内部状态)
    # ------------------------------------------------------------------
    def load(self, name: str) -> dict:
        """加载入口包装 (B3 scheduler 用; 本体见 _fbp_load)。"""
        return self._fbp_load(name)

    def is_loaded(self) -> bool:
        return bool(self._fbp.get("loaded"))

    def is_playing(self) -> bool:
        return bool(self._fbp.get("playing"))

    def mode(self) -> str:
        """当前内部模式: idle / approach / play (scheduler 状态机同步用)。"""
        return self._fbp.get("mode", "idle")

    def chains(self) -> list:
        return list(self._fbp.get("chains", {}))

    def last_approach_done(self) -> list:
        return list(self._last_approach_done)

    def speed_actual(self) -> float:
        return float(self._fbp.get("speed", 1.0))

    # ------------------------------------------------------------------
    # feedback 辅助 (规格 §4.1 Feedback 字段)
    # ------------------------------------------------------------------
    def progress_info(self) -> dict:
        """当前 feedback 快照: progress/frame_idx/num_frames/elapsed (规格 §4.1).

        progress 相对播放段 (接近段期间为 0); frame 信息取臂网格链
        (无臂链填 0, 规格允许"未知填 0").
        """
        st = self._fbp
        T = st["T"] or 0.0
        if st["mode"] == "approach" or not st["playing"]:
            return {"progress": 0.0, "elapsed_sec": 0.0,
                    "frame_idx": 0, "num_frames": 0}
        t = max(0.0, min(st["t"], T))
        num_frames = 0
        frame_idx = 0
        arms = st["chains"].get("arms")
        if arms:
            # 原始流: 帧数=录制流长度; 索引按进度比例映射 (不再假设 100Hz 网格)
            if arms["mode"] == "ker":
                num_frames = max(min(len(arms["left"]["q"]),
                                     len(arms["right"]["q"])) - 1, 0)
            else:
                num_frames = max(len(arms["q"]) - 1, 0)
            if num_frames > 0 and T > 0:
                frame_idx = max(0, min(num_frames,
                                       round(t / T * num_frames)))
        return {"progress": (t / T) if T > 0 else 0.0,
                "elapsed_sec": round(t, 3),
                "frame_idx": frame_idx, "num_frames": num_frames}


# ----------------------------------------------------------------------
# 预热转换 (规格 §4.3: 不得在 goal 执行路径上现转; 录制完成立即转 /
# 服务启动与 list_actions 时补漏)
# ----------------------------------------------------------------------
def convert_recording(bag_dir: Path, timeout: float = 180.0) -> None:
    """跑 openarm_ui.convert_bag 转换单个录制 (monitor _fbp_load L1544-1551 同款)。

    实测 99s 录制 ≈ 2.8s (核对报告 §4); timeout 180 与面板一致。
    """
    subprocess.run(
        [sys.executable, "-m", "openarm_ui.convert_bag", str(bag_dir)],
        check=True, timeout=timeout,
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        env=os.environ.copy())


def warmup_conversions(logger=None) -> list:
    """扫描 FULLBODY_ROOT, 补漏转换所有缺 fullbody.hdf5 的录制 (串行)。

    返回成功转换的目录名列表; 单个失败不中断整体 (下轮再补)。
    """
    done, failed = [], []
    root = FULLBODY_ROOT
    if not root.is_dir():
        return done
    for d in sorted(root.iterdir()):
        if not d.is_dir() or not d.name.startswith("fb_"):
            continue
        if (d / "fullbody.hdf5").is_file():
            continue
        try:
            convert_recording(d)
            done.append(d.name)
            if logger is not None:
                logger.info(f"[预热] 已转换 {d.name}")
        except (subprocess.SubprocessError, OSError) as e:
            failed.append((d.name, str(e)))
            if logger is not None:
                logger.warn(f"[预热] 转换失败 {d.name}: {e}")
    return done
