#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rosbag 全身录制 → fullbody.hdf5 转换 (openarm_ui 自带, 无外部包依赖)。

输入: datasets/fullbody/fb_<时间戳>/ (ros2 bag, sqlite3 或 mcap)
输出: 同目录 fullbody.hdf5 + fullbody_meta.json

各链保留【原始频率与时间戳】, 不做重采样 (录制啥播啥):
  /arms (ker 模式):    commands 指令流  cmd_left/cmd_right (各 N×6) + t
  /arms (teach 模式):  /joint_states 采样 joint_position (N×12) + 关节名 + t
  /neck:               每条 joint_trajectory 消息首点 q (M×2) + t
  /hand:               joint_targets 消息 q (M×n) + 关节名 + t
  /eyou:               PP 命令 q (M×4, 1 值广播展开成 4) + t

双臂模式自动推断: 有 commands 话题→ker, 有 /joint_states→teach, 都有→mixed。

用法:
  python3 -m openarm_ui.convert_bag <bag_dir> [输出目录, 默认同 bag_dir]
"""

import json
import math
import sys
from pathlib import Path

import h5py
import numpy as np
from rosbag2_py import (
    ConverterOptions,
    SequentialReader,
    StorageFilter,
    StorageOptions,
)
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

MSG_TYPES = {
    "std_msgs/msg/Float64MultiArray": Float64MultiArray,
    "sensor_msgs/msg/JointState": JointState,
    "trajectory_msgs/msg/JointTrajectory": JointTrajectory,
}

# 每链流: key -> {"t": [...], "data": [...], "names": None}
# hand  = 单手链 /hand/joint_targets (老 bag 兼容);
# hand_left/hand_right = 双手链 namespace 话题 (2026-09-10 起)
STREAMS = ("arm_cmd_l", "arm_cmd_r", "arm_js", "neck",
           "hand", "hand_left", "hand_right", "eyou")

# 臂关节规范列序 (左 6 + 右 6): /joint_states 发布者的名序是任意的
# (实测 JSB 输出乱序), 必须按名重排, 否则回放端左右臂错位
ARM_CANON = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint7",
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint7",
]

# ===== 断流缝合 (2026-09-12) =====
# 源流 (/joint_states 等) 高负载下会周期性断流 0.2~1.7s (2026-09-11 排查:
# ros2_control 更新循环被挤 + JSB RELIABLE 流控), 恢复后第一条是断流前的
# 陈旧值, 隔几毫秒才跳到真值 — 曲线画成"平走+竖跳", 回放到该时刻手臂猛抽。
# 转换时兜底: 检测超过阈值的间隔, 按值特征丢掉陈旧的恢复样本;
# gap 区间另行写出 (h5 各链 gaps 数据集 + meta), 供曲线面板断开显示。
GAP_THRESHOLD_S = {
    "arm_cmd_l": 0.2, "arm_cmd_r": 0.2, "arm_js": 0.2,   # 250Hz: 0.2s=50 帧
    "neck": 0.2, "hand": 0.2, "hand_left": 0.2, "hand_right": 0.2,
    "eyou": 0.25,   # 10Hz 指令流, 与回放端 EYOU_HOLD_GAP_S 对齐
}
STALE_TOL = 0.02   # rad: 正常采样间隔内的关节移动 (~1°) 远小于此


def stitch_gaps(t, data, threshold, stale_tol: float = STALE_TOL):
    """检测 >threshold 的断流间隔, 丢掉值特征呈"陈旧"的恢复后首条样本.

    陈旧特征 (2026-09-11 实测断流形状): 恢复样本 ≈ 断流前值, 且与再下一条
    差异明显 (真值跳变)。带值判定所以幂等 — 对已清洗的流再跑一遍不会二次
    丢样本; 断流期间没动 (前值≈恢复值≈下一条) 的样本保留, 不丢真数据。
    返回 (t2, data2, gaps), gaps=[(start, end), ...]; 时间戳不重采样。
    """
    drop = set()
    gaps = []
    for i in range(len(t) - 2):
        if t[i + 1] - t[i] <= threshold:
            continue
        gaps.append((round(t[i], 4), round(t[i + 1], 4)))
        pre, post, nxt = data[i], data[i + 1], data[i + 2]
        stale = all(abs(post[k] - pre[k]) <= stale_tol for k in range(len(pre)))
        jumped = any(abs(nxt[k] - post[k]) > stale_tol for k in range(len(post)))
        if stale and jumped:
            drop.add(i + 1)
    if not drop:
        return t, data, gaps
    t2 = [x for k, x in enumerate(t) if k not in drop]
    d2 = [x for k, x in enumerate(data) if k not in drop]
    return t2, d2, gaps


def open_reader(bag_dir: str) -> SequentialReader:
    for storage in ("sqlite3", "mcap"):
        try:
            r = SequentialReader()
            r.open(StorageOptions(uri=bag_dir, storage_id=storage),
                   ConverterOptions("", "cdr"))
            return r
        except Exception:  # noqa: BLE001
            continue
    raise RuntimeError(f"无法打开 bag: {bag_dir}")


def infer_arm_mode(by_topic: dict) -> str:
    has_cmd = ("/left_forward_position_controller/commands" in by_topic
               and "/right_forward_position_controller/commands" in by_topic)
    has_js = "/joint_states" in by_topic
    if has_cmd and has_js:
        return "mixed"
    if has_cmd:
        return "ker"
    if has_js:
        return "teach"
    return "none"


def main():
    bag_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(".")
    out_dir = Path(sys.argv[2]) if len(sys.argv) > 2 else bag_dir

    reader = open_reader(str(bag_dir))
    metas = reader.get_all_topics_and_types()
    by_topic = {m.name: m.type for m in metas}
    arm_mode = infer_arm_mode(by_topic)

    streams = {k: {"t": [], "data": [], "names": None, "idx": None,
                   "idx_sig": None, "reorders": 0, "dropped": 0,
                   "missing": []} for k in STREAMS}
    reader.set_filter(StorageFilter(topics=list(by_topic.keys())))

    t0 = None
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        mtype = by_topic.get(topic)
        cls = MSG_TYPES.get(mtype)
        if cls is None:
            continue
        if t0 is None:
            t0 = t_ns
        t = round((t_ns - t0) / 1e9, 6)
        msg = deserialize_message(data, cls)
        s = None
        if topic == "/joint_states":
            s = streams["arm_js"]
            # 名序可能中途漂移 (JSB 重启/重配置): 按每条消息自身的 name
            # 顺序解析, 签名不变时复用缓存索引 — 而不是只在首帧锁一次
            # (首帧锁死在名序变化后会把后续所有帧的关节映射错位, 造成
            # 十度级跳变; 真机录制实测出现过)
            sig = tuple(msg.name)
            if s.get("idx_sig") != sig:
                raw = list(msg.name)
                idx, missing = [], []
                for n in ARM_CANON:
                    if n in raw:
                        idx.append(raw.index(n))
                    else:
                        idx.append(None)
                        missing.append(n)
                if s.get("idx") is not None:
                    s["reorders"] = s.get("reorders", 0) + 1
                    print(f"⚠ /joint_states 名序变化 "
                          f"(第 {s['reorders']} 次), 已重建映射")
                s["idx"] = idx
                s["idx_sig"] = sig
                s["names"] = list(ARM_CANON)
                s["missing"] = missing
            row = [float(msg.position[i]) if i is not None else 0.0
                   for i in s["idx"]]   # 按规范名序重排; 缺关节补 0 并在 meta 告警
        elif topic == "/left_forward_position_controller/commands":
            s = streams["arm_cmd_l"]
            row = [float(x) for x in msg.data]
        elif topic == "/right_forward_position_controller/commands":
            s = streams["arm_cmd_r"]
            row = [float(x) for x in msg.data]
        elif topic == "/neck_controller/joint_trajectory":
            if not msg.points:
                continue
            s = streams["neck"]
            row = [float(x) for x in msg.points[0].positions]   # 每条消息首点
        elif topic == "/hand/joint_targets":
            s = streams["hand"]
            row = [float(x) for x in msg.position]
            if s["names"] is None:
                s["names"] = list(msg.name)
        elif topic == "/left/hand/joint_targets":
            s = streams["hand_left"]
            row = [float(x) for x in msg.position]
            if s["names"] is None:
                s["names"] = list(msg.name)
        elif topic == "/right/hand/joint_targets":
            s = streams["hand_right"]
            row = [float(x) for x in msg.position]
            if s["names"] is None:
                s["names"] = list(msg.name)
        elif topic == "/Lower_limbs_motor_cmd":
            s = streams["eyou"]
            vals = [float(x) for x in msg.data]
            row = vals * 4 if len(vals) == 1 else vals[:4]      # 广播展开成 4
        if s is not None:
            s["t"].append(t)
            s["data"].append(row)

    # 断流缝合 (先于跳变消毒): 检测 >阈值的断流间隔, 丢掉恢复后第一条
    # "陈旧"样本 (值≈断流前值, 再下一条才跳真值) — 这是曲线"平走+竖跳"
    # 和回放猛抽的直接来源; gap 区间记录到 streams[k]["gaps"] 供写进 h5
    for key in STREAMS:
        st = streams[key]
        st["gaps"] = []
        st["dropped_stale"] = 0
        if len(st["t"]) < 3:
            continue
        n0 = len(st["t"])
        st["t"], st["data"], st["gaps"] = stitch_gaps(
            st["t"], st["data"], GAP_THRESHOLD_S[key])
        st["dropped_stale"] = n0 - len(st["t"])
        if st["gaps"]:
            print(f"⚠ {key}: 断流 {len(st['gaps'])} 处 "
                  f"(最长 {max(e - s for s, e in st['gaps']):.2f}s), "
                  f"剔除 {st['dropped_stale']} 帧陈旧恢复样本")

    # 跳变消毒 (安全): 剔除正常帧距内变化超过物理极限的坏帧 — 来源可能是
    # 双发布者交错 / 编码器冻结恢复快照等; 若几十度的单帧跳变原样进
    # 回放轨迹, JTC 会在数 ms 内插值穿过它 (等效上万度/秒甩臂)。
    # 只对"正常帧距"(dt ≤ NORMAL_DT_S) 生效: 跨断流间隔的值差是真实拖动,
    # 不判 — 否则 30°/帧 极限会把断流恢复的真值帧连坐剔光 (级联截断)
    NORMAL_DT_S = {"arm_js": 0.05, "hand": 0.1, "hand_left": 0.1,
                   "hand_right": 0.1, "eyou": 0.35}
    JUMP_LIMIT_RAD = math.radians(30.0)   # 250Hz 下 = 7500°/s, 远超关节能力
    for key in ("arm_js", "hand", "hand_left", "hand_right", "eyou"):
        st = streams[key]
        if not st["t"]:
            continue
        keep_t, keep_q, last = [], [], None
        for t_i, q_i in zip(st["t"], st["data"]):
            if last is not None and t_i - keep_t[-1] <= NORMAL_DT_S[key] and any(
                    abs(q_i[j] - last[j]) > JUMP_LIMIT_RAD
                    for j in range(len(q_i))):
                st["dropped"] += 1
                continue
            keep_t.append(t_i)
            keep_q.append(q_i)
            last = q_i
        if st["dropped"]:
            print(f"⚠ {key}: 剔除 {st['dropped']} 帧超物理极限跳变 "
                  f"(>{math.degrees(JUMP_LIMIT_RAD):.0f}°/帧)")
        st["t"], st["data"] = keep_t, keep_q

    max_t = max([s["t"][-1] for s in streams.values() if s["t"]] + [0.0])
    frames = {k: len(streams[k]["t"]) for k in STREAMS}
    print(f"bag: {bag_dir.name} | arm_mode={arm_mode} | 时长 {max_t:.2f}s")
    print("  ".join(f"{k}={frames[k]}" for k in STREAMS))

    with h5py.File(out_dir / "fullbody.hdf5", "w") as f:
        f.attrs["source_bag"] = bag_dir.name
        f.attrs["arm_mode"] = arm_mode
        f.attrs["duration_sec"] = round(max_t, 3)
        g = f.create_group("arms")
        g.attrs["mode"] = arm_mode
        for k, ds, w in (("arm_cmd_l", "cmd_left", 6), ("arm_cmd_r", "cmd_right", 6)):
            st = streams[k]
            arr = np.array(st["data"], dtype=float) if st["data"] else np.zeros((0, w))
            g.create_dataset(ds, data=arr)
            g.create_dataset("t_" + k, data=np.array(st["t"], dtype=float))
        st = streams["arm_js"]
        arr = np.array(st["data"], dtype=float) if st["data"] else np.zeros((0, 1))
        g.create_dataset("joint_position", data=arr)
        g.create_dataset("t_arm_js", data=np.array(st["t"], dtype=float))
        if st["names"]:
            g.attrs["joint_names"] = [str(n) for n in st["names"]]
        # 断流区间 (N×2, 空 = 无断流): 曲线面板据此在断流处断开不连线
        for k in ("arm_cmd_l", "arm_cmd_r", "arm_js"):
            g.create_dataset("gaps_" + k,
                             data=np.array(streams[k]["gaps"], dtype=float).reshape(-1, 2))
        g2 = f.create_group("neck")
        st = streams["neck"]
        g2.create_dataset("q", data=np.array(st["data"], dtype=float) if st["data"] else np.zeros((0, 2)))
        g2.create_dataset("t", data=np.array(st["t"], dtype=float))
        g2.create_dataset("gaps", data=np.array(st["gaps"], dtype=float).reshape(-1, 2))
        g3 = f.create_group("hand")
        st = streams["hand"]
        g3.create_dataset("q", data=np.array(st["data"], dtype=float) if st["data"] else np.zeros((0, 1)))
        g3.create_dataset("t", data=np.array(st["t"], dtype=float))
        g3.create_dataset("gaps", data=np.array(st["gaps"], dtype=float).reshape(-1, 2))
        if st["names"]:
            g3.attrs["joint_names"] = [str(n) for n in st["names"]]
        for key in ("hand_left", "hand_right"):     # 双手链 (老 bag 无此两组, 恒为空)
            gx = f.create_group(key)
            st = streams[key]
            gx.create_dataset("q", data=np.array(st["data"], dtype=float) if st["data"] else np.zeros((0, 1)))
            gx.create_dataset("t", data=np.array(st["t"], dtype=float))
            gx.create_dataset("gaps", data=np.array(st["gaps"], dtype=float).reshape(-1, 2))
            if st["names"]:
                gx.attrs["joint_names"] = [str(n) for n in st["names"]]
        g4 = f.create_group("eyou")
        st = streams["eyou"]
        g4.create_dataset("q", data=np.array(st["data"], dtype=float) if st["data"] else np.zeros((0, 4)))
        g4.create_dataset("t", data=np.array(st["t"], dtype=float))
        g4.create_dataset("gaps", data=np.array(st["gaps"], dtype=float).reshape(-1, 2))

    meta = {
        "source_bag": bag_dir.name,
        "arm_mode": arm_mode,
        "duration_sec": round(max_t, 3),
        "missing_joints": streams["arm_js"]["missing"],
        "name_reorders": streams["arm_js"]["reorders"],
        "dropped_jump_frames": streams["arm_js"]["dropped"],
        # 断流缝合汇总 (只列有断流的流): 曲线断开/回放清洗的依据
        "gaps": {k: {"count": len(streams[k]["gaps"]),
                     "max_gap_s": round(max((e - s for s, e in streams[k]["gaps"]),
                                           default=0.0), 3),
                     "dropped_stale": streams[k]["dropped_stale"]}
                 for k in STREAMS if streams[k]["gaps"]},
        "chains": {
            "arms": {"mode": arm_mode, "frames_cmd_l": frames["arm_cmd_l"],
                     "frames_cmd_r": frames["arm_cmd_r"],
                     "frames_js": frames["arm_js"], "units": "rad"},
            "neck": {"frames": frames["neck"], "units": "rad",
                     "interface": "/neck_controller/joint_trajectory"},
            "hand": {"frames": frames["hand"], "units": "rad",
                     "interface": "/hand/joint_targets"},
            "hand_left": {"frames": frames["hand_left"], "units": "rad",
                          "interface": "/left/hand/joint_targets"},
            "hand_right": {"frames": frames["hand_right"], "units": "rad",
                           "interface": "/right/hand/joint_targets"},
            "eyou": {"frames": frames["eyou"], "units": "deg",
                     "interface": "/Lower_limbs_motor_cmd"},
        },
    }
    with open(out_dir / "fullbody_meta.json", "w") as fp:
        json.dump(meta, fp, indent=4, ensure_ascii=False)
    print(f"已写出: {out_dir}/fullbody.hdf5 + fullbody_meta.json")
    if max_t == 0.0:
        print("⚠ bag 里没有消息 (录制时话题无发布者?)")


if __name__ == "__main__":
    main()
