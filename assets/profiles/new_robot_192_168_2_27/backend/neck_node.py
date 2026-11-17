#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
neck_node.py — 颈部 ROS2 节点 (直接驱动 CAN, 不依赖 ros2_control)

把 openarm_simple_hardware.cpp 的 on_activate / return_to_zero / read / write
+ ros2_control 的 JointTrajectoryController + JointStateBroadcaster 三件事
压到一个 Python 节点里, 只管颈部 2 个电机:
    0x01: DM4310   →  neck_joint1
    0x02: DM4340   →  neck_joint2

启动序列 (复刻 openarm_simple_hardware.cpp:223-353):
  1) (可选) 拉起 CAN-FD 1M/5M
  2) OpenArm.init_arm_motors([DM4310, DM4340], [0x01, 0x02], [0x11, 0x12])
  3) set_callback_mode_all(STATE) → enable_all → recv_all        (on_activate)
  4) return_to_zero: 200 步线性插值到 zero_pos (默认 [0, 0]) (MIT PD) (return_to_zero)
  5) 进入控制循环 @ control_hz:
       · refresh_all + recv_all, 读 position/velocity/torque     (read)
       · 发布 /joint_states                                       (JointStateBroadcaster)
       · 若当前有有效轨迹, 按时间线性插值得到目标 → 发 MIT PD     (write)
         否则抱住最后已知目标 (初始为回零目标)

订阅:
    /neck_controller/joint_trajectory   (trajectory_msgs/JointTrajectory)
    /neck_controller/gains              (std_msgs/Float64MultiArray [kp1,kp2,kd1,kd2] 在线调增益)
发布:
    /neck_joint_states                  (sensor_msgs/JointState)

参数 (ros2 param):
    can_interface    string         "can0"
    joint_names      string_array   ["neck_joint1", "neck_joint2"]
    control_hz       double         200.0
    kp               double_array   [10.0, 60.0]     # DM4310 / DM4340
    kd               double_array   [0.7, 2.0]
    bringup_can      bool           true
    zero_steps       int            200
    zero_step_ms     int            10
    zero_pos         double_array   [0.0, 0.0]   # Canonical startup radians
    motor_directions double_array   [1.0, -1.0]  # This robot: reverse J2 only
    motor_zero_offsets double_array [0.0, 0.0]   # Existing motor origins unchanged

退出 (Ctrl-C): 连发 3 次 disable_all (复刻 on_deactivate 第 238-251 行).

注意 MITParam 顺序为 (kp, kd, q, dq, tau) — 与 cpp write() / teleop_collect 一致.

用法:
    # 1) 直接当脚本跑 (需先 source ROS2 环境)
    python3 neck_node.py
    # 2) 包成 ROS2 节点跑
    ros2 run neck_controll neck_node --ros-args -p can_interface:=can0
    # 3) 跳过 CAN 拉起 (已经 ip link 配好时)
    ros2 run neck_controll neck_node --ros-args -p bringup_can:=false
"""

from .command_mirror import CommandMirror

import math
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

from openarm_can import OpenArm, CallbackMode, MotorType, MITParam


# ==========================
# 颈部电机配置
# ==========================
NECK_DOF = 2
MOTOR_TYPES = [MotorType.DM4310, MotorType.DM4340]
MOTOR_IDS = [0x09, 0x010]
MASTER_IDS = [0x19, 0x20]
CAN_FD = True

# ==========================
# 关节限位 (rad) — 适配颈部机械结构, 命令值会被夹到此范围
# ==========================
JOINT_LIMITS = {
    "neck_joint1": [-math.pi / 2, math.pi / 2],
    "neck_joint2": [-math.pi / 6, math.pi / 6],
}

TRAJ_TOPIC = "/neck_controller/joint_trajectory"
JS_TOPIC = "/neck_joint_states"
GAINS_TOPIC = "/neck_controller/gains"   # data=[kp1,kp2,kd1,kd2] 在线调增益


def bringup_can(iface):
    """CAN-FD 1M/5M 拉起 (CAN 配置不跨重启, 每次都要)."""
    subprocess.run(["sudo", "ip", "link", "set", iface, "down"],
                   stderr=subprocess.DEVNULL)
    r = subprocess.run(["sudo", "ip", "link", "set", iface, "up", "type", "can",
                        "bitrate", "1000000", "dbitrate", "5000000",
                        "fd", "on", "restart-ms", "1"])
    return r.returncode == 0


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


class NeckCoordinates:
    """Canonical ROS radians <-> this robot's motor radians, at CAN boundary only."""
    def __init__(self, directions, offsets):
        self.directions = [float(v) for v in directions]
        self.offsets = [float(v) for v in offsets]
        if (len(self.directions) != NECK_DOF or len(self.offsets) != NECK_DOF
                or any(v not in (-1.0, 1.0) for v in self.directions)
                or any(not math.isfinite(v) for v in self.offsets)):
            raise ValueError("motor_directions must be two +/-1 values; offsets two finite radians")

    def motor_position(self, i, logical):
        return self.offsets[i] + self.directions[i] * logical

    def logical_position(self, i, motor):
        return self.directions[i] * (motor - self.offsets[i])

    def signed(self, i, value):
        # Same sign for velocity and effort; PD gains are unchanged.
        return self.directions[i] * value


class NeckNode(Node):
    def __init__(self):
        super().__init__("neck_node")

        # ==========================
        # 参数
        # ==========================
        self.declare_parameter("can_interface", "can2")
        self.declare_parameter("joint_names", ["neck_joint1", "neck_joint2"])
        self.declare_parameter("control_hz", 200.0)
        self.declare_parameter("kp", [10.0, 60.0])
        self.declare_parameter("kd", [0.7, 2.0])
        # self.declare_parameter("kp", [0.0, 0.0])
        # self.declare_parameter("kd", [0.0, 0.0])
        self.declare_parameter("bringup_can", True)
        self.declare_parameter("zero_steps", 200)
        self.declare_parameter("zero_step_ms", 10)
        self.declare_parameter("zero_pos", [0.0, 0.0])
        self.declare_parameter("motor_directions", [1.0, -1.0])
        self.declare_parameter("motor_zero_offsets", [0.0, 0.0])

        iface        = self.get_parameter("can_interface").value
        joint_names  = list(self.get_parameter("joint_names").value)
        hz           = float(self.get_parameter("control_hz").value)
        self.kp      = [float(x) for x in self.get_parameter("kp").value]
        self.kd      = [float(x) for x in self.get_parameter("kd").value]
        zero_steps   = int(self.get_parameter("zero_steps").value)
        zero_step_ms = int(self.get_parameter("zero_step_ms").value)
        zero_pos     = [float(x) for x in self.get_parameter("zero_pos").value]
        do_can       = bool(self.get_parameter("bringup_can").value)

        if len(joint_names) != NECK_DOF:
            self.get_logger().fatal(
                f"joint_names 长度必须为 {NECK_DOF}, 收到 {joint_names}")
            raise SystemExit(1)
        if len(self.kp) != NECK_DOF or len(self.kd) != NECK_DOF:
            self.get_logger().fatal(
                f"kp/kd 长度必须为 {NECK_DOF}, 收到 kp={self.kp} kd={self.kd}")
            raise SystemExit(1)
        if len(zero_pos) != NECK_DOF:
            self.get_logger().fatal(
                f"zero_pos 长度必须为 {NECK_DOF}, 收到 {zero_pos}")
            raise SystemExit(1)

        # Validate before opening CAN or enabling either motor.
        self.coordinates = NeckCoordinates(
            self.get_parameter("motor_directions").value,
            self.get_parameter("motor_zero_offsets").value)
        if zero_steps < 1 or zero_step_ms < 1 or any(not math.isfinite(v) for v in zero_pos):
            raise ValueError("Invalid startup interpolation parameters")
        if any(not JOINT_LIMITS[n][0] <= zero_pos[i] <= JOINT_LIMITS[n][1]
               for i, n in enumerate(joint_names)):
            raise ValueError("zero_pos outside canonical joint limits")
        self.joint_names = joint_names
        self._mirror = CommandMirror(self, joint_names)
        self.zero_pos = zero_pos   # 回零目标 (rad)
        # 每个关节的限位 (按 joint_names 顺序)
        self.joint_limits = [JOINT_LIMITS.get(n, [-math.pi, math.pi])
                             for n in joint_names]

        # ==========================
        # CAN 拉起
        # ==========================
        if do_can:
            self.get_logger().info(f"拉起 CAN 口 {iface}...")
            ok = bringup_can(iface)
            self.get_logger().info(f"  {iface}: {'OK' if ok else '失败'}")
            if not ok:
                raise SystemExit(1)

        # ==========================
        # OpenArm 初始化
        # ==========================
        self.oa = OpenArm(iface, CAN_FD)
        self.oa.init_arm_motors(MOTOR_TYPES, MOTOR_IDS, MASTER_IDS)
        self.arm = self.oa.get_arm()

        # ==========================
        # 阶段 1: 使能 (复刻 on_activate)
        # ==========================
        self.get_logger().info("使能电机 (STATE callback → enable_all → recv_all) ...")
        self.oa.set_callback_mode_all(CallbackMode.STATE)
        self.oa.enable_all()
        time.sleep(0.1)
        self.oa.recv_all()

        # ==========================
        # 阶段 2: 回零 (复刻 return_to_zero)
        # ==========================
        self.get_logger().info("回零 ...")
        try:
            self._return_to_zero(zero_steps, zero_step_ms)
        except Exception:
            self.oa.disable_all()
            raise
        self.get_logger().info("回零完成, neck motors enabled")

        # ==========================
        # 状态缓冲
        # ==========================
        self.pos_states = [0.0] * NECK_DOF
        self.vel_states = [0.0] * NECK_DOF
        self.tau_states = [0.0] * NECK_DOF

        # 抱住目标 (无轨迹时维持) —— 初始为回零目标, 防止回零后被拉回 [0, 0]
        self.hold_pos = list(self.zero_pos)
        self.hold_vel = [0.0] * NECK_DOF

        # ==========================
        # 轨迹缓冲 (订阅回调写, 主循环读)
        # ==========================
        self._traj_lock = threading.Lock()
        self._traj = None    # {t0, durations[], positions[N][DOF], velocities[N][DOF]}

        # ==========================
        # ROS 接口
        # ==========================
        self.joint_pub = self.create_publisher(JointState, JS_TOPIC, 10)
        self.cmd_sub = self.create_subscription(
            JointTrajectory, TRAJ_TOPIC, self.command_callback, 10)
        # 在线增益 (面板 KP/KD 滑条): [kp1, kp2, kd1, kd2]
        self._gains_lock = threading.Lock()
        self.gains_sub = self.create_subscription(
            Float64MultiArray, GAINS_TOPIC, self.gains_callback, 10)

        # ==========================
        # 控制循环
        # ==========================
        period = 1.0 / hz if hz > 0 else 0.005
        self.timer = self.create_timer(period, self.control_loop)
        self.get_logger().info(
            f"neck_node ready: hz={hz}, joints={joint_names}, "
            f"limits={[('%.3f' % lo, '%.3f' % hi) for lo, hi in self.joint_limits]}, "
            f"sub={TRAJ_TOPIC}+{GAINS_TOPIC}, pub={JS_TOPIC}")

    # ==========================
    # 阶段 2: 回零 (openarm_simple_hardware.cpp:305-353)
    # ==========================
    def _return_to_zero(self, steps, step_ms):
        # Request feedback without first commanding motor zero.
        self.oa.refresh_all()
        time.sleep(0.01)
        self.oa.recv_all()
        motors = self.arm.get_motors()
        if len(motors) != NECK_DOF:
            raise RuntimeError("Startup requires both neck motor feedback values")
        start = [self.coordinates.logical_position(i, motors[i].get_position())
                 for i in range(NECK_DOF)]
        if not all(math.isfinite(v) for v in start):
            raise RuntimeError("Non-finite neck startup feedback")
        self.get_logger().info(f"Canonical startup (rad): {start}; target: {self.zero_pos}")
        duration = steps * step_ms / 1000.0
        for s in range(steps + 1):
            u = s / steps
            blend = 10*u**3 - 15*u**4 + 6*u**5
            rate = (30*u**2 - 60*u**3 + 30*u**4) / duration
            q = [start[i] + blend*(self.zero_pos[i]-start[i]) for i in range(NECK_DOF)]
            dq = [rate*(self.zero_pos[i]-start[i]) for i in range(NECK_DOF)]
            self.arm.mit_control_all([
                MITParam(self.kp[i], self.kd[i],
                         self.coordinates.motor_position(i, q[i]),
                         self.coordinates.signed(i, dq[i]), 0.0)
                for i in range(NECK_DOF)])
            self._mirror.push(q, dq, (0.,0.), self.kp, self.kd)
            self.oa.recv_all()
            time.sleep(step_ms / 1000.0)

    # ==========================
    # JointTrajectory 回调
    # ==========================
    def command_callback(self, msg: JointTrajectory):
        # 把消息里的 joint_names 顺序映射到本节点的 NECK_DOF 顺序
        try:
            idx = [msg.joint_names.index(n) for n in self.joint_names]
        except ValueError as e:
            self.get_logger().warn(
                f"轨迹 joint_names 不全: {msg.joint_names} ({e}); 忽略")
            return
        if not msg.points:
            self.get_logger().warn("收到空轨迹, 忽略")
            return

        durations, positions, velocities = [], [], []
        for p in msg.points:
            t_sec = p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            durations.append(t_sec)
            raw_pos = [float(p.positions[i]) for i in idx]
            # 关节限位
            raw_pos = [clamp(raw_pos[j], self.joint_limits[j][0],
                             self.joint_limits[j][1]) for j in range(NECK_DOF)]
            positions.append(raw_pos)
            if p.velocities:
                velocities.append([float(p.velocities[i]) for i in idx])
            else:
                velocities.append([0.0] * NECK_DOF)

        # 按时间递增排序
        order = sorted(range(len(durations)), key=lambda k: durations[k])
        durations  = [durations[k]  for k in order]
        positions  = [positions[k]  for k in order]
        velocities = [velocities[k] for k in order]

        with self._traj_lock:
            self._traj = {
                "t0":         time.monotonic(),
                "durations":  durations,
                "positions":  positions,
                "velocities": velocities,
            }
        self.get_logger().info(
            f"收到轨迹: {len(durations)} 点, 总时长 {durations[-1]:.3f}s, "
            f"目标终态={[round(positions[-1][j], 3) for j in range(NECK_DOF)]}")

    # ==========================
    # 在线增益回调: data = [kp1, kp2, kd1, kd2]
    # ==========================
    def gains_callback(self, msg: Float64MultiArray):
        d = [float(x) for x in msg.data]
        if len(d) != 2 * NECK_DOF or any(
                not math.isfinite(x) or x < 0.0 for x in d):
            self.get_logger().warn(
                f"gains 非法 (需 {2 * NECK_DOF} 个非负有限数): {d}; 丢弃")
            return
        with self._gains_lock:
            self.kp = [d[0], d[1]]
            self.kd = [d[2], d[3]]
        self.get_logger().info(
            f"gains 更新: kp={self.kp} kd={self.kd}")

    # ==========================
    # 主循环 (read + JointState 发布 + write)
    # ==========================
    def control_loop(self):
        # 1) 读电机状态 (复刻 read(): refresh_all + recv_all)
        self.oa.refresh_all()
        self.oa.recv_all()
        motors = self.arm.get_motors()
        for i in range(min(NECK_DOF, len(motors))):
            self.pos_states[i] = self.coordinates.logical_position(i, motors[i].get_position())
            self.vel_states[i] = self.coordinates.signed(i, motors[i].get_velocity())
            self.tau_states[i] = self.coordinates.signed(i, motors[i].get_torque())

        # 2) 发布 /joint_states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(self.joint_names)
        msg.position = list(self.pos_states)
        msg.velocity = list(self.vel_states)
        msg.effort   = list(self.tau_states)
        self.joint_pub.publish(msg)

        # 3) 采样当前目标: 有轨迹按时间插值, 否则抱住
        target_pos, target_vel = self._sample_target()
        self.hold_pos = target_pos
        self.hold_vel = target_vel

        # 4) 发 MIT PD (MITParam 顺序: kp, kd, q, dq, tau); 增益取快照(在线可变)
        with self._gains_lock:
            kp, kd = list(self.kp), list(self.kd)
        try:
            self.arm.mit_control_all([
                MITParam(kp[i], kd[i],
                         self.coordinates.motor_position(i, target_pos[i]),
                         self.coordinates.signed(i, target_vel[i]), 0.0)
                for i in range(NECK_DOF)])
            self._mirror.push(target_pos, target_vel, (0.,0.), kp, kd)
        except Exception as e:
            self.get_logger().error(f"mit_control_all 失败: {e}")

    def _sample_target(self):
        with self._traj_lock:
            traj = self._traj
        if traj is None:
            return list(self.hold_pos), list(self.hold_vel)

        t = time.monotonic() - traj["t0"]
        durs, pos, vel = traj["durations"], traj["positions"], traj["velocities"]

        if t <= durs[0]:
            return list(pos[0]), list(vel[0])
        if t >= durs[-1]:
            return list(pos[-1]), list(vel[-1])

        for k in range(1, len(durs)):
            if t <= durs[k]:
                t0, t1 = durs[k - 1], durs[k]
                if t1 <= t0:
                    return list(pos[k]), list(vel[k])
                a = (t - t0) / (t1 - t0)
                return ([pos[k - 1][j] + a * (pos[k][j] - pos[k - 1][j])
                         for j in range(NECK_DOF)],
                        [vel[k - 1][j] + a * (vel[k][j] - vel[k - 1][j])
                         for j in range(NECK_DOF)])
        return list(pos[-1]), list(vel[-1])

    # ==========================
    # shutdown (复刻 on_deactivate: disable_all x3)
    # ==========================
    def destroy_node(self):
        if hasattr(self, "_mirror"):
            self._mirror.push((0.,0.), (0.,0.), (0.,0.), (0.,0.), (0.,0.), enabled=False)
            self._mirror.close()
        try:
            self.get_logger().info("disable neck motors")
            for _ in range(3):
                try:
                    self.oa.disable_all()
                except Exception:
                    pass
                time.sleep(0.1)
                try:
                    self.oa.recv_all()
                except Exception:
                    pass
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = NeckNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # SIGINT 时 rclpy 的信号处理可能已关闭 context，再调 shutdown 会抛
        # RCLError（"already called"）——先检查再关，双保险包一层异常。
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()