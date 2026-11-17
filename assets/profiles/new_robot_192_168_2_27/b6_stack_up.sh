#!/usr/bin/env bash
# B6 真机验证整栈拉起 (body_actions 改造, 2026-09-05)
#
# ⚠⚠ 运行前确认 (缺一不可):
#   1. 机器人周围清场, 手臂/头部/腰腿活动范围内无人无物
#   2. 你人在现场, 面板急停按钮/物理急停可及
#   3. neck_node 启动会自动回零 —— 头会动!
#
# 用法: bash /ros_ws/scripts/b6_stack_up.sh [--with-hands] [--force] [--check-only]
# 默认不启动手部；--with-hands 在双臂归位后启动双手、使能并回到约定初始手型。
#   --force: 已有栈进程存活时先自动跑 b6_stack_down.sh 再起 (仍拦截 KER 摇操链)
# 日志: /tmp/b6_stack/*.log ; PID 记录: /tmp/b6_stack/pids.txt
set -euo pipefail

# Parse before sourcing ROS so help/invalid options never touch the stack.
WITH_HANDS=0
FORCE=0
CHECK_ONLY=0
usage() {
    echo "用法: bash /ros_ws/scripts/b6_stack_up.sh [--with-hands] [--force] [--check-only]"
    echo "  默认：仅启动原控制栈，不启动双手、不占用手部串口。"
    echo "  --with-hands：双臂归位后，启动双手、使能并平滑回到约定初始手型。"
    echo "  --check-only：只检查，不启动硬件；配合 --with-hands 检查手部串口与标定。"
}
for arg in "$@"; do
    case "$arg" in
        --with-hands) WITH_HANDS=1 ;;
        --force) FORCE=1 ;;
        --check-only) CHECK_ONLY=1 ;;
        -h|--help) usage; exit 0 ;;
        *) usage >&2; exit 2 ;;
    esac
done

. /ros_ws/scripts/b6_env.sh
LOG=/tmp/b6_stack
mkdir -p "$LOG"
PROCESS_HELPER=/ros_ws/scripts/b6_stack_processes.py

hand_preflight() {
    if [ "$WITH_HANDS" = 1 ]; then
        if [ -f /ros_ws/config/disable_hand_actions ]; then
            echo "[b6-stack] 手部禁用开关已开启，无法使用 --with-hands。" >&2
            return 1
        fi
        python3 /ros_ws/scripts/b6_hand_preflight.py
    fi
}

if [ "$CHECK_ONLY" = 1 ]; then
    if python3 "$PROCESS_HELPER" check; then
        hand_preflight
        if [ "$WITH_HANDS" = 1 ]; then
            echo "[b6-stack] 检查通过：无控制栈或 KER 冲突，双手串口与标定预检通过（未启动硬件）。"
        else
            echo "[b6-stack] 检查通过：无控制栈或 KER 冲突（本次不启动手部、不检查手部串口；未启动硬件）。"
        fi
        exit 0
    else
        status=$?
        echo "[b6-stack] 检查未通过（未启动硬件）。"
        exit "$status"
    fi
fi

if python3 "$PROCESS_HELPER" check-stack; then
    :
else
    status=$?
    [ "$status" = 1 ] || exit "$status"
    if [ "$FORCE" = 1 ]; then
        echo "[b6-stack] --force: 检测到已有栈存活，先收栈。"
        bash /ros_ws/scripts/b6_stack_down.sh
        # 收栈失败或仍有残留都不允许启动第二个 CAN 主控。
        python3 "$PROCESS_HELPER" check-stack
    else
        echo "[b6-stack] ⛔ 已有控制栈进程存活。请先 bash /ros_ws/scripts/b6_stack_down.sh"
        exit 1
    fi
fi
if python3 "$PROCESS_HELPER" check-ker; then
    :
else
    status=$?
    echo "[b6-stack] ⛔ KER 检查未通过。先停 KER 摇操链再起栈。"
    exit "$status"
fi

# Hand-port ownership/calibration checks apply only to --with-hands.
hand_preflight
: > "$LOG/pids.txt"

if [ "$WITH_HANDS" = 1 ]; then
    echo "[b6-stack] 模式：包含双手；双臂归位后使能双手并回到约定初始手型。"
else
    echo "[b6-stack] 模式：不启动手部，不占用手部串口。"
fi
echo "[b6-stack] 即将启动控制栈 (domain 101 + 白名单 XML)"
echo "[b6-stack] ⚠ 双臂将归位到面板姿态；neck_node 启动会归位到 (0°, 0°) (头会动) —— 5 秒倒计时, Ctrl-C 取消"
for i in 5 4 3 2 1; do echo "  $i..."; sleep 1; done

# 1) 双臂 bringup (真硬件; ⚠ launch 的 use_fake_hardware 默认是 true,
#    不传 false 会起 mock 假硬件 —— 真机 2026-09-05 踩过: 臂不动+手推发软)
#    ⚠ 真硬件激活时 on_activate 会 enable_all + 归位 (臂会动到 arm_home_pose.json 的面板姿态!)
#    ⚠ arm_type 必须与真机型号一致: 装的插件是 V10 ("OpenArm V10 Simple HW"),
#    launch 默认 openarm_v2.0(v20 xacro)会让 CM validate_storage 直接崩。
#    若日常用别的型号, 改这里。
ros2 launch openarm_bringup openarm.bimanual.launch.py \
    use_fake_hardware:=false arm_type:=v10 \
    startup_home_config:=/ros_ws/config/arm_home_pose.json > "$LOG/bringup.log" 2>&1 &
echo "bringup $!" >> "$LOG/pids.txt"
BRINGUP_PID=$!
echo "[b6-stack] bringup 启动中 (pid $BRINGUP_PID)"
# No fixed sleep: require both controller home targets and settled measurements.
if ! python3 /ros_ws/scripts/b6_wait_ready.py arms --timeout 120; then
    echo "[b6-stack] 双臂未完成初始归位；双手未启动。保留双臂当前保持，查看 bringup.log。"
    exit 1
fi

# 2) 脖子 (⚠ 启动自动归位到 J1=0°, J2=0°；zero_pos 单位 rad)
ros2 run neck_controll neck_node --ros-args \
    -p 'zero_pos:=[0.0, 0.0]' -p 'motor_directions:=[1.0, -1.0]' -p 'motor_zero_offsets:=[0.0, 0.0]' > "$LOG/neck.log" 2>&1 &
echo "neck $!" >> "$LOG/pids.txt"
echo "[b6-stack] neck_node 启动中 (pid $!, 归位目标 0°/0°)"

# 3) Optional controller-only hands; always after the arm readiness barrier.
# No glove/retargeter source is started by this mode.
if [ "$WITH_HANDS" = 1 ]; then
    export FT_CORE_PATH=/workspace/openarm_ros2/FT_core
    export PYTHONPATH="$FT_CORE_PATH${PYTHONPATH:+:$PYTHONPATH}"
    for side in left right; do
        port=$(python3 "$FT_CORE_PATH/hand_ports.py" "$side")
        python3 "$FT_CORE_PATH/hand_controller_node.py" --ros-args \
            -r "__ns:=/$side" -r "__node:=hand_controller_$side" \
            -p mock:=false -p "model_path:=$FT_CORE_PATH/orca_core/models/orcahand_v1_$side" \
            -p "port:=$port" -p playback_topic:=hand/playback_targets \
            -p startup_zero:=false -p startup_pose_file:=/ros_ws/config/hand_startup_pose.json \
            -p "startup_pose_side:=$side" -p startup_zero_duration:=1.0 -p playback_only:=true \
            > "$LOG/hand_$side.log" 2>&1 &
        echo "hand_$side $!" >> "$LOG/pids.txt"
        echo "[b6-stack] $side 手控制节点启动中；使能后从实测位置平滑回到约定初始手型 (pid $!)"
    done
    if ! python3 /ros_ws/scripts/b6_wait_ready.py hands --timeout 90; then
        echo "[b6-stack] 双手未到约定初始手型；回放服务/网页未启动。请查看 hand_left.log、hand_right.log 后收栈。"
        exit 1
    fi
else
    echo "[b6-stack] 未选择 --with-hands：跳过双手启动/归位，不占用手部串口。"
fi

# 4) eyou 腰/下肢 (can4, 使能保持)
#ros2 launch eyou_motor eyou_motor.launch.py > "$LOG/eyou.log" 2>&1 &
#echo "eyou $!" >> "$LOG/pids.txt"
#echo "[b6-stack] eyou 启动中 (pid $!)"

# 5) body_action_server
ros2 launch body_actions body_action_server.launch.py > "$LOG/server.log" 2>&1 &
echo "server $!" >> "$LOG/pids.txt"
echo "[b6-stack] body_action_server 启动中 (pid $!)"

# 6) 面板 (8090)
# ros2 run openarm_ui monitor_ui --ros-args -r __node:=openarm_ui \
#     -p port:=8090 -p advertise_ip:=192.168.8.30 -p hand_chain_side:=left \
#     -p hand_chain_auto_restart:=false > "$LOG/monitor.log" 2>&1 &
# echo "monitor $!" >> "$LOG/pids.txt"
# echo "[b6-stack] 面板启动中 (pid $!, http://192.168.8.30:8090)"

sleep 10
echo "[b6-stack] === 当前节点 ==="
ros2 node list
if [ "$WITH_HANDS" = 1 ]; then
    echo "[b6-stack] 就绪顺序: 双臂初始归位 → 双手使能到约定初始手型 → body_action_server / 8090（颈部保留原启动归位）"
else
    echo "[b6-stack] 就绪顺序: 双臂初始归位 → body_action_server / 8090（颈部保留原启动归位；本次未启动手部）"
fi
echo "[b6-stack] 起栈后请先跑 JTC QoS 订阅侧复验 (见操作手册 0 节)"
