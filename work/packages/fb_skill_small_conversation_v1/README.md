# 01 轻微转头与右手自然伴随

来源：动作文档第None页，小幅讲话伴随动作；示范相对当前参考的连续样条，并非未经检查即可发布的模板。时长 6.00 秒。
手臂：['right']；颈部：True；手指：['right']。
手臂/颈部命令单位为弧度，手指为标定关节角度“度”；实测反馈为空，不需要.db3。
首尾回到约定基准，有手指指令时首尾采用包内声明的实测参考手型。身体/腰腿命令为空。
单侧手臂动作沿用双臂格式，另一侧明确保持实测基准。其他未参与链不发送指令，碰撞检查采用motion.json声明的假设姿态。

本地预览：`python scripts/motion.py preview work/packages/fb_skill_small_conversation_v1`
视频 preview.mp4；正侧面关键帧 keyframes.jpg；角度轨迹 trajectory_deg.csv / trajectory.png。
完整路径检查见 collision_validation.json，装配规则见 collision_rules.json。
仅本地运动学预览与模型自碰撞检查；无真机、动力学、运行时起播过渡或外部障碍物验证。
