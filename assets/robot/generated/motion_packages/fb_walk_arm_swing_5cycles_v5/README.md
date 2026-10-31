# 走路轻摆臂五次 · 肘后收、手抬高初始姿态更新

原服务器包：`fb_walk_arm_swing_5cycles_v4`。新包：`fb_walk_arm_swing_5cycles_v5`。

走路摆臂和轻微伴随动作直接围绕新初始位姿设计；展示类动作保留主要手势位置，以连续轨迹从新姿态展开并返回，无额外旧初始姿态过渡。全部手部样本保持不变，颈部样本保持不变。原始包及ZIP留存本地备份。未参与控制链保持空流，腰腿无指令。

当前基准以 arm_reference.json / hand_reference.json 和 motion.json 中的快照为准。历史配置、关键点设计参数保留供溯源；实际角度及峰值速度以 fullbody.hdf5、trajectory_deg.csv 和 kinematic_validation.json 为准。
模型检查实际使用 motion.json 中的新双臂/双手假设姿态；检查器通用报告中的“calibrated zero”文字不代表本包使用旧手零位。
