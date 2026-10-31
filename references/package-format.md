# 回放格式与验收

正式ZIP保留同名顶层目录。HDF5结构：

|路径|形状|单位|
|---|---|---|
|`arms/cmd_left,right`|N×6或0×6|rad|
|`arms/t_arm_cmd_l,r`|N或0|秒|
|`neck/q,t`|N×2/N或空|rad/秒|
|`hand_left/q,t`、`hand_right/q,t`|N×16/N或空|标定关节deg/秒|
|`arms/joint_position,t_arm_js`|空|不伪造实测反馈|
|`eyou/q,t`、旧 `hand/q,t`|空|不控制身体/腰腿和旧单手链|

双臂顺序J1/J2/J3/J4/J5/J7。手部 `joint_names` 明确16名字，`units=deg`。无参与链为空。历史双臂格式里单臂动作向另一臂发送基准保持，需在元数据明示。

`fullbody_meta.json` 描述链/时钟；`motion.json` 冻结语义、实际参考值、标定、速度及未参与链的假设；`manifest.json` 绑定每文件和模型/映射哈希。无需构造 `.db3` 或伪造实测rosbag。

```bash
python scripts/motion.py check work/drafts/包名 --stage quick --time 2
python scripts/motion.py check work/drafts/包名 --stage candidate --time 2
python scripts/motion.py check work/drafts/包名
python scripts/motion.py render work/drafts/包名
```

完整 `check` 执行路径认证及真实播放器AST/内存桩正反回放，没有ROS/串口初始化。交付要求：运动学报告；覆盖率1.0且当前有效的完整碰撞证明；后端正反消息证明；全长视频/关键帧/角度CSV与图；实际观看后绑定输入和画面的视觉记录；ZIP和目录。

碰撞报告绑定模型、实际网格、规则、映射/标定、MuJoCo版本及实现。失败/不确定保留草稿，不生成正式ZIP。只更新文件校验和不能使旧证明有效。修改轨迹/依赖后重新验证，发布后保持不可变。
