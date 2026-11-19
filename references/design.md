# 设计和仿真迭代

六个参考副本见 `assets/bundle.json`：点头（纯颈部/短节奏）；五次走路摆臂（当前基准附近左右反相）；手指测试1（纯手部往返格式，不是自然手势目标）；右手带话（三类协同）；四指招手（轻摆/弯指相位）；灵巧手展示（抬手/看手/手型协同）。不自动沿用其旧基准注释和速度要求。

文件名、时长、预览入口见 [参考动作目录](../assets/reference_catalog.md)。已完成的最小组合示例在 `work/packages/fb_skill_small_conversation_v1`。新设计一般换新名字；明确要求同名替换时按 [修订流程](catalog-and-revisions.md) 保留旧内容，不直接改正式包。

## 通用设计JSON

`scripts/motion.py create` 接受 `assets/design_small_conversation.json` 所示结构：

- `package` 为草稿 `fb_..._vN`；`name/description` 说明语义。可加 `intent` 保存含义、关键姿态、掌面方向、参与侧和本次审阅重点，它随motion.json冻结。语义不清楚时针对性询问，明确时不额外确认。
- `duration_s/sample_hz` 定义时钟，建议100Hz，支持20–250Hz。
- `tracks` 只列参与链：`arm_left/arm_right/neck/hand_left/hand_right`。
- 每链 `mode` 为 `rest_relative` 或 `absolute`。所有输入是标定硬件关节度数，导出器转换单位。
- `keyframes` 的 `t` 严格递增，覆盖0至总时长；`q` 使用具名角度。相对模式未列关节为0偏移，绝对模式未列关节为当前参考。每个关键帧独立解释，不继承遗漏字段。
- 旧输入继续采用首尾速度/加速度为零的全局五次样条。显式 `kind=pass/stop` 或 `hold_s` 启用分段C2曲线：经过点共享速度/加速度，停留点为零，只有必要的语义位置停止。两种曲线都可能过冲，必须检查实际密集路径。
- 默认速度上限手臂25、颈部15、手指20°/s，靠头手臂15；`speed_caps_deg_s` 可按用户要求和依据覆盖。超速默认延长运动时间并记录；显式保护的静止段保持原时长。`extend_for_speed_limits=false` 直接报告超速。

名字以映射为准：手臂 `openarm_right_joint4` 等，颈部 `neck_joint1/2`；每手拇指mcp/abd/pip/dip，另外四指abd/mcp/pip，共16。名称不保证等同人的解剖位置，先看URDF父子链和关节轴。

复杂轨迹可编写本技能内的新草稿脚本，导入 `motion.designer()`、`package_io.write_draft()`。`t` 为递增秒，`q` 为N×46度数，顺序左臂6、右臂6、颈部2、左手16、右手16。未参与链保留 `designer.rest` 假设值，导出时不建命令流。元数据和完整调用见统一入口 `create()`；不调用历史批次生成器。

`Designer.fk/ik` 支持空间搜索，位置米；IK只是种子，不保证碰撞、辨识度或基准要求。肩肘腕正负与左右对称应以实际掌部位置/法向验证，不能只镜像数值。

## 姿态测量与搜索

```bash
python scripts/pose_tools.py measure work/drafts/包名 --time 2 --side right --out work/verification/pose.json --image work/verification/pose.jpg
python scripts/pose_tools.py solve work/pose_request.json --out work/verification/candidates.json --image work/verification/candidates.jpg
```

测量掌部/肘/腕/中指根部模型原点、掌面法向、肘高、小臂与掌部纵向夹角。标志点和法向局部轴在 `assets/posture_landmarks.json`，是模型几何定义，不是已测量的人体中心线。换手部装配时先对照画面核对。J7角度为零不代表这些方向天然共线；不要用改标定/网格的方法“修美观”。

`--image` 可选，直接生成当前姿态或三个候选的正侧面/双手近景，适合先看关键姿态再生成轨迹。没有经过实际查看的候选图不产生视觉通过记录。

求解请求例如：

```json
{
  "side": "right",
  "palm_position_m": [0.30, -0.27, 1.15],
  "palm_normal_world": [0, -0.8, -0.6],
  "j7_target_deg": 0,
  "j7_tolerance_deg": 2,
  "elbow_max_z_m": 1.16
}
```

坐标为当前模型世界坐标米，例子不是推荐的致敬目标。默认J7目标为种子值，默认种子为当前参考；J7带宽为硬求解边界，其余目标以误差/残差报告，可能无法同时满足。工具复用模型和检查器，返回三种种子结果与静态碰撞结果，不自动宣称目标可达/自然，不创建正式包。按位置误差、掌面方向、肘高和画面选择，再设计入场/送出/收势并完整检查。

## 经过、停留与倍速

关键帧可写 `{"t":2,"kind":"stop","hold_s":0.4,"q":{...}}`，表示在2秒到达并静止0.4秒；停留须在下一关键点之前结束。`pass` 不强制停止，但自然极值仍可能速度为零。首尾始终为零速度/加速度。多次招手用连续的往复经过点，不给每次中点都加停留。

设计中可增加：

```json
{"timing": {"speed_factor":1.5,"protected_holds_s":[[7,10]],"fixed_duration":false}}
```

时间区间使用倍速前的设计时钟。只有全部46列在区间内静止，才能保护这3秒；工具保留角度顺序、同步变换各链及手型语义时间。区间外按1.5倍缩放；限速延长时同样保留静止时长。不能保护一个仍有另一只手或头在动的区间。要加快单侧的一段而保持其他部位时序，直接改该track/短语的时间，不使用全时钟倍速。

`fixed_duration=true` 要求导出总长等于 `duration_s`；自动限速或倍速导致总长变化会拒绝生成，需减少幅度、重新排时序或明确放宽时长。导出时钟可不等间隔，sample_hz为平均值；检查器和播放器依据真实时间戳。运动段、语义停留和台词总时长分别考虑，不能把所有停留也随手加速。

## 可组合设计种子

`assets/motion_components.json` 提供小幅抬臂、平视转头和先张后柔弯短语；沿用相对参考和具名关节，不是未经检查的已认可动作库。它们用于减少重复代码，复杂空间手势仍按FK/IK和画面重塑。

```bash
python scripts/design_tools.py assets/design_composition_example.json --out work/drafts/composition.json
python scripts/motion.py create work/drafts/composition.json
```

composition用每个组件的 `start_s/end_s` 重叠不同部位。相同链重复组件要留独立静止间隔；不自动叠加冲突指令。手型段预留至少0.12秒错峰收尾。模板没有隐式控制左手，也不会自动给非参与侧增加小动作。普通tracks中每个关键帧独立解释，不能认为漏写的关节继承上一帧。

## 自然程度

讲话和PPT示意另见 [自然讲话手势](conversational-gestures.md)。通用入口支持 `hand_phrases`，可以先轻张再弯、左右错峰和细小指间延迟；仍需按当前标定/参考检查，不能直接照搬另一机器的绝对手型。

看全身正侧面和双手细节，判断语义、次数、节奏、掌心方向、肘腕姿态及视线先后。长手掌尤其要检查歪腕。需要语义停留时才停；连续表达不应在每个中间点刹停。基准相关动作围绕新基准做，明确目标动作重塑起落，不能先绕回旧基准。

粗调复用模型/检查器；快速检查失败是修改反馈。成形候选精查，全部流完成后查完整路径。调整路线或幅度直到可行，必要时换候选；不以放宽碰撞规则代替设计。

具体审阅及修改前后对照见 [视觉审阅](visual-review.md)。记录“在哪个时间、哪个部位、什么问题、如何改”，避免只写“自然、通过”。
