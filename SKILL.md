---
name: robot-motion-replay
description: 为内置双臂、颈部和灵巧手机器人设计动作，在 MuJoCo 中迭代预览、检查整段轨迹并导出 OpenArm 回放包；也用于校准后采集参考手型和批量适配已有动作。适用于自然语言动作设计、参考动作平滑、手型更新及明确要求的回放文件上传。
---

# 机器人动作仿真与回放

这是可整体复制的独立技能。模型、网格、标定、离线播放器和参考动作都在本目录中。以下命令均以本目录为当前目录，使用当前环境的 `python`；不要寻找原开发项目或依赖某台电脑的绝对路径。

## 按任务读取

- 新建或修改动作：读 [设计与迭代](references/design.md) 和 [回放格式](references/package-format.md)。
- 观察仿真、修正“不自然”：读 [视觉审阅与迭代](references/visual-review.md)。
- 校准变化、实时角度、起止手型或批量更新：读 [手部参考生命周期](references/hand-reference.md)。
- 环境、模型重建或迁移：读 [独立运行与资源](references/environment.md)。
- 换机、校准或方向/零位/限位疑问：先读 [方向、零位与限位对齐](references/alignment-and-limits.md)。区分电机驱动sign和模型sign，不能在多个层重复反向。
- 讲话/PPT伴随手势：另读 [自然讲话手势](references/conversational-gestures.md)，含先张后弯、双手错峰与近期五组轨迹模板。
- 用户明确要求上传/替换服务器文件：读 [上传替换](references/deployment.md)。本地生成不隐含上传、执行真机动作或修改起栈配置的授权。
- 动作目录、同名更新、上传状态：读 [目录与修订](references/catalog-and-revisions.md)。起栈/回放异常和事件兼容另读 [运行时边界](references/runtime-compatibility.md)。

先运行 `python scripts/motion.py doctor`。依赖缺失时按环境文档准备；手型/标定版本不一致时走手型更新流程，不能改哈希字段来伪装兼容。

模型、标定、参考或目标机器变化时，用 `scripts/state_audit.py` 保存和比较本地依赖。快照差异和实体核验分开记录，不能把磁盘文件一致当成运行进程已加载。

## 当前快照与坐标

- URDF：`assets/robot/URDF_90kg_with_hands/urdf/humanoid_with_hands.urdf`。
- 活动 MJCF：`assets/robot/generated/humanoid_shadow.xml`。分块凸碰撞侧车独立保存，不改视觉模型和命令坐标。
- 真机/模型映射：`assets/robot/configs/{arm,neck,hand}_mapping.json`。左右正负号、偏置和手指轴不能按人类关节名称猜测。
- 当前初始姿态：`assets/robot/configs/{arm,neck,hand}_motion_defaults.json`。手臂 J1/J2/J3/J4/J5/J7；颈部首尾 J1=J2=0°；手部采用命名实测参考。实际采用值冻结到新包元数据，后续默认变化不能重新解释旧包。
- 手臂/颈部 HDF5 命令单位为弧度，灵巧手为标定关节角度的度数。电机编码器是另一坐标，换算只应用一次。

内置默认采用与2026-10-26参考手型一致的标定快照。另附 `assets/calibration_candidates/20261030/`：右手标定已变化，未静默设为当前默认。使用时先导入、重新采集和验证参考手型。本资源快照不代表当前真机状态。

默认连接目标为 **192.168.8.30:2222，绑定wlo1**（`assets/server_profile.json`）。`192.168.2.27` 只作为 `assets/profiles/new_robot_192_168_2_27/` 的可选历史资料，不能把该机的颈部驱动 `[+1,-1]`、双手标定或起栈目标套给默认机器人。换机后逐项核对实体方向、逻辑零、模型偏置、限位和参考姿态；本地数值核对用 `python scripts/alignment_audit.py --out work/verification/alignment.json`，通过不代表实体已对齐。

## 设计循环

1. 明确语义、关键姿态、掌面方向、参与部位、节奏与时长，记入设计 `intent`。有歧义时做针对性澄清，信息足够直接推进。新手势先看关键姿态，再完善全段。
2. 从当前参考设计连续轨迹。基准附近的小动作围绕新基准；明确手势保留语义目标并调整起落，不绕回历史初始姿态。J7优先保持参考；零度或其他腕角是具体动作的约束，不推广为全局规则。
3. 用统一入口或内置Python工具生成独立草稿。`design_tools.py` 组合可重叠的控制链短语；`pose_tools.py` 测量掌面、肘高和腕部几何关系或搜索受限IK候选。支持纯颈部、纯手部、单/双臂和组合。未参与链为空；历史双臂格式中另一臂的保持指令必须声明。
4. 粗调复用模型和 `CollisionChecker.quick()`；成形候选用 `candidate()`，查看正侧面和手部细节继续修改。数值通过不等于自然，不在每个GUI帧做完整扫描。
5. 全部流形成后执行完整 `check`：实际时间轴、首尾、限位、速度、整段插值碰撞和消息桩正反回放。不确定也是未通过，继续调整或换候选；不得改例外/间隙凑通过。
6. `render` 全长视频后，用 `visual_review.py prepare` 准备关键/衔接时间采样。实际查看后记录带时间点的具体观察；未解决缺陷保持 `needs_changes`。说清看了哪些画面，不将“视频可解码”写成“完整观看”，不把助手判断等同用户认可。
7. `publish` 只接受当前有效的完整证明和显式视觉通过，创建包和ZIP。依赖或轨迹变化后重新检查。明确要求同名更新时用修订工具保留旧内容；用户认可、真机测试和上传回执分别记录。

```bash
python scripts/motion.py create assets/design_small_conversation.json
python scripts/motion.py check work/drafts/fb_skill_small_conversation_v2
python scripts/motion.py render work/drafts/fb_skill_small_conversation_v2
python scripts/visual_review.py prepare work/drafts/fb_skill_small_conversation_v2
# 实际查看后，按视觉审阅文档编写 assessment.json；有缺陷用 needs_changes：
python scripts/motion.py review work/drafts/fb_skill_small_conversation_v2 --status passed --reviewer assistant --assessment work/assessment.json --notes '本次实际观察与修改结论'
python scripts/motion.py publish work/drafts/fb_skill_small_conversation_v2
python scripts/motion.py preview work/packages/fb_skill_small_conversation_v2
```

JSON示例是设计起点，不保证所有修改都通过。正式输出默认在 `work/packages/`，失败试验在 `work/drafts/`。批量交付同时提供Markdown/JSON目录，含名称、时长、控制链、包/视频路径和检查状态。

旧版示例 `fb_skill_small_conversation_v1` 已在 `work/packages/` 发布。设计JSON采用新名字v2；再次练习时使用尚未存在的名字，避免覆盖示例。

## 不可遗漏的约定

- 身体/腰腿流为空，不伪造实测反馈，不制造 `.db3`。只指挥本任务参与的链。
- 新手型来自用户认可姿态的稳定实测，不能用连接时任意当前姿态。保留原始值、修正值、标定版本和理由。修正范围以当前任务授权为准。
- 参考姿态通过不代表弯指/组合全路径通过。每个动作检查完整最终轨迹。
- 起栈目标、界面归零/回初始、包内首尾、起播接近和结束恢复是独立层。颈部新包首尾为逻辑0°；本地播放器快照结束归位也已改为0°，服务器实际行为需另行核对。
- 报告绑定模型、实际网格、映射、标定、规则和实现。装配规则必须有内置几何证据，不因初始重叠就排除。
- 直接设置qpos是运动学预览，不能提供物理碰撞阻挡。外部障碍、真实动力学、实际位置到第一帧和结束控制交接不在包内证明范围内。
- 参考包用于格式、节奏、姿态比较；旧默认、旧J7注释和某次倍速要求不是新动作的隐式要求。
