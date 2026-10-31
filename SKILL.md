---
name: robot-motion-replay
description: 为内置双臂、颈部和灵巧手机器人设计动作，在 MuJoCo 中迭代预览、检查整段轨迹并导出 OpenArm 回放包；也用于校准后采集参考手型和批量适配已有动作。适用于自然语言动作设计、参考动作平滑、手型更新及明确要求的回放文件上传。
---

# 机器人动作仿真与回放

这是可整体复制的独立技能。模型、网格、标定、离线播放器和参考动作都在本目录中。以下命令均以本目录为当前目录，使用当前环境的 `python`；不要寻找原开发项目或依赖某台电脑的绝对路径。

## 按任务读取

- 新建或修改动作：读 [设计与迭代](references/design.md) 和 [回放格式](references/package-format.md)。
- 校准变化、实时角度、起止手型或批量更新：读 [手部参考生命周期](references/hand-reference.md)。
- 环境、模型重建或迁移：读 [独立运行与资源](references/environment.md)。
- 用户明确要求上传/替换服务器文件：读 [上传替换](references/deployment.md)。本地生成不隐含上传、执行真机动作或修改起栈配置的授权。

先运行 `python scripts/motion.py doctor`。依赖缺失时按环境文档准备；手型/标定版本不一致时走手型更新流程，不能改哈希字段来伪装兼容。

## 当前快照与坐标

- URDF：`assets/robot/URDF_90kg_with_hands/urdf/humanoid_with_hands.urdf`。
- 活动 MJCF：`assets/robot/generated/humanoid_shadow.xml`。分块凸碰撞侧车独立保存，不改视觉模型和命令坐标。
- 真机/模型映射：`assets/robot/configs/{arm,neck,hand}_mapping.json`。左右正负号、偏置和手指轴不能按人类关节名称猜测。
- 当前初始姿态：`assets/robot/configs/{arm,neck,hand}_motion_defaults.json`。手臂 J1/J2/J3/J4/J5/J7；颈部首尾 J1=J2=0°；手部采用命名实测参考。实际采用值冻结到新包元数据，后续默认变化不能重新解释旧包。
- 手臂/颈部 HDF5 命令单位为弧度，灵巧手为标定关节角度的度数。电机编码器是另一坐标，换算只应用一次。

内置默认采用与2026-10-26参考手型一致的标定快照。另附 `assets/calibration_candidates/20261030/`：右手标定已变化，未静默设为当前默认。使用时先导入、重新采集和验证参考手型。本资源快照不代表当前真机状态。

## 设计循环

1. 明确语义、参与部位、节奏、时长和幅度。信息足够就推进，只询问影响动作含义的缺失信息。
2. 从当前参考设计连续轨迹。基准附近的小动作围绕新基准；明确手势保留语义目标并调整起落，不绕回历史初始姿态。J7优先保持参考，有实际姿态需要再改变。
3. 用统一入口或内置Python工具生成全新名字的草稿。支持纯颈部、纯手部、单/双臂和组合。未参与链为空；历史双臂格式中另一臂的保持指令必须声明。
4. 粗调复用模型和 `CollisionChecker.quick()`；成形候选用 `candidate()`，查看正侧面和手部细节继续修改。数值通过不等于自然，不在每个GUI帧做完整扫描。
5. 全部流形成后执行完整 `check`：实际时间轴、首尾、限位、速度、整段插值碰撞和消息桩正反回放。不确定也是未通过，继续调整或换候选；不得改例外/间隙凑通过。
6. `render` 生成全长视频、关键帧和轨迹。实际查看后用 `review` 记录具体观察。没看画面不能写已审阅，渲染成功不等于视觉评价通过。
7. `publish` 只接受当前有效的完整证明，创建新包和ZIP。相关数据/模型/标定/实现变化后重新检查，历史包不覆盖。

```bash
python scripts/motion.py create assets/design_small_conversation.json
python scripts/motion.py check work/drafts/fb_skill_small_conversation_v1
python scripts/motion.py render work/drafts/fb_skill_small_conversation_v1
# 实际查看 keyframes.jpg 和 preview.mp4 后记录观察：
python scripts/motion.py review work/drafts/fb_skill_small_conversation_v1 --reviewer assistant --notes '描述实际看到的节奏、首尾、掌心与手指姿态'
python scripts/motion.py publish work/drafts/fb_skill_small_conversation_v1
python scripts/motion.py preview work/packages/fb_skill_small_conversation_v1
```

JSON示例是设计起点，不保证所有修改都通过。正式输出默认在 `work/packages/`，失败试验在 `work/drafts/`。批量交付同时提供Markdown/JSON目录，含名称、时长、控制链、包/视频路径和检查状态。

内置同名示例已在 `work/packages/` 发布。直接预览即可；练习重新发布时先把设计JSON的 `package` 改为新的版本名，避免覆盖示例。

## 不可遗漏的约定

- 身体/腰腿流为空，不伪造实测反馈，不制造 `.db3`。只指挥本任务参与的链。
- 新手型来自用户认可姿态的稳定实测，不能用连接时任意当前姿态。保留原始值、修正值、标定版本和理由。修正范围以当前任务授权为准。
- 参考姿态通过不代表弯指/组合全路径通过。每个动作检查完整最终轨迹。
- 报告绑定模型、实际网格、映射、标定、规则和实现。装配规则必须有内置几何证据，不因初始重叠就排除。
- 直接设置qpos是运动学预览，不能提供物理碰撞阻挡。外部障碍、真实动力学、实际位置到第一帧和结束控制交接不在包内证明范围内。
- 参考包用于格式、节奏、姿态比较；旧默认、旧J7注释和某次倍速要求不是新动作的隐式要求。
