# 校准与初始手型

标定0°、模型0°、动作初始手型是不同概念。安装/拉线改变后同一读数不保证同一物理姿态。实时测量只说明当前在哪里；只有用户认可的手型才可成为新参考。

## 导入新标定

把同一来源的左右 `orcahand_v1_left/right/{config,calibration}.yaml` 放进本技能。内置待导入版本在 `assets/calibration_candidates/20261030`：

```bash
python scripts/hand_reference.py import-calibration assets/calibration_candidates/20261030
python scripts/hand_reference.py import-calibration assets/calibration_candidates/20261030 --apply
```

第一条预览，第二条备份并更新本地YAML、映射电机ID/方向/比例和标定版本，不改几何、ROM、控制器或真机。ROM/拓扑变化需单独更新模型；USB改变先确认左右身份。导入后旧参考通常不再匹配，必须重新采集，不能修改旧参考哈希伪装兼容。已有包的依赖证明也失效，适配后重查。

## 采集认可姿态的实测反馈

先让用户调整到认可的初始手型。工具只读现有反馈，不主动移动、不使能、不回零、不抢串口：

```bash
python scripts/hand_reference.py capture --url http://192.168.8.30:8092/api/ui/snapshot --seconds 2 --out work/measurements/hands.json --accepted-initial-pose
```

该选项表示当前已有认可姿态的依据，不要求对已授权事项重复确认。HTTP适配器支持8092 `latest.feedback:left_hand/right_hand`：必须有效、年龄≤500ms、标定/USB身份一致。无服务时可导入真实读数文件或写只读反馈适配器，不能用目标指令代替实测。

采样文件结构为 `source=measured_feedback`、`calibration_files_sha256`、`samples`。每个sample含 `timestamp_s`、真实唯一 `sequence`、`hands.left/right`；每手含 `serial`、16个 `names`、对应 `motor_ids` 和标定度数 `joint_deg`，可含 `errors`。指纹为左右config/calibration实际SHA256。检查至少3个不同样本、≥0.2秒、有限值和完整电机映射，默认各关节极差≤0.6°，取中位数。不能复制一帧伪造稳定序列。`current_hand_raw_snapshot.json` 只有历史单次测量，供审计，不满足新采集要求。

## 候选、修正、验证、采用

```bash
python scripts/hand_reference.py prepare --samples work/measurements/hands.json --id hands_new_reference_v1 --accepted-initial-pose
python scripts/hand_reference.py validate work/hand_references/hands_new_reference_v1/reference.json
python scripts/hand_reference.py adopt work/hand_references/hands_new_reference_v1/reference.json
```

原始测量保留。超限/指间重叠时，依据当前授权做小幅搜索，把采用的**绝对角度**写进修正JSON，例如 `{"left":{"middle_abd":-2.0}}`，换新ID重新 `prepare --adjustments 文件`。默认可表达的修正幅度上限3°是工具参数，不代表用户已授权；有充分依据和授权可通过 `--max-adjustment-deg` 改变。不得修改限位、标定或碰撞间隙接受手型。

验证检查静态姿态与当前标定/分块碰撞模型；完整通过后才 `adopt`。档案保留原始值、采用值、差值、采样和证明哈希。采用只改本地生成默认，不改既有包或起栈目标。模型不含真实安装偏差时，仿真通过不保证真实指尖间隙。

## 批量适配动作

从当前动作目录选择每种动作的最新版，不选全部历史。选择JSON为列表，每项 `source` 指本技能内源包路径，`package` 指全新版本名。模式JSON按旧包名映射左右手，例如：

```json
{"fb_right_finger_greeting_20261022_v5":{"right":"rest_relative"}}
```

```bash
python scripts/hand_reference.py rebase --selection work/selection.json --modes work/retarget_modes.json
```

- `rest_relative`：轻微弯指/讲话小动作，整段保留相对旧参考的偏移，移到新参考附近。
- `absolute`：数字/点赞/握拳等目标手型，原时钟前后默认各22%用五次包络调整参考差，中段保留语义目标。逐包 `endpoint_blend_fraction` 可调。不是只改首末帧，也不增加回旧参考的过渡。
- 左右手可用不同模式。历史包没声明模式时工具拒绝猜测，由设计者判定。
- 标定版本变化必须核对中间目标，核对后加 `--calibration-targets-reviewed`；这不代替完整碰撞和画面检查。
- 无手指流跳过，未参与手保持空流。逐样本保留全部非手数据和所有时间戳。若新路径超速，先报告，再有意延长新版本时钟，不能暗改手臂/颈部节奏。

结果写 `work/hand_rebase_result.json`，草稿在 `work/drafts`。失败项保留并继续调整；成功草稿仍需 `motion.py check/render/review/publish`，不会自动ZIP或上传。新手型只能使新报告成立，不能复用旧通过字段。

## 真机部署

用户要求后才上传替换/迁移标签。起栈目标独立，用户要求才改。播放器实际位置到第一帧的平滑接近是运行时逻辑，不能代替离线参考更新。
