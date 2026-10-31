# Robot Motion Replay

独立的 Codex skill：基于随附机器人模型生成双臂、颈部和灵巧手动作，在 MuJoCo 中预览、检查完整轨迹并导出 OpenArm 回放包。

技能入口：[SKILL.md](SKILL.md)。整个仓库是技能单元，包含模型、网格、碰撞代理、标定配置、脚本、六个生成参考和一个完整真机原始录制，不依赖原开发项目目录。

## 使用

把本仓库完整克隆到 Codex 的 skills 目录，或让 Codex 直接读取本仓库的 `SKILL.md`。保留仓库目录结构。

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python scripts/motion.py doctor
```

环境要求为 Python 3.11 和可用的 OpenGL/EGL；离线流程不需要 ROS、串口或服务器。完整说明见 [独立运行与资源](references/environment.md)，动作示例见 [参考目录](assets/reference_catalog.md)。

## 标定与手型更新

动作初始手型与标定零位分别管理。新校准后，采集用户认可姿态的稳定实测反馈，保留原始值与采用值；经检查采用新参考，再按动作含义调整整条手指轨迹。详见 [手部参考生命周期](references/hand-reference.md)。

内置默认是与2026-10-26参考手型匹配的标定快照，另附2026-10-30候选配置。资源快照不代表当前真机状态。

## 验证

```bash
python scripts/self_test.py --full
python scripts/check_portability.py --full
python scripts/bundle_manifest.py
```

这些命令在本地测试生成、仿真、消息桩回放、导出与迁移，不发送真实关节指令。原始录制仅作历史参考；正式新包需有当前完整轨迹与视觉检查。上传及真机操作按当次用户要求执行。
