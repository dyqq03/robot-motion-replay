# 独立运行与资源

整个文件夹是技能单元，入口为根目录 `SKILL.md`。复制到其他电脑或该电脑的技能目录即可；不是只复制 `assets/robot`。代码从自身位置定位资源，无原项目依赖或外部软链接。

## 环境

已验证 Python3.11、MuJoCo3.2.3。推荐隔离环境：

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python scripts/motion.py doctor
```

Python环境、系统OpenGL/EGL驱动不随技能打包。交互窗口需要桌面显示；无窗口渲染默认EGL，可用 `MUJOCO_GL` 选择兼容后端。无EGL/Mesa时安装系统驱动。MuJoCo版本改变后须实际重检旧证明，不能只改版本字段。FFmpeg由imageio-ffmpeg提供，中文字体已内置。离线流程无需ROS、FT SDK、串口或服务器。

|路径|用途|
|---|---|
|`assets/robot/URDF_90kg_with_hands`|活动URDF及网格|
|`assets/robot/generated/humanoid_shadow.xml`|活动视觉/命令MJCF|
|`assets/robot/generated/mesh_assets`|MuJoCo所需简化网格|
|`assets/robot/generated/collision_proxy_doc50_20261021`|分块凸碰撞模型和清单|
|`assets/robot/configs/design_collision_rules.json`|有几何证据的设计规则；原始全局规则另存|
|`assets/robot/configs/hand_models`|有效双手config/calibration原始YAML字节|
|`assets/robot/configs/*_motion_defaults.json`|当前生成参考，与标定零点不同|
|`assets/robot/backend`|真实后端快照，仅AST/内存消息桩执行|
|`assets/robot/evidence`|掌/拇指装配和碰撞代理证据|
|`assets/robot/generated/motion_packages`|六个重新核验的参考副本|
|`assets/reference_recordings`|一个完整原始真机招手录制，含HDF5、rosbag及下载校验清单；仅作历史参照|
|`assets/calibration_candidates/20261030`|待导入新标定，不是活动配置|

默认手臂角度（J1/J2/J3/J4/J5/J7，度）：左 `[8,-14.3,1.1,48.1,1.5,-12]`；右 `[-7.797832,14.190975,-0.893156,48.112546,-1.536982,12.380270]`。此表供理解，生成读取JSON完整精度。

普通设计不重建模型。URDF/网格/映射变化时，在新资源版本内运行内置 `build_mujoco_model.py --help` 和 `build_collision_proxy.py --help`，并读 `assets/robot/evidence/doc50_collision_geometry.md`。视觉模型变化会使旧侧车来源哈希失效，必须相应重建/复核，不只更新哈希。关节零点、baselink/视觉偏移和安装几何不是一回事。

`python scripts/self_test.py` 在临时目录检查生成、后端消息桩、手型采样和适配；`--full` 另做实际小动作全路径、视频和导出保护。交付验证证据在 `work/verification/`。`scripts/bundle_manifest.py` 维护固定资源哈希，不将运行日志和新动作纳入固定清单。

`python scripts/check_portability.py --full` 实际复制整个技能到临时位置，从不同工作目录运行，并用Python审计阻止读取原技能。`scripts/model_snapshot.py` 数值核对50个关节的映射、URDF和MJCF限位及当前标定；`--refresh-report` 仅在来源已核实后刷新描述性报告，改变报告会使旧动作证明失效，随后必须完整重检。

`python scripts/test_deployment.py` 用内置正式ZIP和临时目录测试离线上传安装器，不连接服务器。原始真机包的模型预览可运行：

```bash
python assets/robot/scripts/preview_motion_package.py assets/reference_recordings/server_20261016/fb_right_wave --recorded
```

原始录制不是新生成并通过当前检查的正式包，不应直接作为新动作的安全证明。六个生成参考和可运行示例的目录见 `assets/reference_catalog.md`。维护者使用 `scripts/prepare_examples.py` 会重建内置参考证明及画面，并清除旧视觉审阅；普通动作设计不运行它。
