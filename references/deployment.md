# 上传和替换服务器回放文件

仅在用户要求上传/替换时使用。当前任务已有授权就直接完成，不再人为增加确认。上传不执行动作，不改起栈/前端/控制配置。默认profile是已使用过的服务器地址和回放目录，但连接前核对它是否仍适用于当前任务。

`assets/server_profile.json` 是可编辑的远程目标配置。服务器 `/ros_ws/...` 是目标服务路径，不是本地代码依赖。`bind_interface` 可改为本机接口或空值。凭据通过SSH key/agent或环境变量 `ROBOT_SSH_PASSWORD` 提供，不保存密码到技能或报告。

## 预检与上传

先检查当前profile、用户授权范围与服务状态。需要查看时：

```bash
python scripts/deploy.py inspect
```

选择JSON是列表，每项为技能内正式文件夹路径 `package_path`，可选 `replaces` 指定确切服务器旧包名。没有 `replaces` 就只添加新版，不猜测要删哪个版本。例如：

```json
[
  {"package_path":"work/packages/fb_skill_small_conversation_v1"}
]
```

```bash
python scripts/deploy.py upload work/upload_selection.json
python scripts/deploy.py upload work/upload_selection.json --apply
```

第一条只检查本地正式证明、ZIP与文件一致性，无网络。第二条连接服务器，逐字节备份标签JSON到本地；需替换时先下载实际旧包归档并逐文件核对。所有新版上传到临时位置、验证后才安装、迁移相关标签并删除确切旧目录。不会把本地包内视频作为独立动作。

临近提交再次检查：HTTP无录制/回放/接近/暂停/试教，且ROS只读body服务为空闲。收栈后API不存在时，要求相关控制进程都已退出，不为上传重启控制栈。旧包仍被页面缓存时拒绝删除，先加载一个不被替换的包再重试，不需要播放。

远端上级 `AGENTS.md` 若存在，工具保存其内容并停在写入前，由执行者阅读、适配规则；不忽略服务器本地约定。相关确认若已由任务授权满足，不重复询问。

结果和旧文件本地备份在 `work/deployments/批次/`。当前上传工具需服务器Python3；替换时本地Python3.11和SSH/SCP可用。大包传输给出较长超时，但断网失败不自动删除未知目录。依据回执核对已安装内容后再继续，不能盲目循环上传/删除。脚本不会发送回放、使能、关节目标、起栈或收栈命令。

本技能交付只进行离线安装器测试，没有为验证此脚本连接真机。真实使用时仍需完成当次服务器状态/指纹核对，并更新当前动作目录中的包名、上传状态及回执。
