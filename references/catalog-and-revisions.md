# 动作目录与同名修订

## 独立记录状态

```bash
python scripts/catalog.py build --out work/catalog/actions
```

生成JSON和Markdown，默认扫描 `work/packages/`，可用 `--roots` 选择技能内批次目录。记录名称、包名、时长、控制链、机器人、参考、视频和当前视觉审阅；“最高命名版本”只按同动作ID/机器人分组的vN比较，不等同最佳或用户已认可。不同语义不能只因为名字近似自动合并，可在 motion.json 明确 `action_id`。

助手审阅从报告读取；用户认可和真机测试是另外的内容绑定事件：

```bash
python scripts/catalog.py record work/packages/包名 --kind user_accepted --evidence '引用用户实际认可此版的消息'
python scripts/catalog.py record work/packages/包名 --kind physical_tested --evidence '引用实际测试记录和结果'
```

没有相应证据就不运行这些命令。命令是操作者记录，不独立证明测试发生；同名包内容变化后旧认可不自动继承。撤回用 `--kind withdrawn`。事件在 `work/catalog/events.json`，不修改已发布包。

目录从 `work/deployments/*/receipt.json` 读取历史上传回执，并核对当前包的三份核心输入哈希；当前远端存在状态始终标为未查询。需要回答“现在服务器上有什么”时另做已授权的只读核对，不能用历史上传替代。

## 名字不变、内容可追溯

默认新名字发布。用户明确要求仍用v8等旧名字时，先在 `work/drafts/同名包` 生成和完整审阅新草稿：

```bash
python scripts/motion.py publish work/drafts/同名包 --replace-local
```

已有正式目录与ZIP必须对应。工具将旧目录/ZIP移入 `work/revisions/包名/时间_内容哈希_随机标识/`，保存修订回执，再安装新目录和ZIP。提交异常恢复原件；同名更新不在旧正式目录直接编辑。内部内容哈希区分修订，显示名保持不变。模型/标定变化仍需重新检查，归档旧证明不等于旧版仍适合当前机器。

替换服务器时选择项明确填写：

```json
[{"package_path":"work/packages/fb_example_v8","replaces":"fb_example_v8"}]
```

上传工具先下载并核对实际旧内容，再在空闲条件下替换，保留同名标签并更新时长。新名替换旧名同样支持。批次不允许互相覆盖或交叉换名。见部署文档；仅本地发布不会触发上传。
