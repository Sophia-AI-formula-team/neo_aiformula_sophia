# 部署与验证

先读PROBLEM_ANALYSIS与AGENT_HANDOFF。只使用Unity MCP，禁止Computer Use。拿到另一对话明确释放的CPU/Editor窗口后执行；当前仅准备完成，不得把以下命令当作已执行。

1. 独立CPU窗口运行纯标准库测试：`python -B test_bbox_range.py`。18个测试方法覆盖大小边界、反比、目标阈值、缺失/旧证据、颜色一致、分辨率/相机变化、误差区间、原路径等价和worker单行接线。它不启动Unity、模型或图像处理。测试通过后在独立临时目录验证安装/回滚，不对live项目做测试夹具。
2. 获得Editor所有权后先经MCP确认clean Edit、worker=0，拍最新源/参数基线。保留Lane对话后续合法改动，不恢复旧整项目快照。确认实际显示面缩放=1、相机640×360及K/R/t与配置相符。
3. `python -B install.py --simulator <Simulator绝对路径> --action inspect --enable-trial`。仅预检worker、旧range/RGB检测器和本地payload哈希。任何漂移都拒绝；不可force，不可覆盖他人文件。
4. 仅在已获独占的clean Edit运行：`python -B install.py --simulator <路径> --action apply --enable-trial --exclusive-clean-edit --receipt <全新目录>/install.json`。两新文件+worker一处import；无C#、原semantic/LP/LYA或场景修改。receipt逐文件记录，worker备份在receipt同目录。没有`--enable-trial`则保留旧距离算法。
5. 用MCP刷新并检查console；在新case内启动既有真实RGB链，内外圈分别检查红灯接近/停车/绿灯恢复。保留逐帧框、signal_range/source/reason/target_reached、实际轨迹和控制日志。单独记录小框无距离时旧semantic的停车行为，禁止将其伪装成正确忽略误检。测试有时限，结束检查worker清理，并返还Editor/CPU。
6. 配置修改后须重启worker；不支持帧中热切换。距离模式故障/配置损坏/框太小均无distance，不自动退回旧射线，不保存旧range。关闭方法为配置enabled=false后重启（保留原色彩/控制逻辑）。

完整回滚（确保after文件未再修改）：`python -B install.py --simulator <路径> --action rollback --exclusive-clean-edit --receipt <原目录>/install.json`。只恢复该worker备份并移除两新增文件；Unity生成的.meta不删除，也不自动刷新。若配置已人为改过，hash拒绝回滚，先审查差异，不强制覆盖。部分安装失败也需按receipt检查，不自动重试。

制动仍有速度、延迟和非零不确定性保护。8px是观测到达阈值，不保证命令停车恰在8.8785m；实跑回执必须分别报告观察距离和实际车头距离。像素公式不使用灯位置/真值进行运行时距离输出，真值只能用于离线验收。
