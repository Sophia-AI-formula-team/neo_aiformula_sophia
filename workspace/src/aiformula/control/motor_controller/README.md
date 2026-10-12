# motor_controller 0.2.1

- 来源：用户指定的 `aiformula0124/aiformula/control/motor_controller/motor_controller/motor_controller.py`，原样导入；完整路径和 SHA256 见 `UPSTREAM.json`。
- 改动：在原轮速补偿前增加正、负方向分别拟合的 Sigmoid 反函数转向补偿；ROS 话题与 CAN `0x210` 接口保持不变（右、左轮各一个小端有符号 int32）。
- 验证：相同算法文件已在车上单包编译并核对安装哈希；用户反馈车辆可以转向。仓库元数据升为 0.2.1，车上此前构建的包元数据仍为 0.2.0。
- 已知问题：按当前车轮参数，零指令 `v=0, ω=0` 会输出右轮 −63、左轮 +63 RPM；直行也可能产生差速。本次保留指定算法，未修复此问题，零指令停车尚不成立。
- 上一版本保留在 PR #19；车上替换前备份为 `/home/control/aiformula_backups/20261012_motor_controller_usb`。历史上游哈希保留在 `UPSTREAM.json` 的 `upstream_files`。
