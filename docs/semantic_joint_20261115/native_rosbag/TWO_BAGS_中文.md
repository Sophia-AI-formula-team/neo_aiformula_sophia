# 当前交付：内圈、外圈两个独立bag

2026-11-16。按用户纠正，当前交付分为两个独立ROS 2 bag，各包含该起点实际两圈的数据。此前“两场合成一个bag、工具和说明混入同一个ZIP”的交付停用；原始来源和验证记录保留作历史。

## 当前下载

- [内圈两圈ZIP，86,615,579字节](https://drive.google.com/file/d/197DT_eWKn7g9pQdmOpfaPCimQ2-EZ3tf/view?usp=drivesdk)
- [外圈两圈ZIP，91,214,113字节](https://drive.google.com/file/d/1TWG_QqmSNBD4xx09KDSdIjQ_F-BuEhy5/view?usp=drivesdk)

两份采用标准ZIP的LZMA无损压缩（method14），请用7-Zip、WinRAR或支持LZMA ZIP的解压程序。两份均已从云端完整下载并校验SHA256；ZIP内数据库的SHA256也与未压缩bag一致。两份ZIP不再分卷。

分享目录目前仅有这两个ZIP，旧分卷、脚本和说明等9个旧文件已清除。当前权限仍是受限访问；已连接的个人Drive接口不支持“任何人可查看”，用户要求只用已连接接口，因此未再请求网页登录，未声称公开分享完成。云端文件与清理结果见two_clean_bags_drive_receipt.json。

## 压缩包里有什么

`semantic_inner_2laps.zip`只有：

```text
semantic_inner_2laps/
  metadata.yaml
  semantic_inner_2laps.db3
```

`semantic_outer_2laps.zip`只有：

```text
semantic_outer_2laps/
  metadata.yaml
  semantic_outer_2laps.db3
```

ROS 2 bag本身是目录，metadata.yaml记录话题、类型、计数和时长，db3保存序列化消息；ZIP是传输用的外层压缩。两份必须分别解压后才能执行`ros2 bag play`。包内不附脚本、CSV、日志、校验JSON或其他说明。

## bag中保留的话题

只保留原MapAssist所需的九个订阅话题及相机raw/CameraInfo，共11个话题：

- `/aiformula_sensing/gyro_odometry_publisher/odom`
- `/original_processed_point_a`、`/original_processed_point_b`、`/original_processed_point_c`
- `/lane_stop_flag`、`/map_assist/branch_active`、`/map_assist/reset`
- `/map_assist/traffic_light_input`、`/map_assist/controller_cmd_vel`
- `/camera/image_raw`、`/camera/camera_info`

不含原合并包的重复JPEG话题、`/recorded/...`输出和额外元数据话题。每份保留一个显式冷启动reset；两场的时间轴均独立从约1秒开始，外圈Header时间与消息时间一致平移，不混用内圈地图。

| bag | 消息数 | 相机独立原图数 | 里程计数 | 已接受ABC帧数 |
| --- | ---: | ---: | ---: | ---: |
| 内圈两圈 | 10922 | 971 | 4233 | 960 |
| 外圈两圈 | 11722 | 1041 | 4525 | 1035 |

所有消息已读回；所有raw图像像素与原记录一致。既有图像来自保存的真实输入JPEG解码，不是overlay，不能恢复未保存的无损传感器像素。原包源码没有改变，原生ROS运行验收仍未执行。

回放参数、源码位置、工具和详细验证仍在本地/本文档目录外部保留，不塞进bag的压缩包。使用原包时，沿用前文的原订阅接口及单来源控制接线，1倍速播放。
