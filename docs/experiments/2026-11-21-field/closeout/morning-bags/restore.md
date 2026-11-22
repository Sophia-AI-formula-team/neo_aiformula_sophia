# 2026-11-21 录包恢复

四份原始录包各自打成一个 tar.zst，再切为最多 96 MiB 的分卷。每组都有 `.part-00` 起连续编号，不能单独解包。先在下载目录运行 `sha256sum -c parts-SHA256SUMS.txt` 校验分卷。

以 10:31 的录包为例，按文件名顺序拼接和解包。

```bash
cat 1121-04_rosbag2_2026_11_21-10_31_49.tar.zst.part-* > 1121-04_rosbag2_2026_11_21-10_31_49.tar.zst
tar --zstd -xf 1121-04_rosbag2_2026_11_21-10_31_49.tar.zst
```

`archive-SHA256SUMS.txt` 记录拼接后每份完整档案的 SHA-256，`original-SHA256SUMS.txt` 记录解包后的原始文件。录包内容、实际时间和初步状态见 `bag-catalog.md`。
