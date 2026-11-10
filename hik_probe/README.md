# hik_probe：海康 USB3 相机 SDK 探针程序

在**笔记本 + 虚拟相机**和**实验室主机 + 真机**上各运行一次，自动完成 `sdk_notes.md` 中需要实测的项目，并把原始数据和分析结果保存下来，方便对比。

不依赖 ROS，只需要 MVS SDK、CMake 和 Python 3（分析脚本只用标准库）。

## 一、它会测什么

| 测试 | 对应笔记中的问题 |
|---|---|
| 导出节点表（约 100 个节点的类型、读写属性、当前值、范围和选项）、GenICam XML、参数文件 | U10：真机有哪些节点；对比虚拟相机和真机的差异 |
| 不 Latch 连续读时间戳；InvalidateNodes 后读 | U2：读到的是缓存值还是实时值 |
| 静态下 10 Hz 对时 10 秒 | U1：时间戳单位是 ns 还是 tick |
| 设置帧率 30 和目标帧率后读 ResultingFrameRate | U8：它是实际帧率还是最大帧率 |
| 两台相机同时自由运行 60 秒，每帧记录设备时间戳、SDK 主机时间戳和三种系统时钟 | U3：帧时间戳和 DeviceTimestamp 是否同一时钟；U5：nHostTimeStamp 的单位和时钟；U15：带宽和丢帧 |
| 取流中以 20 Hz 对时，每秒记录一次状态（缓存积压、带宽、温度、USB 错误帧） | U2：取流中能否对时；U12：对时往返时间；初步的 U13：时钟漂移 |
| 取流中把 Width、OffsetX、PixelFormat、ExposureTime 写回当前值 | U10：哪些节点在取流中被锁定、返回什么错误码 |
| 停流后再对时一次、读 DeviceUptime | U14（部分）：停流后时间戳是否连续 |
| 按换算后的时间配对左右相机的帧 | 验证方案：最近帧时间差应在 0 到半个帧周期之间 |

## 二、安全性

- 只**临时**修改触发模式、采集模式、曝光、增益自动、帧率这几项，**结束时自动恢复原值**（`--no-restore` 可关闭）。
- **不会**执行 UserSetSave，相机断电后一定恢复原样。
- 导出节点表时只读取，**不会执行任何命令节点**（DeviceReset、TriggerSoftware 等）。
- 取流中的写入测试写的是**当前值**，不会改变实际设置。
- 按 Ctrl+C 可以提前结束，程序仍会按正确顺序停流、恢复参数、关闭相机。

## 三、编译

```bash
cd hik_probe
cmake -S . -B build
cmake --build build -j
```

编译报错时，把完整的错误信息发给我。

## 四、运行

**运行前关闭 MVS 客户端里的相机连接**，否则会提示相机被占用（返回 0x80000203）。

### 笔记本（虚拟相机）

```bash
./build/hik_probe --label laptop_virtual --ids left,right
python3 analyze.py probe_out/<生成的目录>
```

注意：虚拟相机的时间戳可能全部为 0，流异常回调也不支持。这些在分析结果里会明确标注出来，属于正常现象，**以真机结果为准**。

### 实验室主机（真机）

1. 先在 MVS 里用 Find Me 确认两台相机，分别把 Device User ID 设为 `left` 和 `right`（如果还没设置）。然后关闭 MVS 的连接。
2. 运行：

```bash
./build/hik_probe --label lab_real --ids left,right
python3 analyze.py probe_out/<生成的目录>
```

如果还没设置 User ID，去掉 `--ids left,right`，程序会自动选取前两台相机。

### 可选：测试时钟漂移（U13）

时钟漂移需要长时间运行才看得出来。真机上有空时可以跑 30 分钟：

```bash
./build/hik_probe --label lab_real_long --ids left,right --duration 1800
```

### 常用参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `--label` | 空 | 本次运行的标签，会写进目录名 |
| `--ids` | 全部（最多 2 台） | 按 User ID 或序列号选择相机 |
| `--fps` | 60 | 自由运行帧率 |
| `--duration` | 60 | 双相机取流时长（秒） |
| `--exposure` | 5000 | 曝光时间（微秒） |
| `--static` | 10 | 静态对时时长（秒） |
| `--latch-hz` | 20 | 取流中对时频率 |
| `--node-num` | 30 | SDK 图像缓存节点数 |

## 五、输出文件

每次运行生成一个目录：`probe_out/<日期时间>_<主机名>_<标签>/`

| 文件 | 内容 |
|---|---|
| `summary.txt` | 程序结束时打印的简要汇总 |
| `analysis.txt` | `analyze.py` 生成的详细分析 |
| `log.txt` | 完整运行日志 |
| `env.txt` | SDK 版本、系统信息、环境变量、`lsusb -t` |
| `devices.csv` | 枚举到的所有设备 |
| `config.csv` | 每一步设置的返回码和读回值 |
| `<相机>_nodes_before.csv` / `_nodes_after.csv` | 测试前后的节点表 |
| `<相机>_genicam.xml`（或 `.zip`） | 相机的 GenICam 描述文件 |
| `<相机>_features.ini` | SDK 导出的参数文件 |
| `<相机>_latch_static.csv` | 静态对时采样 |
| `<相机>_latch_run.csv` | 取流中的对时采样 |
| `<相机>_frames.csv` | 每一帧的帧号、时间戳、尺寸、像素格式等 |
| `<相机>_status.csv` | 每秒一次的状态 |

## 六、发给我对比

两台机器都跑完后，分别打包整个输出目录：

```bash
cd probe_out
tar czf laptop_virtual.tar.gz <笔记本的输出目录>
tar czf lab_real.tar.gz <主机的输出目录>
```

把两个压缩包发给我。也可以先自己对比节点表：

```bash
python3 analyze.py <笔记本的输出目录> <主机的输出目录>
```

结果写在第二个目录的 `compare_nodes.txt` 里。

## 七、实验室电脑不能导出文件时

结果文件留在实验室电脑上，只把**结论卡**带出来：

1. **笔记本上**：跑完虚拟相机后，把输出目录复制到仓库的 `reference/` 下并提交（`probe_out/` 被 .gitignore 忽略，`reference/` 不会）：
   ```bash
   mkdir -p reference && cp -r probe_out/<笔记本的输出目录> reference/laptop_virtual
   git add reference && git commit -m "add laptop virtual-camera probe result" && git push
   ```
2. **实验室电脑上**：`git pull`，编译，运行真机测试，然后在实验室电脑上直接做对比分析：
   ```bash
   python3 analyze.py probe_out/<主机的输出目录> reference/laptop_virtual
   ```
3. 屏幕最后会打印**结论卡**（约 40 行，同时保存为 `brief.txt`）。用 `cat probe_out/<主机的输出目录>/brief.txt` 显示出来，**拍照**发给我即可。

## 八、开发说明

- SDK 的函数原型、结构体和调用顺序都依据 `sdk_notes.md`（已逐项对照 `/opt/MVS/include` 核对）。
- 遵守笔记中的约束：每个进程只调用一次 Initialize / Finalize；枚举后立即按值拷贝设备信息；取流前设置缓存节点数；停止时先 join 取图线程，再 StopGrabbing；只在取图成功后调用 FreeImageBuffer。
- 取图线程只调用取图和释放；控制线程（对时、读状态）用每台相机一把锁串行化。这本身也是对“同一句柄多线程使用”（U6）的一次初步验证。
