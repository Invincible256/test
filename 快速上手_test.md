# 海康双相机 · 快速上手（实验室主机）

> 只讲怎么用。命令直接复制粘贴即可。每一步都写了“应该看到什么”。

---

## 0. 第一次使用：准备（只做一次）

**① 编译**（代码有更新时也要重新做）

```bash
cd ~/camera/hik_camera_ws_step4_5/hik_camera_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

最后看到 `Summary: 2 packages finished` 就是成功。

**② 做一个环境脚本**，以后每个新终端只要执行一行：

```bash
cat > ~/camera/env.sh << 'EOF'
source /opt/ros/jazzy/setup.bash
source ~/camera/hik_camera_ws_step4_5/hik_camera_ws/install/setup.bash
EOF
```

以后**每打开一个新终端，先执行**：

```bash
source ~/camera/env.sh
```

**③ 关掉 MVS 客户端**（MVS 连着相机时，程序打不开相机）。

---

## 1. 启动程序（终端 A，一直开着）

```bash
source ~/camera/env.sh
ros2 launch hik_camera_driver stereo.launch.py config:=lab_real.yaml
```

✅ **应该看到**：左右相机各自打印打开信息，最后一行是 `就绪，等待 ~/start`。

❌ 如果看到“打开相机失败”：检查 MVS 是否已经关闭，相机 User ID 是否为 left / right。

---

## 2. 开始出图（终端 B）

```bash
source ~/camera/env.sh
ros2 service call /stereo/start std_srvs/srv/Trigger
```

✅ **应该看到**：`success: True`，以及 `左[成功…] 右[成功…]`。

之后，终端 A **每 10 秒**会打印状态行：

```
状态：取流中 新增帧 600，帧号跳变 0，丢包帧 0，超时 0，错误 0 ……
```

**新增帧约 600 = 帧率 60 Hz，正常。** 以这一行为准，**不要用 `ros2 topic hz` 测图像话题**，它测大图像不准，会偏低。

---

## 3. 看画面

```bash
ros2 run rqt_image_view rqt_image_view
```

左上角的下拉框选择 `/stereo/left/image_raw` 或 `/stereo/right/image_raw`。

👉 **用红色、蓝色的东西对着相机**，看颜色对不对。颜色反了就告诉我。

---

## 3b. 演示窗口（比 rqt 流畅，适合演示）

启动时加 `demo:=true`：

```bash
ros2 launch hik_camera_driver stereo.launch.py config:=lab_real.yaml demo:=true
```

然后执行第 2 步的 start，会弹出一个左右并排的窗口，顶部显示两台相机的帧率、左右时间差。**先用鼠标点一下窗口，让它获得焦点，按键才有效**：

| 按键 | 作用 |
|---|---|
| `+` / `-`（或鼠标滚轮） | 放大 / 缩小 |
| `0` | 恢复不放大 |
| `w` `a` `s` `d`（或方向键） | 放大后移动视野 |
| `1` `2` `3` `4` | 显示分辨率 100% / 75% / 50% / 25%（**越低越省 CPU**，默认 50%） |
| `[` `]` | 显示帧率 -5 / +5（默认 30） |
| `v` | 切换：并排 / 只看左 / 只看右 |
| `r` | 画面旋转 180°（只影响显示） |
| `f` | 全屏 |
| `p` | 截图（保存到 `~/camera/snapshots/`） |
| `h` | 显示帮助 |
| `q` | 关闭窗口（相机继续工作）。**请用 q 关闭**，点窗口右上角的 × 有可能会重新弹出 |

**边演示边录制时**：把分辨率调到 `3` 或 `4`、帧率调到 30 以下，并看终端状态行的“帧号跳变”保持为 0。**正式采集训练数据时不要加 `demo:=true`。**

## 4. 看状态（一键，适合拍照）

```bash
ros2 run hik_camera_driver camstat.py              # 两台相机 + 左右配对统计
ros2 run hik_camera_driver camstat.py left         # 只看左相机
ros2 run hik_camera_driver camstat.py --watch 2    # 每 2 秒自动刷新，Ctrl+C 退出
```

每台相机显示一块，示意：

```
== MV-CB013-A0UC-S serial=xxxx user_id=left ==
状态      取流中 | 掉线 否 | 启动配置 OK
帧率      实测 59.99 fps（近 10 s）| 设定 60.00 | 相机 ResultingFrameRate 60.00
帧数      本次取流累计 36012 帧
丢帧      帧号跳变 0 | 丢包帧 0 | 取图超时 0 | 取图错误 0
曝光      5000.0 us | 增益 0.00 dB（相机当前值）
ROI       1280x1024 +0+0（最大 1280x1024）
像素格式  BayerGB8 -> bayer_gbrg8
温度      38.1 °C | 带宽 78644 KB/s（约 76 MB/s）
对时      正常 +5.2 ppm rms 7 us | 接收时间帧 0 | 临时模型帧 0
```

| 看什么 | 正常 |
|---|---|
| 实测帧率 | 约 60 |
| 丢帧那一行 | 全部为 0 |
| 掉线 | 否 |
| 对时 | 正常 |
| 配对（最上面的 stereo 部分） | 平均和最大都 ≤ 8.33 ms |

## 5. 修改参数（出图中也能改）

下面以左相机为例；改右相机，把 `left` 换成 `right`。

**曝光**（单位 µs，立即生效）：

```bash
ros2 service call /stereo/left/set_exposure hik_camera_interfaces/srv/SetValue "{value: 3000.0}"
```

**增益**（单位 dB，立即生效）：

```bash
ros2 service call /stereo/left/set_gain hik_camera_interfaces/srv/SetValue "{value: 2.0}"
```

**ROI**（画面会中断约 0.5 秒）：

```bash
ros2 service call /stereo/left/set_roi hik_camera_interfaces/srv/SetRoi "{width: 640, height: 480, offset_x: 320, offset_y: 272}"
# 改回全幅：
ros2 service call /stereo/left/set_roi hik_camera_interfaces/srv/SetRoi "{width: 1280, height: 1024, offset_x: 0, offset_y: 0}"
```

✅ **应该看到**：`success: True`，以及读回的实际值。在 rqt_image_view 里能看到亮度或尺寸的变化。

**帧率**（两台都要改，立即生效）：

```bash
ros2 param set /stereo/left fps 30.0
ros2 param set /stereo/right fps 30.0
```

**白平衡**：见老师要求的白平衡校准（`white_balance_once`），校准后把数字写进配置文件。

**其他任何参数（对比度、Gamma、锐化等）**：用通用命令，格式和配置文件 `camera_settings` 一样（`节点名:类型:值`）：

```bash
# 先读：看当前值和可选范围
ros2 service call /stereo/left/read_node hik_camera_interfaces/srv/ReadNode "{name: 'ContrastRatio'}"
# 再写：
ros2 service call /stereo/left/set_node hik_camera_interfaces/srv/SetNode "{spec: 'ContrastRatioEnable:bool:true'}"
ros2 service call /stereo/left/set_node hik_camera_interfaces/srv/SetNode "{spec: 'ContrastRatio:int:60'}"
```

常用的节点名：

| 功能 | 节点 | 说明 |
|---|---|---|
| 对比度 | `ContrastRatioEnable`（bool）、`ContrastRatio`（int） | 先打开开关，数值才生效 |
| Gamma | `GammaEnable`（bool）、`GammaSelector`（enum）、`Gamma`（float） | |
| 锐化 | `SharpnessEnable`（bool）、`Sharpness`（int） | |
| 数字增益 | `DigitalShiftEnable`（bool）、`DigitalShift`（float） | 噪点比模拟增益大，一般不用 |
| 黑电平 | `BlackLevelEnable`（bool）、`BlackLevel` | |
| 降噪 | `DigitalNoiseReductionMode`（enum） | |
| 翻转 | `ReverseX`、`ReverseY`（bool） | 取流中可能被锁定，要先 stop。翻转后 Bayer 排列可能改变，要重新检查颜色 |

节点名以 `read_node` 实际读到的为准，读不到就到 MVS 属性树里查准确名称。⚠️ **这些设置会改变像素值：采集训练数据时，两台相机必须设置一样，并且采集和推理时保持不变**。

**想让参数每次启动都生效**：编辑配置文件 `src/hik_camera_driver/config/lab_real.yaml`：曝光、增益、ROI 改 `exposure_us`、`gain_db`、`roi_*`；对比度等其他参数，按上面的格式加进 `camera_settings` 列表（两台相机共用，保证一致）。改完重启程序即可，不需要重新编译。

---

## 6. 录制

**① 先看磁盘空间**（两台相机每分钟约 9.4 GB）：

```bash
df -h ~
```

**② 录制**（先录 10 秒试试，按 Ctrl+C 结束）：

```bash
mkdir -p ~/camera/bags
ros2 bag record -o ~/camera/bags/test1 \
  /stereo/left/image_raw /stereo/right/image_raw \
  /stereo/left/frame_meta /stereo/right/frame_meta /stereo/sync_status
```

**③ 检查录到了什么**：

```bash
ros2 bag info ~/camera/bags/test1
```

✅ **应该看到**：两路 image_raw 的条数接近（约 60 条/秒 × 录制秒数）。

---

## 7. 停止和退出

```bash
ros2 service call /stereo/stop std_srvs/srv/Trigger    # 停止出图（程序还在，可以再 start）
```

终端 A 按 **Ctrl+C** 退出整个程序。之后可以重新启动，也可以打开 MVS。

---

## 8. 测试清单：边跑边打勾

| # | 目标 | 怎么验证 | 结果 |
|---|---|---|---|
| 1 | 能启动 | 第 1 步看到“就绪” | ☐ |
| 2 | 两台相机同时出图 | 第 2 步 success: True | ☐ |
| 3 | 帧率 60 | 状态行新增帧约 600 | ☐ |
| 4 | 画面和颜色正确 | 第 3 步，红蓝物体 | ☐ |
| 5 | 曝光、增益能改 | 第 5 步，亮度变化 | ☐ |
| 6 | ROI 能改、能改回 | 第 5 步，尺寸变化 | ☐ |
| 7 | 左右同步 | 第 4 步，配对平均和最大都 ≤ 8.33 ms | ☐ |
| 8 | 能录制 | 第 6 步，bag info 条数正常 | ☐ |
| 9 | 长时间稳定 | 出图 30 分钟后看第 4 步，帧号跳变 = 0 | ☐ |
| 10 | 能正常退出、重启 | 第 7 步后重新执行第 1、2 步 | ☐ |

**哪一项不通过，就把那一步的终端输出拍照发给我。**

---

## 9. 遇到问题

| 现象 | 处理 |
|---|---|
| 打开相机失败 / 0x80000203 | 关掉 MVS 和其他用到相机的程序，Ctrl+C 退出后重新启动 |
| `ros2: command not found` 或者找不到包 | 这个终端没有执行 `source ~/camera/env.sh` |
| start 返回 False | 看终端 A 里的红色错误信息，拍照发给我 |
| 状态行出现“已掉线” | 检查 USB 线，重新插好后 Ctrl+C 重启程序 |
| 新增帧明显少于 600 | 执行第 4 步，拍照发给我 |
