#!/usr/bin/env python3
"""bag_to_pairs.py —— 把录制的双相机数据按拍摄时刻配对，导出成左右并排的图片

用法（先 source ~/camera/env.sh，让 Python 能找到 hik_camera_interfaces 消息类型）：
  python3 bag_to_pairs.py ~/camera/bags/test1                      # 默认每 30 对导出 1 张，最多 100 张
  python3 bag_to_pairs.py ~/camera/bags/test1 --every 1 --max 20   # 连续导出前 20 对
  python3 bag_to_pairs.py ~/camera/bags/test1 --start 10 --every 1 --max 30   # 从第 10 秒开始连续导出 30 对
  python3 bag_to_pairs.py ~/camera/bags/test1 --png --separate     # PNG 无损；另外单独保存左、右原图

输出（默认在 <bag 目录>_pairs/ 下）：
  pair_00000.jpg ……   左右并排的图片，顶部写着这一对的时间差、帧号、拍摄时刻
  pairs.csv           全部配对结果（包括没有导出图片的），可以用表格软件打开

配对规则与 /stereo 协调节点相同：按图像时间戳（换算后的拍摄时刻）配对，时间差 ≤ 半个帧周期才算一对。
"""
import argparse
import csv
import datetime
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("缺少 OpenCV：请执行 sudo apt install python3-opencv（需要管理员权限）")

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit("找不到 ROS2 的 Python 库：请先执行 source ~/camera/env.sh")

# ROS Bayer 编码 → OpenCV 转换代码（与 cv_bridge 的对应关系相同；OpenCV 的 Bayer 命名比 ROS 错开一位）
BAYER_TO_BGR = {
    "bayer_rggb8": cv2.COLOR_BayerBG2BGR,
    "bayer_bggr8": cv2.COLOR_BayerRG2BGR,
    "bayer_gbrg8": cv2.COLOR_BayerGR2BGR,
    "bayer_grbg8": cv2.COLOR_BayerGB2BGR,
}


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def fmt_time(ns):
    t = datetime.datetime.fromtimestamp(ns / 1e9)
    return t.strftime("%H:%M:%S.") + f"{t.microsecond // 1000:03d}"


def to_bgr(msg, swap_rb=False):
    h, w, step = msg.height, msg.width, msg.step
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)[: h * step].reshape(h, step)
    enc = msg.encoding
    if enc in BAYER_TO_BGR:
        img = cv2.cvtColor(np.ascontiguousarray(buf[:, :w]), BAYER_TO_BGR[enc])
    elif enc == "rgb8":
        img = cv2.cvtColor(np.ascontiguousarray(buf[:, : w * 3]).reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    elif enc == "bgr8":
        img = np.ascontiguousarray(buf[:, : w * 3]).reshape(h, w, 3).copy()
    elif enc == "mono8":
        img = cv2.cvtColor(np.ascontiguousarray(buf[:, :w]), cv2.COLOR_GRAY2BGR)
    else:
        raise ValueError(f"不支持的图像编码 {enc}")
    if swap_rb:
        img = img[:, :, ::-1].copy()
    return img


def open_reader(uri, topics):
    storage = "mcap" if any(f.endswith(".mcap") for f in os.listdir(uri)) else "sqlite3"
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage), rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    present = [t for t in topics if t in types]
    if present:
        reader.set_filter(rosbag2_py.StorageFilter(topics=present))
    return reader, types, present


def read_stamps(uri, left_ns, right_ns):
    """第一遍：只读小消息 frame_meta，得到每一帧的时间戳和帧号；没有 frame_meta 时退回读图像头"""
    meta = [f"{left_ns}/frame_meta", f"{right_ns}/frame_meta"]
    reader, types, present = open_reader(uri, meta)
    use_meta = len(present) == 2
    if not use_meta:
        print("录制文件里没有 frame_meta，改为读取图像的时间戳（会慢一些）")
        reader, types, present = open_reader(uri, [f"{left_ns}/image_raw", f"{right_ns}/image_raw"])
    out = {0: [], 1: []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        side = 0 if topic.startswith(left_ns + "/") else 1
        msg = deserialize_message(data, get_message(types[topic]))
        fn = getattr(msg, "frame_num", -1)
        src = getattr(msg, "time_source", -1)
        out[side].append((stamp_ns(msg.header), fn, src))
    for s in out:
        out[s].sort()
    return out


def pair(left, right, half_ns):
    """与协调节点相同的配对：比较两个队首，时间差 ≤ 半个周期则配成一对"""
    i = j = 0
    pairs, unpaired = [], [0, 0]
    while i < len(left) and j < len(right):
        dt = right[j][0] - left[i][0]
        if abs(dt) <= half_ns:
            pairs.append((left[i], right[j]))
            i += 1
            j += 1
        elif dt > 0:
            unpaired[0] += 1
            i += 1
        else:
            unpaired[1] += 1
            j += 1
    unpaired[0] += len(left) - i
    unpaired[1] += len(right) - j
    return pairs, unpaired


def banner(width, lines, height=70):
    img = np.zeros((height, width, 3), np.uint8)
    for k, text in enumerate(lines):
        cv2.putText(img, text, (10, 28 + 30 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description="把双相机录制数据配对后导出为左右并排的图片")
    ap.add_argument("bag", help="ros2 bag 目录（例如 ~/camera/bags/test1）")
    ap.add_argument("--out", default="", help="输出目录，默认 <bag>_pairs")
    ap.add_argument("--left", default="/stereo/left", help="左相机命名空间")
    ap.add_argument("--right", default="/stereo/right", help="右相机命名空间")
    ap.add_argument("--fps", type=float, default=60.0, help="帧率，用于确定配对阈值（半个周期）")
    ap.add_argument("--start", type=float, default=0.0, help="从录制开始后第几秒开始导出")
    ap.add_argument("--every", type=int, default=30, help="每隔几对导出一张（1 = 连续导出）")
    ap.add_argument("--max", type=int, default=100, help="最多导出几张")
    ap.add_argument("--scale", type=float, default=1.0, help="图片缩放比例，例如 0.5")
    ap.add_argument("--png", action="store_true", help="保存为 PNG（无损，文件大）；默认 JPG")
    ap.add_argument("--separate", action="store_true", help="另外单独保存左、右原图")
    ap.add_argument("--swap-rb", action="store_true", help="交换红蓝通道（颜色反了时使用）")
    a = ap.parse_args()

    uri = os.path.abspath(os.path.expanduser(a.bag)).rstrip("/")
    out_dir = os.path.expanduser(a.out) if a.out else uri + "_pairs"
    os.makedirs(out_dir, exist_ok=True)
    half_ns = int(1e9 / a.fps / 2)

    print("第 1 遍：读取时间戳 ……")
    st = read_stamps(uri, a.left, a.right)
    pairs, unpaired = pair(st[0], st[1], half_ns)
    if not pairs:
        sys.exit(f"没有配对成功的帧（左 {len(st[0])} 帧，右 {len(st[1])} 帧）")
    dts = np.array([abs(r[0] - l[0]) / 1e6 for l, r in pairs])
    t0 = min(st[0][0][0], st[1][0][0])
    print(f"左 {len(st[0])} 帧，右 {len(st[1])} 帧；配对 {len(pairs)} 对，未配对 左 {unpaired[0]} 右 {unpaired[1]}")
    print(f"配对时间差 ms：平均 {dts.mean():.3f}  中位 {np.median(dts):.3f}  最大 {dts.max():.3f}（阈值 {half_ns / 1e6:.2f}）")

    # 选出要导出的对
    chosen = {}
    for k, (l, r) in enumerate(pairs):
        if (l[0] - t0) / 1e9 < a.start:
            continue
        if len(chosen) >= a.max:
            break
        if a.every <= 1 or k % a.every == 0:
            chosen[k] = (l, r)
    want = {0: {v[0][0]: k for k, v in chosen.items()}, 1: {v[1][0]: k for k, v in chosen.items()}}
    ext = ".png" if a.png else ".jpg"
    params = [] if a.png else [cv2.IMWRITE_JPEG_QUALITY, 95]

    print(f"第 2 遍：导出 {len(chosen)} 张图片到 {out_dir} ……")
    reader, types, _ = open_reader(uri, [f"{a.left}/image_raw", f"{a.right}/image_raw"])
    got, n_written = {}, 0
    rows = {}
    while reader.has_next() and n_written < len(chosen):
        topic, data, _ = reader.read_next()
        side = 0 if topic.startswith(a.left + "/") else 1
        msg = deserialize_message(data, get_message(types[topic]))
        k = want[side].get(stamp_ns(msg.header))
        if k is None:
            continue
        got.setdefault(k, {})[side] = to_bgr(msg, a.swap_rb)
        if len(got[k]) < 2:
            continue
        imgs = got.pop(k)
        l, r = chosen[k]
        left_img, right_img = imgs[0], imgs[1]
        if a.scale != 1.0:
            left_img = cv2.resize(left_img, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA)
            right_img = cv2.resize(right_img, None, fx=a.scale, fy=a.scale, interpolation=cv2.INTER_AREA)
        if left_img.shape[0] != right_img.shape[0]:  # 左右 ROI 不同时，按较小的高度对齐
            hh = min(left_img.shape[0], right_img.shape[0])
            left_img, right_img = left_img[:hh], right_img[:hh]
        dt_ms = (r[0] - l[0]) / 1e6
        side_by_side = cv2.hconcat([left_img, right_img])
        top = banner(side_by_side.shape[1], [
            f"pair {k}   R-L = {dt_ms:+.3f} ms   (t = {(l[0] - t0) / 1e9:.3f} s)",
            f"LEFT  {fmt_time(l[0])}  frame {l[1]}      RIGHT  {fmt_time(r[0])}  frame {r[1]}",
        ])
        name = f"pair_{k:05d}{ext}"
        cv2.imwrite(os.path.join(out_dir, name), cv2.vconcat([top, side_by_side]), params)
        if a.separate:
            cv2.imwrite(os.path.join(out_dir, f"pair_{k:05d}_left{ext}"), imgs[0], params)
            cv2.imwrite(os.path.join(out_dir, f"pair_{k:05d}_right{ext}"), imgs[1], params)
        rows[k] = name
        n_written += 1

    with open(os.path.join(out_dir, "pairs.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pair", "t_s", "left_stamp_ns", "right_stamp_ns", "right_minus_left_ms", "left_frame_num", "right_frame_num", "image"])
        for k, (l, r) in enumerate(pairs):
            w.writerow([k, f"{(l[0] - t0) / 1e9:.6f}", l[0], r[0], f"{(r[0] - l[0]) / 1e6:.3f}", l[1], r[1], rows.get(k, "")])
    print(f"完成：导出 {n_written} 张图片；全部配对结果见 {os.path.join(out_dir, 'pairs.csv')}")


if __name__ == "__main__":
    main()
