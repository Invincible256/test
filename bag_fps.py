#!/usr/bin/env python3
"""bag_fps.py —— 录制文件概要 + 每个话题的帧率（只读 metadata.yaml，几乎瞬间完成）

用法：
  python3 bag_fps.py ~/camera/bags/test1
  python3 bag_fps.py ~/camera/bags/test1 ~/camera/bags/test2     # 一次看多个
  python3 bag_fps.py ~/camera/bags/test1 --fps 60                 # 指定期望帧率（默认 60）
"""
import argparse
import os
import sys

try:
    import yaml
except ImportError:
    sys.exit("缺少 PyYAML：sudo apt install python3-yaml（ROS 环境里一般已经有了）")


def human(n):
    for unit in ("B", "KiB", "MiB", "GiB", "TiB"):
        if n < 1024 or unit == "TiB":
            return f"{n:.1f} {unit}"
        n /= 1024


def summarize(bag, expect_fps):
    bag = os.path.abspath(os.path.expanduser(bag)).rstrip("/")
    meta_path = os.path.join(bag, "metadata.yaml")
    if not os.path.exists(meta_path):
        print(f"\n{bag}：没有 metadata.yaml（录制可能没有正常结束），可以先执行 ros2 bag reindex {bag}")
        return
    info = yaml.safe_load(open(meta_path))["rosbag2_bagfile_information"]
    dur_s = info["duration"]["nanoseconds"] / 1e9
    size = sum(os.path.getsize(os.path.join(bag, f)) for f in os.listdir(bag) if os.path.isfile(os.path.join(bag, f)))
    topics = sorted(
        ((t["topic_metadata"]["name"], t["message_count"]) for t in info["topics_with_message_count"]),
        key=lambda x: x[0],
    )
    expected = round(dur_s * expect_fps)

    print(f"\n== {os.path.basename(bag)} ==  时长 {dur_s:.2f} s  大小 {human(size)}  消息 {info.get('message_count', '?')} 条")
    print(f"{'话题':<32}{'条数':>8}{'帧率 Hz':>10}{'比期望少':>10}")
    counts = {}
    for name, n in topics:
        counts[name] = n
        fps = n / dur_s if dur_s > 0 else 0.0
        short = ""
        if name.endswith("/image_raw") or name.endswith("/frame_meta"):
            short = f"{expected - n:>10d}"
        print(f"{name:<32}{n:>8d}{fps:>10.2f}{short}")

    l, r = counts.get("/stereo/left/image_raw"), counts.get("/stereo/right/image_raw")
    if l is not None and r is not None:
        diff = l - r
        verdict = "正常（只是开头 / 结尾的差别）" if abs(diff) <= 10 else "差得较多，建议检查帧号有没有跳变"
        print(f"左右图像相差 {diff:+d} 帧（约 {abs(diff) / expect_fps:.2f} s）→ {verdict}")
    print(f"（期望 {expect_fps:g} Hz × {dur_s:.2f} s ≈ {expected} 帧；帧率按整个录制时长计算，开头结尾的启动停止会让结果略低）")


def main():
    ap = argparse.ArgumentParser(description="录制文件概要 + 每个话题的帧率")
    ap.add_argument("bags", nargs="+", help="一个或多个 bag 目录")
    ap.add_argument("--fps", type=float, default=60.0, help="期望帧率")
    a = ap.parse_args()
    for b in a.bags:
        summarize(b, a.fps)


if __name__ == "__main__":
    main()
