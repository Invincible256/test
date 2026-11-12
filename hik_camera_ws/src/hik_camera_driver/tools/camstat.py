#!/usr/bin/env python3
"""camstat —— 一键显示相机状态

  ros2 run hik_camera_driver camstat.py              # 两台相机 + 配对统计（调用 /stereo/get_status）
  ros2 run hik_camera_driver camstat.py left         # 只看左相机（调用 /stereo/left/get_status）
  ros2 run hik_camera_driver camstat.py --watch 2    # 每 2 秒刷新一次，Ctrl+C 退出

直接用 ros2 service call 也能看到同样的内容，但换行会显示成 \\n，不方便阅读。
"""
import argparse
import os
import sys
import time

import rclpy
from std_srvs.srv import Trigger


def main():
    ap = argparse.ArgumentParser(description="一键显示相机状态")
    ap.add_argument("camera", nargs="?", default="", help="left / right；不填则显示两台相机和配对统计")
    ap.add_argument("--watch", type=float, default=0.0, help="每隔几秒刷新一次（0 = 只显示一次）")
    ap.add_argument("--timeout", type=float, default=5.0)
    a = ap.parse_args()
    service = f"/stereo/{a.camera}/get_status" if a.camera else "/stereo/get_status"

    rclpy.init()
    node = rclpy.create_node("camstat")
    cli = node.create_client(Trigger, service)
    try:
        if not cli.wait_for_service(timeout_sec=a.timeout):
            print(f"找不到服务 {service}：相机程序是否已经启动？这个终端是否执行了 source ~/camera/env.sh？")
            return 1
        while True:
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=a.timeout + 5)
            if a.watch > 0:
                os.system("clear")
            if fut.result() is None:
                print(f"调用 {service} 超时")
            else:
                print(time.strftime("%H:%M:%S"), service)
                print(fut.result().message)
            if a.watch <= 0:
                break
            time.sleep(a.watch)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
