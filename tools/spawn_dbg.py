#!/usr/bin/env python3
"""直接调用 /spawn_entity 并打印 status_message，定位 spawn 失败原因。

用法: spawn_dbg.py <sdf 路径> <实体名>
"""
import sys
import time

import rclpy
from gazebo_msgs.srv import SpawnEntity


def main():
    path, name = sys.argv[1], sys.argv[2]
    with open(path, encoding="utf-8") as fh:
        xml = fh.read()

    rclpy.init()
    node = rclpy.create_node("spawn_dbg")
    cli = node.create_client(SpawnEntity, "/spawn_entity")
    if not cli.wait_for_service(timeout_sec=10.0):
        print("/spawn_entity 不可用")
        return 1

    req = SpawnEntity.Request()
    req.name = name
    req.xml = xml
    req.robot_namespace = ""
    req.reference_frame = "world"

    fut = cli.call_async(req)
    while rclpy.ok() and not fut.done():
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.01)

    resp = fut.result()
    print(f"success={resp.success}")
    print(f"status_message={resp.status_message!r}")
    node.destroy_node()
    return 0


if __name__ == "__main__":
    sys.exit(main())
