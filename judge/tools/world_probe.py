#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""world_probe.py —— 打印 Gazebo 世界里的模型数量，用作"世界是否加载完"的门控。

用法：
    python3 tools/world_probe.py [等待服务的秒数]
输出：
    一行整数（模型数）；连不上或超时打印 0。

为什么需要它：robocup.world 是一座完整城市，本机加载要数十秒到数分钟。
官方 robocup.launch 会立刻并发 spawn 6 架无人机，撞上未就绪的世界就会
"Spawn service timed out ... entity to appear"，6 架全废。裁判端必须先等
模型数稳定，再逐架 spawn。
"""
import sys

try:
    import rospy
    from gazebo_msgs.srv import GetWorldProperties
except Exception:
    print(0)
    sys.exit(0)


def main():
    wait = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    rospy.init_node("zzu_world_probe", anonymous=True, disable_signals=True)
    try:
        rospy.wait_for_service("/gazebo/get_world_properties", timeout=wait)
        proxy = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)
        print(len(proxy().model_names))
    except Exception:
        print(0)


if __name__ == "__main__":
    main()
