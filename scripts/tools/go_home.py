#!/usr/bin/env python3
"""单点导航: 回起点 (0,0,0), 带 150s 超时。"""
import time

import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def main():
    rclpy.init()
    nav = BasicNavigator()
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = nav.get_clock().now().to_msg()
    goal.pose.position.x = 0.0
    goal.pose.position.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0
    nav.goToPose(goal)

    t0 = time.time()
    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb and hasattr(fb, 'distance_remaining'):
            print(f'距目标 {fb.distance_remaining:.2f} m  ({time.time()-t0:.0f}s)', flush=True)
        if time.time() - t0 > 150:
            print('超时 150s, 取消', flush=True)
            nav.cancelTask()
            break
        time.sleep(1.0)

    result = nav.getResult()
    ok = result == TaskResult.SUCCEEDED
    print(f'回起点: {"成功" if ok else f"失败({result})"}  耗时 {time.time()-t0:.1f}s')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
