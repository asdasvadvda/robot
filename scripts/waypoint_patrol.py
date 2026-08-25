#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
巡检航点自主巡逻: 基于 nav2_simple_commander.BasicNavigator, 依次到达地图上的
航点, 每点停留 DWELL 秒, 全部走完后导航回起点。Nav2 会自行规划路径并避障。

航点来源: waypoints.yaml (默认 /home/ubuntu/my_ros2_ws/scripts/waypoints.yaml), 由
record_waypoint.py 记录生成; 没有该文件时退回内置 WAYPOINTS 占位列表。

用法(容器内, 先起 nav_bringup.launch.py):
    python3 /home/ubuntu/my_ros2_ws/scripts/waypoint_patrol.py --initial-pose 0 0 0
    python3 /home/ubuntu/my_ros2_ws/scripts/waypoint_patrol.py --initial-pose 0 0 0 --dwell 3.0
    python3 /home/ubuntu/my_ros2_ws/scripts/waypoint_patrol.py --list-waypoints   # 只打印航点, 不移动

坐标是**地图(map)系**下的位置。初始位姿/起点默认 (0,0,0) (建图起点)。
"""
import argparse
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# ---------------------------------------------------------------------------
# 默认参数
# ---------------------------------------------------------------------------
WAYPOINTS_FILE = '/home/ubuntu/my_ros2_ws/scripts/waypoints.yaml'

# 无 waypoints.yaml 时的占位回退(仅提醒用, 不是真实巡检点)
FALLBACK_WAYPOINTS = [
    (1.0, 0.0, 0.0),
    (1.0, 1.0, 90.0),
]

DWELL = 3.0            # 每个航点停留秒数
HOME = (0.0, 0.0, 0.0) # 走完全部航点后返回的起点(地图系)
WAYPOINT_TIMEOUT = 120.0


def load_waypoints(path):
    """从 yaml 读航点, 支持 [x, y, yaw] 或 {x,y,yaw} 两种形式"""
    if not os.path.exists(path):
        return None
    import yaml
    with open(path, 'r') as f:
        data = yaml.safe_load(f) or {}
    wps = data.get('waypoints') or []
    out = []
    for w in wps:
        if isinstance(w, (list, tuple)):
            out.append((float(w[0]), float(w[1]), float(w[2])))
        else:
            out.append((float(w['x']), float(w['y']), float(w['yaw'])))
    return out


def make_pose(x, y, yaw_deg, clock, frame='map'):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = clock.now().to_msg()
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    yaw = math.radians(float(yaw_deg))
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def run_patrol(nav, waypoints, timeout, dwell):
    results = []
    for i, (x, y, yaw) in enumerate(waypoints):
        goal = make_pose(x, y, yaw, nav.get_clock())
        nav.get_logger().info(f'[航点 {i+1}/{len(waypoints)}] -> ({x}, {y}, {yaw}°) 出发')
        nav.goToPose(goal)

        t0 = time.time()
        last_log = 0.0
        while not nav.isTaskComplete():
            if time.time() - t0 > timeout:
                nav.get_logger().warn(f'[航点 {i+1}] 超时 {timeout:.0f}s, 取消')
                nav.cancelTask()
                break
            fb = nav.getFeedback()
            if fb and hasattr(fb, 'distance_remaining') and \
               time.time() - last_log >= 2.0:
                last_log = time.time()
                nav.get_logger().info(
                    f'  [航点 {i+1}] 距目标 {fb.distance_remaining:.2f} m  '
                    f'({time.time()-t0:.0f}s)')
            time.sleep(0.2)

        elapsed = time.time() - t0
        result = nav.getResult()
        ok = (result == TaskResult.SUCCEEDED)
        status = '到达' if ok else {
            TaskResult.FAILED: '失败',
            TaskResult.CANCELED: '已取消',
        }.get(result, '未知')
        nav.get_logger().info(f'[航点 {i+1}] {status}  耗时 {elapsed:.1f}s')
        results.append((ok, elapsed))
        if ok:
            nav.get_logger().info(f'[航点 {i+1}] 停留 {dwell:.0f}s')
            time.sleep(dwell)
    return results


def go_home(nav, home, timeout):
    x, y, yaw = home
    goal = make_pose(x, y, yaw, nav.get_clock())
    nav.get_logger().info(f'[返回起点] -> ({x}, {y}, {yaw}°) 出发')
    nav.goToPose(goal)
    t0 = time.time()
    while not nav.isTaskComplete():
        if time.time() - t0 > timeout:
            nav.get_logger().warn(f'[返回起点] 超时 {timeout:.0f}s, 取消')
            nav.cancelTask()
            return False
        fb = nav.getFeedback()
        if fb and hasattr(fb, 'distance_remaining'):
            nav.get_logger().info(
                f'  [返回起点] 距目标 {fb.distance_remaining:.2f} m  '
                f'({time.time()-t0:.0f}s)')
        time.sleep(0.2)
    ok = (nav.getResult() == TaskResult.SUCCEEDED)
    nav.get_logger().info(f'[返回起点] {"已到家" if ok else "未成功"}  耗时 {time.time()-t0:.1f}s')
    return ok


def main():
    p = argparse.ArgumentParser(description='巡检航点自主巡逻 (nav2_simple_commander)')
    p.add_argument('--waypoints-file', default=WAYPOINTS_FILE, help='航点 yaml 文件')
    p.add_argument('--initial-pose', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'),
                   default=None, help='初始位姿 (地图系), 不传则用 RViz 里设的')
    p.add_argument('--dwell', type=float, default=DWELL, help='每个航点停留秒数')
    p.add_argument('--home', type=float, nargs=3, metavar=('X', 'Y', 'YAW_DEG'),
                   default=list(HOME), help='走完全部航点后返回的起点')
    p.add_argument('--no-return-home', action='store_true', help='走完航点不返回起点')
    p.add_argument('--loops', type=int, default=1, help='巡逻圈数')
    p.add_argument('--waypoint-timeout', type=float, default=WAYPOINT_TIMEOUT,
                   help='单点超时秒数')
    p.add_argument('--list-waypoints', action='store_true',
                   help='只打印航点列表, 不移动')
    args = p.parse_args()

    waypoints = load_waypoints(args.waypoints_file)
    if waypoints is None:
        waypoints = FALLBACK_WAYPOINTS
        print(f'警告: 未找到 {args.waypoints_file}, 使用内置占位航点')
    elif not waypoints:
        print(f'错误: {args.waypoints_file} 里没有航点, 先用 record_waypoint.py 记录')
        return
    home = tuple(args.home)

    if args.list_waypoints:
        for i, (x, y, yaw) in enumerate(waypoints):
            print(f'航点 {i+1}: ({x}, {y}, {yaw}°)')
        print(f'返回起点: ({home[0]}, {home[1]}, {home[2]}°)' + ('' if args.no_return_home else '  [执行]'))
        return

    rclpy.init()
    nav = BasicNavigator()

    if args.initial_pose is not None:
        nav.setInitialPose(make_pose(*args.initial_pose, nav.get_clock()))
        nav.get_logger().info(
            f'已设置初始位姿 ({args.initial_pose[0]}, {args.initial_pose[1]}, '
            f'{args.initial_pose[2]}°)')
    else:
        # 未传初始位姿: 从 tf 读当前真实位姿赋给 nav.initial_pose。
        # 原因: waitUntilNav2Active -> _waitForInitialPose 在车静止时(AMCL 不发
        # /amcl_pose)会误判"未初始化", 用默认 (0,0,0) 重置 AMCL, 把车瞬移回
        # 建图起点, 车再从起点绕一大圈开回目标点。用当前位置兜底则重置无害。
        from tf2_ros import Buffer, TransformListener
        tf_buffer = Buffer()
        tf_listener = TransformListener(tf_buffer, nav)
        deadline = time.time() + 8.0
        while time.time() < deadline:
            rclpy.spin_once(nav, timeout_sec=0.2)
            try:
                t = tf_buffer.lookup_transform(
                    'map', 'base_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                p = t.transform.translation
                q = t.transform.rotation
                yaw_deg = math.degrees(2.0 * math.atan2(q.z, q.w))
                nav.initial_pose = make_pose(p.x, p.y, yaw_deg, nav.get_clock())
                nav.get_logger().info(
                    f'从 tf 读取当前位姿 ({p.x:.2f}, {p.y:.2f}, {yaw_deg:.1f}°) 作为 initial_pose, 避免被重置')
                break
            except Exception:
                pass
    nav.get_logger().info('等待 Nav2 就绪 (amcl/planner/controller active)...')
    nav.waitUntilNav2Active()
    nav.get_logger().info('Nav2 就绪, 开始巡逻')

    all_results = []
    try:
        for loop in range(args.loops):
            nav.get_logger().info(f'===== 第 {loop+1}/{args.loops} 圈 =====')
            loop_results = run_patrol(nav, waypoints, args.waypoint_timeout, args.dwell)
            all_results.append(loop_results)
            if not args.no_return_home:
                go_home(nav, home, args.waypoint_timeout)
            if loop + 1 < args.loops:
                time.sleep(1.0)
    except KeyboardInterrupt:
        nav.get_logger().warn('手动中断')
    finally:
        nav.cancelTask()
        nav.destroyNode()
        rclpy.shutdown()

    # 汇总
    print('\n=========== 巡检汇总 ===========', flush=True)
    total_ok = 0
    total_n = 0
    for loop, loop_results in enumerate(all_results):
        for i, (ok, elapsed) in enumerate(loop_results):
            total_n += 1
            total_ok += 1 if ok else 0
            print(f'  圈{loop+1} 航点{i+1}: {"OK" if ok else "XX"}  {elapsed:6.1f}s', flush=True)
    print(f'  成功 {total_ok}/{total_n}', flush=True)


if __name__ == '__main__':
    main()
