#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
建图往返驱动: 从当前位置前进 --dist 米(闭环直行) -> 停 -> 保存 slam_toolbox 地图
-> 后退回到起点(闭环直行) -> 停

闭环直行 = 航向保持(PID) + 横向纠偏(Cross-track): 用 odom 位置算偏离本段直线的
横向误差, 把航向设定点拉回出发直线, 压住小车"越走越偏"的问题。

用法(容器内):
    python3 /home/ubuntu/my_ros2_ws/scripts/map_drive_roundtrip.py --dist 4.0 --speed 0.3
    python3 /home/ubuntu/my_ros2_ws/scripts/map_drive_roundtrip.py --dist 3.0 --force
    python3 /home/ubuntu/my_ros2_ws/scripts/map_drive_roundtrip.py --precheck-only

可选:
    --map-name  地图保存路径(不含扩展名), 默认 /home/ubuntu/my_ros2_ws/maps/my_map_<时间戳>
    --k-xt / --max-xt   横向纠偏增益/最大航向偏转 (传 --k-xt 0 可只做航向保持)
    --force    预检未通过也放行起步(运行时激光安全刹仍生效)

前置条件(均已就绪):
    - robot_driver_node 运行中(/odom + odom->base_link TF)
    - 雷达运行(/MS200/scan)
    - slam_toolbox async_slam_toolbox_node 运行中(提供服务 /slam_toolbox/save_map)

安全: 起步前激光预检(前进>=dist+margin, 后退>=margin), 行驶中半球安全刹(<0.25m 急停)。
位移用 odom 相邻帧积分(路径长度), 不是纯定时, 这样即使轻微偏航也够准。
"""
import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from slam_toolbox.srv import SaveMap
from std_msgs.msg import String


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class MapDriveRoundtrip(Node):
    def __init__(self, dist, speed, map_name, kp, ki, kd, max_wz, k_xt, max_xt,
                 margin=0.4, force=False):
        super().__init__('map_drive_roundtrip')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/MS200/scan', self.scan_cb, 10)
        self.speed = speed
        self.dist = dist
        self.map_name = map_name
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_wz = max_wz
        self.k_xt = k_xt
        self.max_xt = max_xt
        self.margin = margin
        self.force = force

        self.x = self.y = self.yaw = None
        self.scan = None

        # save_map 服务(来自 slam_toolbox)
        self.save_cli = self.create_client(SaveMap, '/slam_toolbox/save_map')
        while not self.save_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('等待 /slam_toolbox/save_map 服务...')
        self.get_logger().info('save_map 服务就绪')

    # ---------- 基础 ----------
    def odom_cb(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def scan_cb(self, m):
        self.scan = m

    def wait_odom(self, timeout=5.0):
        t0 = time.time()
        while self.x is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.x is None:
            self.get_logger().error('收不到 /odom, 先启动底盘驱动: '
                                    'ros2 launch robot_driver ekf_localization.launch.py')
            sys.exit(1)

    def publish_zero(self, secs=0.5):
        """发布全零 = 停车"""
        end = time.time() + secs
        while time.time() < end:
            self.pub.publish(Twist())
            rclpy.spin_once(self, timeout_sec=0.05)

    # ---------- 激光预检 (只读, 不动小车) ----------
    def wait_scan(self, timeout=3.0):
        t0 = time.time()
        while self.scan is None and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.scan is not None

    def cone_min(self, ang_min, ang_max):
        """返回 [ang_min,ang_max] 范围内最近的有效障碍距离; 无有效点返回 None"""
        s = self.scan
        best = None
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if not (ang_min <= a <= ang_max):
                continue
            if 0.0 < r < s.range_max:
                best = r if best is None else min(best, r)
        return best

    def scan_summary(self):
        """前/后/左/右四个锥形最近障碍。前/后只用 ±8° 直行窄锥, 避免走廊侧墙斜影误判。"""
        deg = math.radians
        fwd = self.cone_min(-deg(8), deg(8))
        bwd = self.cone_min(math.pi - deg(8), math.pi + deg(8))
        lft = self.cone_min(deg(65), deg(115))
        rgt = self.cone_min(-deg(115), -deg(65))
        return {'forward': fwd, 'backward': bwd, 'left': lft, 'right': rgt}

    def semicircle_min(self, center_angle):
        """[center±90°] 半球内最近有效障碍; 无有效点返回 None"""
        s = self.scan
        if s is None:
            return None
        best = None
        for i, r in enumerate(s.ranges):
            a = s.angle_min + i * s.angle_increment
            if abs(wrap_pi(a - center_angle)) > math.pi / 2:
                continue
            if 0.0 < r < s.range_max:
                best = r if best is None else min(best, r)
        return best

    def precheck(self):
        """起步前检查: 前进>=dist+margin, 后退>=margin(后退段只回到起点, 不退过起点)。
        返回 (ok, 说明str)"""
        if not self.wait_scan():
            return False, '收不到 /MS200/scan, 无法预检, 拒绝起步'
        sm = self.scan_summary()
        issues = []
        if sm['forward'] is not None and sm['forward'] < self.dist + self.margin:
            issues.append(f'前进方向最近障碍 {sm["forward"]:.2f} m < 需要 {self.dist + self.margin:.2f} m')
        if sm['backward'] is not None and sm['backward'] < self.margin:
            issues.append(f'后退方向最近障碍 {sm["backward"]:.2f} m < 需要 {self.margin:.2f} m')
        side_vals = [d for d in (sm['left'], sm['right']) if d is not None]
        side = min(side_vals) if side_vals else None
        if side is not None and side < 0.25:
            issues.append(f'侧面空间过窄 {side:.2f} m')
        ok = not issues

        def fmt(d):
            return '∞' if d is None else f'{d:.2f}'

        msg = ('通过' if ok else '未通过: ' + '; '.join(issues)) + \
              f'  (前 {fmt(sm["forward"])} / 后 {fmt(sm["backward"])} / ' \
              f'左 {fmt(sm["left"])} / 右 {fmt(sm["right"])} m)'
        return ok, msg

    # ---------- 闭环直行控制 ----------
    @staticmethod
    def lateral(x, y, x0, y0, yaw0):
        """相对 (x0,y0,yaw0) 直线的横向偏移 (m), 正=向左偏"""
        return -math.sin(yaw0) * (x - x0) + math.cos(yaw0) * (y - y0)

    def ctrl_wz(self, direction, x0, y0, yaw0, int_err, prev_yaw, last_yaw_rate):
        """横向纠偏 + 航向保持 PID。
        返回 (cmd_wz, err, int_err, prev_yaw, last_yaw_rate)"""
        yaw = self.yaw
        e_xt = self.lateral(self.x, self.y, x0, y0, yaw0)
        xt_ang = max(-self.max_xt, min(self.max_xt, self.k_xt * e_xt))
        yaw_setpoint = yaw0 - direction * xt_ang
        err = wrap_pi(yaw_setpoint - yaw)
        dt = 0.02
        yaw_rate = wrap_pi(yaw - prev_yaw) / dt
        prev_yaw = yaw
        last_yaw_rate = 0.8 * last_yaw_rate + 0.2 * yaw_rate   # 平滑
        int_err += err * dt
        int_err = max(-0.3, min(0.3, int_err))                 # 抗饱和限幅
        cmd_wz = self.kp * err + self.ki * int_err - self.kd * last_yaw_rate
        cmd_wz = max(-self.max_wz, min(self.max_wz, cmd_wz))
        return cmd_wz, err, int_err, prev_yaw, last_yaw_rate

    def drive_to_distance(self, target, direction, ref, label):
        """以 direction*speed 闭环直行 target 米, 按 odom 路径长度积分直到 >= target"""
        x0, y0, yaw0 = ref
        px, py = self.x, self.y
        traveled = 0.0
        last_log = 0.0
        int_err = 0.0
        prev_yaw = self.yaw
        last_yaw_rate = 0.0
        while traveled < target:
            # 运行时安全刹: 行驶方向半球内最近障碍 < 0.25m 立即急停
            danger = self.semicircle_min(0.0 if direction > 0 else math.pi)
            if danger is not None and danger < 0.25:
                self.get_logger().error(f'[SAFETY-STOP] {label}: 前方 {danger:.2f} m 太近, 已急停')
                self.publish_zero(0.5)
                break
            cmd_wz, err, int_err, prev_yaw, last_yaw_rate = \
                self.ctrl_wz(direction, x0, y0, yaw0, int_err, prev_yaw, last_yaw_rate)
            tw = Twist()
            tw.linear.x = direction * self.speed
            tw.angular.z = cmd_wz
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.x is None:
                continue
            traveled += math.hypot(self.x - px, self.y - py)
            px, py = self.x, self.y
            if time.time() - last_log >= 0.5:
                last_log = time.time()
                lat = self.lateral(self.x, self.y, x0, y0, yaw0)
                self.get_logger().info(
                    f'[{label}] 已行驶 {traveled:.3f}/{target:.2f} m  '
                    f'y_lat={lat:+.3f}  yaw={math.degrees(self.yaw):+.1f}°  wz={cmd_wz:+.3f}')
        self.publish_zero(0.4)
        self.get_logger().info(f'[{label}-done] 实际行驶 {traveled:.3f} m')
        return traveled

    def drive_back_to_start(self, ref, tol=0.06):
        """闭环后退: 沿起点直线(横向纠偏) 退回到净位移 <= tol, 带行驶上限保护"""
        x0, y0, yaw0 = ref
        px, py = self.x, self.y
        traveled = 0.0
        max_back = self.dist + 0.4
        last_log = 0.0
        disp = 999.0
        int_err = 0.0
        prev_yaw = self.yaw
        last_yaw_rate = 0.0
        direction = -1.0
        while True:
            danger = self.semicircle_min(math.pi)   # 后退方向半球
            if danger is not None and danger < 0.25:
                self.get_logger().error(f'[SAFETY-STOP] back: 后方 {danger:.2f} m 太近, 已急停')
                self.publish_zero(0.5)
                break
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.x is not None:
                traveled += math.hypot(self.x - px, self.y - py)
                px, py = self.x, self.y
            dxs, dys = self.x - x0, self.y - y0
            disp = math.hypot(dxs, dys)
            if disp <= tol:
                break
            if traveled >= max_back:
                self.get_logger().warn(f'后退保护上限 {max_back:.2f} m 触发, 停车')
                break
            cmd_wz, err, int_err, prev_yaw, last_yaw_rate = \
                self.ctrl_wz(direction, x0, y0, yaw0, int_err, prev_yaw, last_yaw_rate)
            tw = Twist()
            tw.linear.x = direction * self.speed
            tw.angular.z = cmd_wz
            self.pub.publish(tw)
            if time.time() - last_log >= 0.5:
                last_log = time.time()
                lat = self.lateral(self.x, self.y, x0, y0, yaw0)
                self.get_logger().info(
                    f'[back] 距起点 {disp:.3f} m  y_lat={lat:+.3f}  '
                    f'yaw={math.degrees(self.yaw):+.1f}°  wz={cmd_wz:+.3f}')
        self.publish_zero(0.4)
        self.get_logger().info(f'[back-done] 实际后退 {traveled:.3f} m, 净位移 {disp:.3f} m')
        return traveled

    # ---------- 保存地图 ----------
    def save_map(self):
        req = SaveMap.Request()
        req.name = String()
        req.name.data = self.map_name
        self.get_logger().info(f'调用 save_map 保存到 {self.map_name} ...')
        future = self.save_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('save_map 调用失败/超时')
            return False
        resp = future.result()
        if resp.result == SaveMap.Response.RESULT_SUCCESS:
            self.get_logger().info('地图保存成功')
            return True
        self.get_logger().error(f'save_map 失败 result={resp.result} '
                                '(1=NO_MAP_RECEIVED, 255=UNDEFINED_FAILURE)')
        return False

    # ---------- 主流程 ----------
    def run(self):
        self.get_logger().info(
            f'=== 建图往返(闭环直行): 前进 {self.dist:.1f}m -> 停 -> 保存地图 -> 后退回起点 ===')
        self.get_logger().info(f'PID: Kp={self.kp} Ki={self.ki} Kd={self.kd} max_wz={self.max_wz} '
                               f'| 横向纠偏 k_xt={self.k_xt} /m, 限幅 ±{math.degrees(self.max_xt):.1f}°')

        # 起步前激光预检 (只读)
        ok, msg = self.precheck()
        self.get_logger().info(f'[precheck] 激光预检{msg}')
        if not ok:
            if self.force:
                self.get_logger().warn('--force: 预检未通过但放行, 靠运行时激光安全刹保护')
            else:
                self.get_logger().error('预检未通过, 拒绝起步。请先挪开小车/障碍或加 --force')
                self.publish_zero(0.3)
                return
        self.get_logger().info('3 秒后开始! Ctrl+C 中止')
        time.sleep(3)

        self.wait_odom()
        x0, y0, yaw0 = self.x, self.y, self.yaw
        ref = (x0, y0, yaw0)
        self.get_logger().info(f'[start] 起点 odom = ({x0:+.3f}, {y0:+.3f}) yaw={math.degrees(yaw0):+.1f}°')

        self.drive_to_distance(self.dist, +1.0, ref, 'forward')
        f_lat = self.lateral(self.x, self.y, x0, y0, yaw0)

        # 停顿让 slam 消化最后一段, 再保存
        self.get_logger().info('[pause] 停顿 3s, 等地图更新...')
        self.publish_zero(3.0)

        ok = self.save_map()

        self.drive_back_to_start(ref)

        xf, yf = self.x, self.y
        print('\n=========== 结果汇总 ===========', flush=True)
        print(f'  起点   : ({x0:+.3f}, {y0:+.3f})', flush=True)
        print(f'  终点   : ({xf:+.3f}, {yf:+.3f})', flush=True)
        print(f'  净位移 : {math.hypot(xf - x0, yf - y0):.3f} m (期望 ~0, 即回到原点)', flush=True)
        print(f'  前进段横向偏移: {f_lat:+.3f} m (闭环应 ≈0)', flush=True)
        print(f'  保存   : {"成功" if ok else "失败 -> 见日志, 可手动重试" }', flush=True)
        print(f'  文件   : {self.map_name}.pgm / {self.map_name}.yaml', flush=True)
        self.publish_zero(0.5)


def main():
    p = argparse.ArgumentParser(description='建图往返(闭环直行): 前进 dist 米 -> 停 -> 保存地图 -> 后退回起点')
    p.add_argument('--dist', type=float, default=4.0, help='单向往返距离 m')
    p.add_argument('--speed', type=float, default=0.3, help='线速度 m/s')
    p.add_argument('--map-name', default=None,
                   help='地图保存路径(不含扩展名), 默认 /home/ubuntu/my_ros2_ws/maps/my_map_<时间戳>')
    p.add_argument('--kp', type=float, default=1.8, help='航向 PID 比例增益')
    p.add_argument('--ki', type=float, default=0.15, help='航向 PID 积分增益')
    p.add_argument('--kd', type=float, default=0.25, help='航向 PID 微分增益')
    p.add_argument('--max-wz', type=float, default=0.5, help='最大角速度 rad/s')
    p.add_argument('--k-xt', type=float, default=1.5, help='横向纠偏增益 1/m (传0=只做航向保持)')
    p.add_argument('--max-xt', type=float, default=0.15, help='横向纠偏最大航向偏转 rad')
    p.add_argument('--margin', type=float, default=0.4, help='预检安全余量 m')
    p.add_argument('--force', action='store_true',
                   help='预检未通过也放行起步(运行时激光安全刹仍生效)')
    p.add_argument('--precheck-only', action='store_true',
                   help='只读跑激光预检并打印周围空间, 不动小车')
    args = p.parse_args()

    if args.map_name is None:
        args.map_name = f'/home/ubuntu/my_ros2_ws/maps/my_map_{time.strftime("%Y%m%d_%H%M%S")}'

    rclpy.init()
    node = MapDriveRoundtrip(args.dist, args.speed, args.map_name,
                             args.kp, args.ki, args.kd, args.max_wz,
                             args.k_xt, args.max_xt, args.margin, args.force)
    try:
        if args.precheck_only:
            ok, msg = node.precheck()
            print(f'[precheck] 激光预检{msg}', flush=True)
            print('仅预检, 未动小车。通过与否: ', '通过' if ok else '未通过', flush=True)
        else:
            node.run()
    except KeyboardInterrupt:
        print('\n中断, 已尝试停车', flush=True)
    finally:
        node.publish_zero(0.3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
