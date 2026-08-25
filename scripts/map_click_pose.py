#!/usr/bin/env python3
"""
map_click_pose.py — 在地图图片上点两下,输出 AMCL 初始位姿命令

用途:胶带标记丢了之后,不用再找建图原点。把车 teleop 到房间里任意
有辨识度的位置,打开本工具在地图上点一下车所在的位置、再点一下车头
朝向,即可得到喂给 record_waypoint.py --initial-pose 的坐标(地图坐标系,
与 Nav2 的 map 系一致)。

不需要 ROS,宿主机或容器里都能跑,但要有图形界面(桌面 / VNC / SSH -X):
    python3 /home/ubuntu/my_ros2_ws/scripts/map_click_pose.py
    python3 /home/ubuntu/my_ros2_ws/scripts/map_click_pose.py --map <其他地图.yaml>

无图形界面的机器(SSH 纯终端)可用 --info 打印地图参数和原点像素位置:
    python3 /home/ubuntu/my_ros2_ws/scripts/map_click_pose.py --info

交互:
    左键第 1 下: 选锚点(车实际停的位置)
    左键第 2 下: 选朝向(车头指向,从锚点指向第二个点)
    右键:        清除全部标记重选
    q / 关窗口:  退出
"""
import argparse
import math
import os
import sys

import numpy as np


def load_yaml(path):
    """读取地图 yaml,优先 PyYAML,没有则手写解析(只取用到的字段)。"""
    try:
        import yaml
        with open(path) as f:
            return yaml.safe_load(f)
    except ImportError:
        d = {}
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                key, _, val = line.partition(':')
                key = key.strip()
                val = val.strip().strip('"\'').strip()
                if key == 'image':
                    d['image'] = val
                elif key == 'resolution':
                    d['resolution'] = float(val)
                elif key == 'origin':
                    d['origin'] = [float(x.strip()) for x in val.strip('[]').split(',')]
        return d


def load_pgm(path):
    """读取 PGM 灰度图(P5 二进制 / P2 文本),返回 (H, W) 的 uint8 数组。"""
    with open(path, 'rb') as f:
        raw = f.read()
    pos = 0

    def next_token():
        nonlocal pos
        while pos < len(raw) and raw[pos] in b' \t\r\n':
            pos += 1
        if pos < len(raw) and raw[pos] == ord('#'):
            while pos < len(raw) and raw[pos] != ord('\n'):
                pos += 1
            return next_token()
        start = pos
        while pos < len(raw) and raw[pos] not in b' \t\r\n':
            pos += 1
        return raw[start:pos]

    magic = next_token()
    w = int(next_token())
    h = int(next_token())
    maxval = int(next_token())
    if magic == b'P5':
        pos += 1  # 跳过 maxval 后的单个空白字符
        if pos < len(raw) and raw[pos] in b'\r\n':
            pos += 1  # 兼容 CRLF
        data = np.frombuffer(raw[pos:pos + w * h], dtype=np.uint8)
    elif magic == b'P2':
        vals = []
        while len(vals) < w * h:
            tok = next_token()
            if not tok:
                break
            vals.append(int(tok))
        data = np.asarray(vals, dtype=np.uint8)
    else:
        raise ValueError(f'不支持的 PGM 格式: {magic.decode()!r}')
    if data.size < w * h:
        raise ValueError(f'PGM 数据不完整: 需要 {w*h} 字节,实际 {data.size}')
    return data.reshape(h, w)


def setup_cjk_font():
    """优先使用系统中文字体,避免 matplotlib 标题/标注显示为方块。"""
    try:
        import matplotlib.font_manager as fm
        cjk = [f.name for f in fm.fontManager.ttflist
               if any(k in f.name for k in ('CJK', 'Noto Sans SC', 'WenQuanYi',
                                            'SimHei', 'Droid Sans Fallback'))]
        if cjk:
            import matplotlib.pyplot as plt
            plt.rcParams['font.sans-serif'] = [cjk[0], 'DejaVu Sans']
            plt.rcParams['axes.unicode_minus'] = False
    except Exception:
        pass


def print_ascii(pgm_path, arr, res, ox, oy, oyaw):
    """无图形界面模式:把地图画成字符画(每字符 4 像素 = 0.2m)。"""
    H, W = arr.shape
    S = 4  # 每字符覆盖 4x4 像素 = 0.2m
    nc, nr = (W + S - 1) // S, (H + S - 1) // S
    free, occ = arr >= 240, arr < 128

    def col_x(c):
        return ox + (c * S + S / 2.0) * res

    def row_y(r):
        return oy + (H - r * S - S / 2.0) * res

    # 地图原点 (0,0) 所在的字符格
    j0 = (0.0 - ox) / res - 0.5
    i0 = H - (0.0 - oy) / res - 0.5
    c0, r0 = int(j0 // S), int(i0 // S)

    grid = []
    for r in range(nr):
        line = []
        for c in range(nc):
            if c == c0 and r == r0:
                line.append('O')
                continue
            if c % 5 == 0 and r % 5 == 0:
                line.append('+')
                continue
            blk = arr[r * S:(r + 1) * S, c * S:(c + 1) * S]
            fo = (blk >= 240).mean()
            oc = (blk < 128).mean()
            line.append('#' if oc > 0 else (' ' if fo > 0.6 else '.'))
        grid.append(''.join(line))

    top = [' '] * nc
    for c in range(0, nc, 5):
        lab = f'{col_x(c):+.1f}'
        for k, ch in enumerate(lab):
            cc = c - 1 + k
            if 0 <= cc < nc:
                top[cc] = ch
    print('     ' + ''.join(top))
    for r in range(nr):
        if r % 5 == 0:
            print(f'{row_y(r):+5.1f} ' + grid[r])
        else:
            print('      ' + grid[r])
    print()
    print('每字符 = 0.2m, 每 5 字符 = 1m;  O = 地图原点 (0,0);  右=东(+x), 上=北(+y)')
    print('# = 墙/障碍, 空格 = 可通行, . = 未探索')
    print('看好车在哪一格后,告诉我 列 行,或用:')
    print('    python3 scripts/map_click_pose.py --click 列 行 [朝向度]')


def print_info(pgm_path, arr, res, ox, oy, oyaw):
    """无界面模式:打印地图参数,以及原点(0,0)在图片上的像素位置。"""
    H, W = arr.shape
    # 世界坐标 -> 像素: 列 j(从左), 行 i(从上), 像素中心 = j+0.5 / i+0.5
    j = (0.0 - ox) / res - 0.5
    i = H - (0.0 - oy) / res - 0.5
    print(f'地图: {pgm_path} ({W}x{H}, 分辨率 {res} m/px, 原点 [{ox}, {oy}, {oyaw}])')
    print(f'世界范围: x [{ox:.2f}, {ox + W * res:.2f}] m,  y [{oy:.2f}, {oy + H * res:.2f}] m')
    print()
    print(f'地图原点 (0,0) 在图片中的像素位置 (左上角为 0,0):')
    print(f'    my_map.pgm:  列 col ≈ {j:.1f}   行 row ≈ {i:.1f}')
    for name in ('my_map.jpg', 'my_map.png'):
        jp = os.path.join(os.path.dirname(pgm_path), name)
        if os.path.exists(jp):
            k = 4  # 常见的 4 倍放大预览图
            print(f'    {name}: 列 col ≈ {j * k:.1f}   行 row ≈ {i * k:.1f}  (按 4 倍放大换算)')
    print('在图片查看器里定位到该像素,看看周围是什么家具/墙角,')
    print('再到房间里找到对应位置,把车开过去、车头朝地图 +x 方向,')
    print('然后照旧 --initial-pose 0 0 0。')


def main():
    p = argparse.ArgumentParser(description='在地图上点两下,输出 --initial-pose 命令')
    p.add_argument('--map', default='/home/ubuntu/my_ros2_ws/maps/my_map.yaml',
                   help='地图 yaml 路径(默认 my_map.yaml)')
    p.add_argument('--info', action='store_true', help='无界面:打印地图参数和原点像素位置')
    p.add_argument('--ascii', action='store_true',
                   help='无界面:打印字符画地图(--ascii 输出后可用 --click 换算)')
    p.add_argument('--click', type=float, nargs='+', metavar='COL ROW [YAW_DEG]',
                   help='把 --ascii 的字符坐标(列 行)换算成 --initial-pose 命令;'
                        '可选第三个参数给朝向(度)')
    args = p.parse_args()

    map_yaml = args.map
    if not os.path.exists(map_yaml):
        sys.exit(f'找不到地图文件: {map_yaml}')
    d = load_yaml(map_yaml)
    pgm_path = os.path.join(os.path.dirname(map_yaml), d.get('image', 'my_map.pgm'))
    arr = load_pgm(pgm_path)
    res = float(d['resolution'])
    ox, oy, oyaw = (float(v) for v in d['origin'])
    H, W = arr.shape

    if args.click:
        if len(args.click) < 2 or len(args.click) > 3:
            sys.exit('--click 需要 列 行 [朝向度]')
        col, row = args.click[0], args.click[1]
        S = 4
        x = ox + (col * S + S / 2.0) * res
        y = oy + (H - row * S - S / 2.0) * res
        if len(args.click) == 3:
            yaw = args.click[2]
            print()
            print('=' * 62)
            print(f'字符坐标 (列={col:g}, 行={row:g}) -> 地图坐标: x={x:.3f}  y={y:.3f}  朝向: {yaw:.1f}°')
            print('>>> 在容器内执行:')
            print(f'    python3 /home/ubuntu/my_ros2_ws/scripts/record_waypoint.py '
                  f'--initial-pose {x:.3f} {y:.3f} {yaw:.1f}')
            print('=' * 62)
        else:
            print(f'字符坐标 (列={col:g}, 行={row:g}) -> 地图坐标: x={x:.3f}  y={y:.3f}')
            print(f'再给朝向(度): python3 scripts/map_click_pose.py --click {col:g} {row:g} <yaw度>')
        return

    if args.info:
        print_info(pgm_path, arr, res, ox, oy, oyaw)
        return

    if args.ascii:
        print_ascii(pgm_path, arr, res, ox, oy, oyaw)
        return

    try:
        import matplotlib
        matplotlib.use('TkAgg')
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit('宿主机没有 matplotlib,请先安装: pip install matplotlib numpy\n'
                 '(或者用 --info 模式在纯终端下获取原点像素位置)')
    setup_cjk_font()

    H, W = arr.shape
    x0, x1 = ox, ox + W * res
    y0, y1 = oy, oy + H * res

    fig, ax = plt.subplots(figsize=(13, 13 * H / W))
    # PGM 第 0 行在最北(最上),翻转后 imshow 上北下南,extent 给世界坐标
    ax.imshow(np.flipud(arr), cmap='gray', origin='lower', extent=[x0, x1, y0, y1])
    ax.set_aspect('equal')
    ax.set_xlabel('x (m, 向东)')
    ax.set_ylabel('y (m, 向北)')
    ax.set_title('左键: 第1下选锚点, 第2下选朝向 | 右键清除 | q 退出', fontsize=10)
    ax.grid(True, ls=':', lw=0.5, alpha=0.6, color='tab:blue')
    ax.set_xticks(np.arange(np.ceil(x0), x1, 1.0))
    ax.set_yticks(np.arange(np.ceil(y0), y1, 1.0))

    # 地图原点标记:红点 + 红色箭头指向 map 系 +x 方向
    dx, dy = math.cos(oyaw), math.sin(oyaw)
    ax.plot(0, 0, 'r.', ms=12)
    ax.annotate('', xy=(dx, dy), xytext=(0, 0),
                arrowprops=dict(arrowstyle='-|>', color='red', lw=2))
    ax.text(0.08, 0.08, '地图原点 (0,0)', color='red', fontsize=9,
            bbox=dict(fc='white', alpha=0.7, pad=1))

    sel = {'x': None, 'y': None}
    marks = []

    def clear_marks():
        for m in marks:
            try:
                m.remove()
            except Exception:
                pass
        marks.clear()

    def on_click(event):
        if event.inaxes != ax:
            return
        if event.button == 1:
            if sel['x'] is None:
                sel['x'], sel['y'] = event.xdata, event.ydata
                marks.append(ax.plot(event.xdata, event.ydata, 'go', ms=10)[0])
            else:
                x2, y2 = event.xdata, event.ydata
                yaw = math.degrees(math.atan2(y2 - sel['y'], x2 - sel['x']))
                yaw = (yaw + 180.0) % 360.0 - 180.0  # 归一化到 (-180, 180]
                marks.append(ax.annotate(
                    '', xy=(x2, y2), xytext=(sel['x'], sel['y']),
                    arrowprops=dict(arrowstyle='-|>', color='cyan', lw=2)))
                print()
                print('=' * 62)
                print(f'锚点: x={sel["x"]:.3f}  y={sel["y"]:.3f}   朝向: {yaw:.1f}°')
                print('>>> 在容器内执行:')
                print(f'    python3 /home/ubuntu/my_ros2_ws/scripts/record_waypoint.py '
                      f'--initial-pose {sel["x"]:.3f} {sel["y"]:.3f} {yaw:.1f}')
                print('(车先 teleop 开到锚点,朝向摆好后,再执行上面的命令)')
                print('=' * 62)
                sel['x'] = sel['y'] = None  # 自动复位,可继续选下一个锚点
        elif event.button == 3:
            sel['x'] = sel['y'] = None
            clear_marks()
        fig.canvas.draw_idle()

    def on_move(event):
        if event.inaxes != ax:
            return
        if sel['x'] is None:
            fig.canvas.manager.set_window_title(
                f'({event.xdata:.2f}, {event.ydata:.2f}) m')
        else:
            yaw = math.degrees(math.atan2(event.ydata - sel['y'], event.xdata - sel['x']))
            yaw = (yaw + 180.0) % 360.0 - 180.0
            fig.canvas.manager.set_window_title(
                f'锚点 ({sel["x"]:.2f}, {sel["y"]:.2f}) → 朝向 {yaw:.0f}°')

    def on_key(event):
        if event.key == 'q':
            plt.close(fig)

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('motion_notify_event', on_move)
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()


if __name__ == '__main__':
    main()
