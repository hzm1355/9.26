#!/usr/bin/env python3
# 订阅一次 /map（volatile QoS，绕开 nav2 map_saver 的 transient_local 不兼容问题），
# 统计地图质量指标 + 输出 ASCII 预览 + 保存 PGM/PNG 快照。
import sys
import time
from collections import Counter

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

received = {}


def main():
    rclpy.init()
    node = rclpy.create_node('map_analyzer')

    def cb(msg):
        received['map'] = msg

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.VOLATILE)
    node.create_subscription(OccupancyGrid, '/map', cb, qos)

    deadline = time.time() + 15
    while 'map' not in received and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=1.0)
    if 'map' not in received:
        print('MAP_TIMEOUT: 15s 内未收到 /map')
        return 1

    m = received['map']
    w, h = m.info.width, m.info.height
    res = m.info.resolution
    ox, oy = m.info.origin.position.x, m.info.origin.position.y
    data = list(m.data)

    c = Counter()
    for v in data:
        if v < 0:
            c['unknown'] += 1
        elif v == 0:
            c['free'] += 1
        elif v >= 65:
            c['occupied'] += 1
        else:
            c['gray'] += 1
    total = w * h
    print(f'MAP {w}x{h} res={res:.3f} origin=({ox:.2f},{oy:.2f})')
    print(f'world-extent: x[{ox:.2f},{ox+w*res:.2f}] y[{oy:.2f},{oy+h*res:.2f}]')
    print(f'unknown {c["unknown"]} ({100*c["unknown"]/total:.1f}%)  '
          f'free {c["free"]} ({100*c["free"]/total:.1f}%)  '
          f'occupied {c["occupied"]} ({100*c["occupied"]/total:.1f}%)  '
          f'gray {c["gray"]} ({100*c["gray"]/total:.1f}%)')
    print(f'explored-ratio {100*(total-c["unknown"])/total:.1f}%')

    # ASCII 预览（第 1 行 = 地图最北 y 最大；data 首行是 y 最小，需翻转）
    COLS, ROWS = 78, 42
    lines = []
    for r in range(ROWS - 1, -1, -1):
        line = ''
        for cc in range(COLS):
            y0 = r * h // ROWS
            y1 = max(y0 + 1, (r + 1) * h // ROWS)
            x0 = cc * w // COLS
            x1 = max(x0 + 1, (cc + 1) * w // COLS)
            occ = unk = fre = 0
            for yy in range(y0, y1):
                base = yy * w
                for xx in range(x0, x1):
                    v = data[base + xx]
                    if v < 0:
                        unk += 1
                    elif v >= 65:
                        occ += 1
                    elif v == 0:
                        fre += 1
            if occ:
                ch = '#'
            elif fre and unk == 0:
                ch = '.'
            elif fre:
                ch = ','
            else:
                ch = ' '
            line += ch
        lines.append(line)
    print('ASCII preview (每格约 0.07m 级，# 障碍 . 纯free , free混杂unknown 空白 unknown)：')
    print('\n'.join(lines))

    # PGM 快照（翻转行序：data 首行是 y 最小，PGM 首行应为图顶）
    with open('/tmp/v2x_map.pgm', 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        row = bytearray(w)
        for yy in range(h):
            for xx in range(w):
                v = data[(h - 1 - yy) * w + xx]
                if v < 0:
                    row[xx] = 205
                elif v == 0:
                    row[xx] = 254
                elif v >= 65:
                    row[xx] = 0
                else:
                    row[xx] = 127
            f.write(row)
    print('PGM_SAVED /tmp/v2x_map.pgm')
    try:
        from PIL import Image
        Image.open('/tmp/v2x_map.pgm').save(
            '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/map_snapshot.png')
        print('PNG_SAVED map_snapshot.png')
    except Exception as e:
        print('PNG_SKIP', e)
    return 0


if __name__ == '__main__':
    sys.exit(main())
