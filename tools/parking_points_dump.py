#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 parking_zones.yaml 的点位与尺寸打成一张人读表（条目 1"点位与尺寸可配置"的字幕证据）。

为什么单独一个脚本，而不是在取证脚本里现写 python -c：capq 是 eval 一条 shell 命令，
里面再嵌带引号的 python 很容易被转义吃掉，输出一行空字幕就等于没证据（踩过）。
本脚本只读 yaml、不碰 ROS，也不写任何文件。
"""
import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DEF = os.path.join(HERE, os.pardir, 'src', 'v2x_demo', 'config', 'parking_zones.yaml')


def row(*cells):
    print('  %-14s %-10s %s' % tuple(cells))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--zones', default=DEF)
    a = ap.parse_args()
    with open(a.zones, encoding='utf-8') as fh:
        pk = yaml.safe_load(fh)['parking']
    print('配置文件：%s' % os.path.abspath(a.zones))
    print('── 场地 ──')
    lot, bar = pk['lot'], pk['barrier']
    print('  占地 x %.2f..%.2f × y %.2f..%.2f，路面 z=%.2f' %
          (lot['x0'], lot['x1'], lot['y0'], lot['y1'], pk['road_z']))
    print('── 道闸动力学与杆件尺寸（全局一份，两道闸共用）──')
    print('  抬起角 %.4f rad  落下角 %.1f  容差 %.2f  抬/落超时 %.1f/%.1f s  保持全开 %.1f s'
          % (bar['open_angle'], bar['closed_angle'], bar['angle_tolerance'],
             bar['raise_timeout_s'], bar['lower_timeout_s'], bar['hold_time_s']))
    print('  effort=%.0f N·m  velocity=%.1f  阻尼/摩擦 %.2f/%.2f  P/D %.0f/%.0f'
          % (bar['effort'], bar['velocity'], bar['damping'], bar['friction'],
             bar['p_gain'], bar['d_gain']))
    print('  杆截面 %s m  杆质量 %.1f kg  柱半径 %.2f  柱高 %.2f  机箱 %s'
          % (bar['section'], bar['mass_kg'], bar['post_radius'],
             bar['post_height'], bar['housing_size']))
    print('  地感释放后延迟 %.1f s 落杆；杆下占用时禁止落杆=%s（防砸）；'
          '堵转判据 = 力矩顶到 effort 的 %.0f%%' %
          (bar['clearance_delay_s'], bar['refuse_lower_while_occupied'],
           100.0 * bar['jam_effort_ratio']))
    print('── 闸机（道闸设备 + 卡口相机 + 地感线圈，一整套按 gate 配）──')
    for g in pk['gates']:
        b, c, lp = g['boom'], g['camera'], g['loop']
        print('  %s（%s，%s）' % (g['id'], g['label'], g['direction']))
        print('    闸杆  pivot=%s 长=%.2f m 轴=%s  hub_z=%.2f' %
              ('%s' % b['pivot'], b['length'], b['axis'], b['hub_z']))
        print('    相机  pose=%s  %dx%d@%gHz  hfov=%.2f rad' %
              ('%s' % c['pose'], c['width'], c['height'], c['update_rate'], c['hfov']))
        print('    相机  抓拍点=%s  对焦=%s  触发半径 %.1f~%.1f m  等帧 %.1f s' %
              ('%s' % c['capture_point'], '%s' % c['focus'],
               c['near_radius'], c['far_radius'], c['frame_wait']))
        print('    相机  落盘=%s  图题=%s' % (c['capture_dir'], c['image_topic']))
        print('    地感  %s center=%s size=%s' % (lp['id'], '%s' % lp['center'],
                                                  '%s' % lp['size']))
        print('    点位  核验位=%s  等待位=%s' % ('%s' % g['queue_point'],
                                                  '%s' % g['staging_point']))
    print('── 车位 ──')
    for s in pk['stalls']:
        row(s['id'], '%s' % s['center'], 'size=%s 朝向=%.4f' %
            ('%s' % s['size'], s['heading']))
    print('── 划线几何（world 里的地面/边线/通道宽）──')
    for k, v in (pk.get('stripes') or {}).items():
        # 字幕一行放不下太长（版面 990 px 主区、每行约 48 个半角位），截到 110 字符
        print(('  %-10s %s' % (k, v))[:110])
    print('── 摆渡车 car_6 ──')
    sh = pk['shuttle']
    print('  模型 %s  巡航 %.2f m/s  到位半径 %.2f m  车位停留 %.0f s  周期 %.0f s' %
          (sh['model'], sh['cruise_speed'], sh['arrive_radius'],
           sh['stall_dwell_s'], sh['loop_period_s']))
    for r in ('entry_route', 'exit_route', 'reset_route'):
        print('  %-12s %d 个点：%s' % (r, len(sh[r]),
                                       ' → '.join('(%g,%g)' % (p[0], p[1]) for p in sh[r])))
    print('── 管控与记录 ──')
    ctl = pk['control']
    print('  白名单 %s  黑名单 %s  未登记按临时车=%s  临时卡有效 %.0f s' %
          (list((ctl.get('whitelist') or {}).keys()), ctl.get('blacklist'),
           ctl['unknown_as_temp'], ctl['temp_ticket_ttl_s']))
    print(('  话题/服务：%s' % {k: v for k, v in ctl.items()
                                if k.endswith(('topic', 'srv'))})[:110])
    print('  记录目录：%s（%s / %s）' % (pk['records']['dir'], pk['records']['pass_file'],
                                        pk['records']['action_file']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
