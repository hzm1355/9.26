#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""停车场"通行记录 ↔ 抓拍 ↔ 识别结果"三方对账（任务五条目 2 的字幕证据）。

条目 2 要求"虚拟摄像头输出图像话题，可对接车牌识别算法节点，记录通行时间"。
这三件事分属三份记录，单独看任何一份都能自说自话：
  · /tmp/v2x_parking/parking_pass.jsonl   停车场记录器：车牌、置信度、capture_id、原图路径、四段时间
  · /tmp/v2x_records/plate_records.jsonl  卡口记录器：算法名、真实牌、识别牌、耗时、光照
  · /tmp/v2x_parking_in|out/<capture_id>.png  相机插件按次落盘的抓拍原图
只有三者能按同一个 capture_id 串起来，才说明"这个号是这台相机拍的那张图、
被那个算法认出来的"，而不是节点自己编的。任何一环缺失都当场点名，不糊成"看起来对"。

用法：
  pk_show_capture.py                 # 最近 2 条通行记录
  pk_show_capture.py --n 4 --gate gate_out
  pk_show_capture.py --capture-id C0013-gate_in
"""
import argparse
import json
import os
import sys

PJ = '/tmp/v2x_parking/parking_pass.jsonl'
PR = '/tmp/v2x_records/plate_records.jsonl'


def read_jsonl(path):
    """整读；文件没来 = 记录器没起或还没落第一条，返回 None 让调用方点名。"""
    if not os.path.exists(path):
        return None
    out = []
    with open(path, encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    return out


def frame_verdict(path):
    """判这张抓拍到底有没有内容 —— 别用文件字节数。
    停车场两路 1080p 的实拍帧实测 42~48 KB（背景是纯色地面、牌面只占几十个像素，压得动），
    而判据当初是从高速卡口那两路（60~90 KB）搬来的，于是把有内容的帧念成"疑似空帧"。
    像素标准差才与内容对应：整幅同色的空帧 std≈0，实测有车牌的帧 std 49~79、灰度级 210~254。
    读不到图（插件正在写这个文件）也照实说，不冒充"空帧"。"""
    try:
        from PIL import Image
        im = Image.open(path).convert('L')
        px = list(im.resize((192, 108)).getdata())
        mean = sum(px) / len(px)
        std = (sum((p - mean) ** 2 for p in px) / len(px)) ** 0.5
        return '存在（内容标准差 %.1f、灰度级 %d）' % (std, len(set(px))) \
            if std > 3.0 else '!! 空帧（标准差 %.1f）' % std
    except Exception as e:
        return '!! 读不出像素（%s），只报字节数不下内容结论' % type(e).__name__


def show(rec, pr_map):
    cid = rec.get('capture_id') or ''
    img = rec.get('image_path') or ''
    # 方向/结果直接用记录器落盘时算好的 *_name 字段：ParkingPass 的 DIR_IN 是 0，
    # 而 ParkingQuery 请求里的 DIR_IN 是 1（0 留给"不限"），这里自己再映射一次就会串号。
    print('◆ %s  %s %s  车=%s  牌=%s  %s/%s  识别=%.2f' % (
        rec.get('pass_id', '?'), rec.get('direction_name', '?'),
        rec.get('gate_id', '?'), rec.get('vehicle_id', ''),
        rec.get('plate_number') or '（空）', rec.get('kind_name', '?'),
        rec.get('result_name', '?'), float(rec.get('plate_confidence', 0.0))))
    print('  通行时间 到场=%.2f 抬到位=%.2f 离场=%.2f 落到位=%.2f'
          '｜抬%.2f 行%.2f 落%.2f 全程%.2f s' % (
              float(rec.get('arrive_s', 0.0)), float(rec.get('raise_s', 0.0)),
              float(rec.get('clear_s', 0.0)), float(rec.get('lower_s', 0.0)),
              float(rec.get('raise_dur_s', 0.0)), float(rec.get('pass_dur_s', 0.0)),
              float(rec.get('lower_dur_s', 0.0)), float(rec.get('cycle_s', 0.0))))
    print('  capture_id=%s' % (cid or '（空：这条没有抓拍，是手动/故障路径）'))
    if not cid:
        return
    nb = rec.get('plate_number') or ''
    p = pr_map.get(cid)
    if p is None:
        print('  !! 卡口记录里没有这个 capture_id —— plate_recorder 当时没在线，'
              '算法名/真实牌无法核对（不是识别失败）')
    else:
        same = (p.get('plate_number') or '') == nb
        print('  识别行  算法=%s 真实=%s 识别=%s %s 耗时=%.1f ms 光照=%s' % (
            p.get('algorithm', '?'), p.get('truth_number') or '（无真值）',
            p.get('plate_number') or '（空）',
            '一致' if same else '!! 与通行记录不同号',
            float(p.get('process_ms', 0.0)), p.get('lighting', '?')))
        if p.get('image_path') and p.get('image_path') != img:
            print('  !! 两份记录的图片路径不一致：%s vs %s' % (img, p.get('image_path')))
    if img:
        if os.path.exists(img):
            kb = os.path.getsize(img) / 1024.0
            print('  抓拍原图 %s（%.0f KB）%s' % (img, kb, frame_verdict(img)))
        else:
            print('  !! 抓拍原图不在盘上：%s' % img)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pass-jsonl', default=PJ)
    ap.add_argument('--plate-jsonl', default=PR)
    ap.add_argument('--n', type=int, default=2, help='看最近几条通行记录')
    ap.add_argument('--gate', default='', help='只看某道闸（gate_in / gate_out）')
    ap.add_argument('--capture-id', default='', help='只看这一个抓拍号')
    a = ap.parse_args()

    passes = read_jsonl(a.pass_jsonl)
    if passes is None:
        print('!! 没有 %s：停车场记录器没起来或还没落过盘' % a.pass_jsonl)
        return 1
    recs = read_jsonl(a.plate_jsonl)
    if recs is None:
        print('!! 没有 %s：卡口记录器没在线，识别侧无法核对' % a.plate_jsonl)
        recs = []
    pr_map = {r.get('capture_id'): r for r in recs if r.get('capture_id')}

    if a.capture_id:
        sel = [r for r in passes if r.get('capture_id') == a.capture_id]
        if not sel:
            print('!! 通行记录里没有 capture_id=%s（现有 %d 条）' % (a.capture_id, len(passes)))
            return 1
    else:
        pool = [r for r in passes if not a.gate or r.get('gate_id') == a.gate]
        if not pool:
            print('!! %s 还没有通行记录（总 %d 条）' % (a.gate or '停车场', len(passes)))
            return 1
        sel = pool[-a.n:]
    print('停车场通行 %d 条｜卡口识别 %d 条｜本次核对 %d 条' % (
        len(passes), len(recs), len(sel)))
    for r in sel:
        show(r, pr_map)
    return 0


if __name__ == '__main__':
    sys.exit(main())
