#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""裁掉 room_33 沙盘网格里"停车场前面"的建筑三角面（只删视觉，不碰物理）。

为什么能这么干：沙盘模型 room_33 的 **collision 早就换成了等效盒**（world 里
50×6.7×51.2 的台面盒 + 五个 bldg_* 隐形盒），OBJ 的 470 万面只负责"画"。
所以删掉某几块三角面：车、LiDAR、闸机、地感、识别全都不受影响，变的只有画面。

几何换算沿用 tools/mesh_intrusion_audit.py（已用 bldg_school 的实测轮廓校过）：
  世界 x = 10*obj.x，世界 y = -10*obj.z，高出路面 h = 10*obj.y
判据：一个面的**全部顶点**世界坐标落在某个矩形内、且该面最高点 h >= HMIN 才删。
用"全部顶点"而不是"中心"，是为了不吃到矩形边上那块路面/标线的侧壁。

用法（WSL，仓库根目录）：
  python3 tools/pk173_objcut308.py            # dry-run：数面 + 出前后栅格 PNG
  python3 tools/pk173_objcut308.py --write    # 另存裁剪版，原件一字不动
产物：
  /tmp/pk173_counts.txt        每个矩形的面数
  /tmp/pk173_before_after.png  俯视对比（黑=将被裁掉，白=保留）
  <原件>.cut.obj               裁剪后的新网格
"""
import array
import os
import sys
import time

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OBJ = os.path.join(ROOT, 'src', 'smart_car_sim', 'models', 'room_33',
                   'meshes', 'room_33.obj')
S = 10.0
HMIN = 1.0
CELL = 0.5
WX0, WX1 = 2.0, 22.0
WY0, WY1 = -20.0, -3.0

# 待裁矩形（x0,y0,x1,y1,标签）。轮廓按 tools/pk170_lotbld308.py 的连通块实测值，
# 每块往外留 0.25..0.5 m，免得楼体边缘那圈 1 m 高的裙墙留在原地。
RECTS = [
    (12.5, -15.2, 18.0, -13.5, 'S1 停车场正南(入口车道前面)'),
    (2.5, -19.5, 7.75, -11.5, 'W1 停车场西侧+西南角'),
    (17.5, -12.5, 21.0, -4.0, 'E1 停车场东侧'),
    (5.5, -5.2, 11.5, -0.5, 'N1 停车场正北'),
]


def face_idx(tok, nv):
    """把一行 f 的顶点引用换成 0-based 下标；负数按"已读顶点数"回指。"""
    out = []
    for t in tok:
        s = t.split(b'/')[0]
        try:
            i = int(s)
        except ValueError:
            continue
        out.append(nv + i if i < 0 else i - 1)
    return out


def opt(name, default):
    for a in sys.argv[1:]:
        if a.startswith('--' + name + '='):
            return a.split('=', 1)[1]
    return default


def main():
    write = '--write' in sys.argv
    obj = opt('obj', OBJ)
    only = opt('only', '')
    rects = [r for r in RECTS if not only or r[4][:2] in only.split(',')]
    t0 = time.time()
    vx = array.array('f')
    vy = array.array('f')
    vz = array.array('f')
    tri = array.array('i')
    nface = ncut = 0
    nv = 0

    def rect_hit(x0, x1, y0, y1, hmax):
        if hmax < HMIN:
            return -1
        for k, (rx0, ry0, rx1, ry1, _) in enumerate(rects):
            if x0 >= rx0 and x1 <= rx1 and y0 >= ry0 and y1 <= ry1:
                return k
        return -1

    counts = [0] * len(rects)
    tall_total = 0
    with open(obj, 'rb', buffering=1 << 24) as fh:
        for raw in fh:
            pfx = raw[:2]
            if pfx == b'v ':
                p = raw[2:].split()
                vx.append(float(p[0]))
                vy.append(float(p[1]))
                vz.append(float(p[2]))
                nv += 1
            elif pfx == b'f ':
                nface += 1
                idx = face_idx(raw[1:].split(), nv)
                if len(idx) < 3:
                    continue
                xs = [S * vx[i] for i in idx]
                ys = [-S * vz[i] for i in idx]
                hs = [S * vy[i] for i in idx]
                k = rect_hit(min(xs), max(xs), min(ys), max(ys), max(hs))
                if max(hs) >= HMIN:
                    tall_total += 1
                if k >= 0:
                    counts[k] += 1
                    ncut += 1
                    continue
                # 保留的面：栅格化到 0.5 m 俯视格（取扇形三角化的每个三角形重心）
                for j in range(1, len(idx) - 1):
                    tri.extend((idx[0], idx[j], idx[j + 1]))
                if (nface & 0x7FFFF) == 0:
                    sys.stderr.write('  面 %d  顶点 %d  待删 %d  %.0fs\n' % (
                        nface, nv, ncut, time.time() - t0))
                    sys.stderr.flush()
    print('扫描完成：f 行 %d  顶点 %d  高面(>=%.1f m) %d  待删 %d  %.0fs' % (
        nface, nv, HMIN, tall_total, ncut, time.time() - t0))
    for k, (_, _, _, _, label) in enumerate(rects):
        print('  %-30s 面数 %8d' % (label, counts[k]))

    # 保留面的 0.5 m 栅格
    nc = int((WX1 - WX0) / CELL)
    nr = int((WY1 - WY0) / CELL)
    g = np.zeros(nr * nc, dtype=np.int32)
    if tri:
        t = np.frombuffer(tri, dtype=np.int32).reshape(-1, 3)
        ax = np.array(vx, dtype=np.float32)
        ay = np.array(vy, dtype=np.float32)
        az = np.array(vz, dtype=np.float32)
        cx = ((S * ax[t].mean(axis=1) - WX0) / CELL).astype(np.int32)
        cy = ((-S * az[t].mean(axis=1) - WY0) / CELL).astype(np.int32)
        ok = (cx >= 0) & (cx < nc) & (cy >= 0) & (cy < nr) & (S * ay[t].max(axis=1) >= HMIN)
        g = np.bincount(cy[ok] * nc + cx[ok], minlength=nr * nc).astype(np.int32)
        del t, ax, ay, az
    import cv2
    gm = g.reshape(nr, nc).astype(np.float32)
    gm = np.clip(gm / max(1.0, gm.max()), 0, 1)
    after = (255 - gm * 200).astype(np.uint8)
    cv2.imwrite('/tmp/pk173_after.png',
                cv2.resize(after, (nc * 8, nr * 8), interpolation=cv2.INTER_NEAREST))
    with open('/tmp/pk173_counts.txt', 'w', encoding='utf-8') as fh:
        for k, (_, _, _, _, label) in enumerate(rects):
            fh.write('%s 面数 %d\n' % (label, counts[k]))
        fh.write('合计待删 %d / 全图 f 行 %d\n' % (ncut, nface))
    print('裁剪后俯视栅格 /tmp/pk173_after.png（黑=仍保留的高面，白=空地或已裁掉）')
    print('源文件 %s  f 行 %d  待删 %d' % (obj, nface, ncut))

    if not write:
        print('dry-run 结束（未写文件）。加 --write 才生成裁剪版。')
        return 0

    out = obj + '.cut.obj'
    nv = 0
    written = 0
    dropped = 0
    with open(obj, 'rb', buffering=1 << 24) as fh, \
            open(out + '.part', 'wb', buffering=1 << 24) as oh:
        for raw in fh:
            pfx = raw[:2]
            if pfx == b'v ':
                nv += 1
            elif pfx == b'f ':
                idx = face_idx(raw[1:].split(), nv)
                if len(idx) >= 3:
                    xs = [S * vx[i] for i in idx]
                    ys = [-S * vz[i] for i in idx]
                    hs = [S * vy[i] for i in idx]
                    if rect_hit(min(xs), max(xs), min(ys), max(ys), max(hs)) >= 0:
                        dropped += 1
                        continue
            oh.write(raw)
            written += 1
    os.replace(out + '.part', out)
    print('已写出 %s（%d 行，删掉 %d 面，与扫描计数%s）' % (
        out, written, dropped, '一致' if dropped == ncut else '不一致=异常'))
    return 0 if dropped == ncut else 3


if __name__ == '__main__':
    sys.exit(main())
