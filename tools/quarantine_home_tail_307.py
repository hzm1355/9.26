#!/usr/bin/env python3
# 收尾：apply 第 7 步只把 home src 的 6 个 parking/etc **节点**移进了归档，
# 5 个 *_selftest.py 漏了 —— 于是 install 里那 5 条符号链接既没断链（目标还在）
# 也没被摘掉（CMakeLists 已不再列它们），colcon 重编不会碰它们，就永远留在产物里。
#
# 这一步做三件事，全部对账后再落盘：
#   1) 把 home src 里剩下的 parking_/etc_ 实体文件移进归档 wsl_home/v2x_demo/（沿用第 7 步命名）
#   2) 顺手清掉 home src __pycache__ 里的同源 .pyc（留着的话 import 还能蒙对，读数会骗人）
#   3) 删 install 里指向这些文件的符号链接（此刻它们已随第 1 步变成断链）
# 安全护栏：只动 basename 以 parking_/etc_ 开头的；其它一条不碰。
import hashlib
import os
import shutil
import subprocess
import sys

HOME_PKG = '/home/hzm/v2x_ws/src/v2x_demo'
ARCH = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/存档_弃用任务五六_1016'
DEST = os.path.join(ARCH, 'wsl_home', 'v2x_demo')
INSTALL_LIB = '/home/hzm/v2x_ws/install/v2x_demo/lib/v2x_demo'
D_TREE = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/v2x_demo'


def md5(p):
    h = hashlib.md5()
    with open(p, 'rb') as f:
        for blk in iter(lambda: f.read(1 << 16), b''):
            h.update(blk)
    return h.hexdigest()


def main():
    if '--dry' in sys.argv:
        print('DRY：只点名，不移动不删除')
    else:
        os.makedirs(DEST, exist_ok=True)

    print('===== 1) home src 里剩下的 parking_/etc_ 实体 =====')
    moved = []
    for root, dirs, files in os.walk(HOME_PKG):
        if '__pycache__' in root:
            continue
        for fn in sorted(files):
            if not (fn.startswith('parking_') or fn.startswith('etc_')):
                continue
            src = os.path.join(root, fn)
            rel = os.path.relpath(root, HOME_PKG)
            dside = os.path.join(D_TREE, fn) if rel == 'v2x_demo' else None
            same = '归档/ D盘有同名:%s' % ('是' if dside and os.path.exists(dside) else '否')
            if dside and os.path.exists(dside):
                same += ' 内容一致(去行尾符):%s' % (
                    '是' if subprocess.run(['diff', '--strip-trailing-cr', '-q', dside, src],
                                          capture_output=True).returncode == 0 else '否')
            print('  %s/%s  %d 字节  md5=%s  %s' % (
                rel, fn, os.path.getsize(src), md5(src), same))
            moved.append((src, fn))
    print('  小计 %d 个' % len(moved))

    print('===== 2) home src __pycache__ 里的同源 .pyc =====')
    cache = os.path.join(HOME_PKG, 'v2x_demo', '__pycache__')
    pycl = []
    if os.path.isdir(cache):
        for fn in sorted(os.listdir(cache)):
            if fn.startswith('parking_') or fn.startswith('etc_'):
                pycl.append(os.path.join(cache, fn))
                print('  %s  %d 字节' % (fn, os.path.getsize(os.path.join(cache, fn))))
    print('  小计 %d 个' % len(pycl))

    print('===== 3) install 里指向以上文件的符号链接 =====')
    links = []
    for b in sorted(os.listdir(INSTALL_LIB)):
        if b.startswith('parking_') or b.startswith('etc_'):
            links.append(os.path.join(INSTALL_LIB, b))
            tgt = os.readlink(os.path.join(INSTALL_LIB, b))
            print('  %s -> %s（目标存在:%s）' % (b, tgt, '是' if os.path.exists(os.path.join(INSTALL_LIB, b)) else '否'))
    print('  小计 %d 条' % len(links))

    if '--dry' in sys.argv:
        print('DRY 结束：一个字节没动')
        return

    print('===== 4) 落盘：移动 home 实体（先归档后删原路径）=====')
    for src, fn in moved:
        dst = os.path.join(DEST, fn)
        if os.path.exists(dst):
            print('  !! 归档里已有 %s，跳过覆盖，只删 home 原件' % fn)
            os.remove(src)
            continue
        shutil.copy2(src, dst)
        assert md5(dst) == md5(src), fn
        os.remove(src)
        print('  已移入归档：wsl_home/v2x_demo/%s' % fn)

    print('===== 5) 落盘：删 .pyc 与 install 符号链接 =====')
    for p in pycl:
        os.remove(p)
        print('  删 pyc：%s' % os.path.basename(p))
    for p in links:
        os.remove(p)
        print('  删链接：%s' % os.path.basename(p))

    print('===== 6) 效果复核（三处计数都应 0）=====')
    rest = []
    for root, dirs, files in os.walk(HOME_PKG):
        for fn in files:
            if fn.startswith('parking_') or fn.startswith('etc_'):
                rest.append(os.path.join(root, fn))
    print('  home src 里 parking_/etc_ 实体：%d（应 0）' % len(rest))
    for p in rest:
        print('    还剩 %s' % p)
    li = [b for b in os.listdir(INSTALL_LIB) if b.startswith(('parking_', 'etc_'))]
    print('  install/lib 里 parking_/etc_：%d（应 0）%s' % (len(li), li))
    ds = [b for b in os.listdir(D_TREE) if b.startswith(('parking_', 'etc_'))]
    print('  D 盘真值树里 parking_/etc_：%d（应 0）%s' % (len(ds), ds))
    print('  归档 wsl_home/v2x_demo 现有：%s' % sorted(os.listdir(DEST)))


if __name__ == '__main__':
    main()
