#!/usr/bin/env python3
# 把任务五（停车场）的代码从 存档_弃用任务五六_1016 **复制**回工程树（归档保持原样，
# 这样万一中途要再回退，归档还是完整的那一份）。任务六 ETC 的文件一个不碰。
#
# 恢复面（体检结论，见 /tmp/quarantine_parketc_307/inv308.log）：
#   - v2x_msgs 的 Parking/Barrier 接口与 libv2x_barrier.so **从没被搬走**，world md5 未变 → 都不用恢复
#   - 要恢复的只有：6 个节点/自测 py + parking_zones.yaml + 19 个 tools 脚本
#   - 两份 CMakeLists 只补 parking 那 6 行（etc 那 5 行继续留在归档里，否则 build 会因缺件再失败）
#   - vid_rec.py / restart_clean_stack.sh 直接回退到隔离前的原件备份（当时唯一的改动就是摘停车场段）
# 用法：python3 tools/restore_parking_308.py [--dry]
import hashlib
import os
import shutil
import subprocess
import sys

SIM = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
ARCH = os.path.join(SIM, '存档_弃用任务五六_1016')
HOME_PKG = '/home/hzm/v2x_ws/src/v2x_demo'

NODES = [
    'parking_manager_node.py',
    'parking_manager_selftest.py',
    'parking_recorder_node.py',
    'parking_recorder_selftest.py',
    'parking_shuttle_node.py',
    'parking_shuttle_selftest.py',
]
TOOLS = [
    'build_parking.sh', 'check_parking_layout.py', 'gen_parking_world.py', 'parking_cli.py',
    'parking_points_dump.py', 'parking_rec_selftest.py', 'parking_shuttle_selftest.py',
    'parking_shuttle_selftest.sh', 'restart_parking89.sh', 'restart_parking91.sh',
    'start_parking_nodes.sh', 'survey_parking_coverage.py', 'survey_parking_footprint.py',
    'vid_parking_1.sh', 'vid_parking_2.sh', 'vid_parking_3.sh', 'vid_parking_4.sh',
    'vid_parking_5.sh', 'vid_parking_preflight.sh',
]
def md5(p):
    h = hashlib.md5()
    with open(p, 'rb') as f:
        for blk in iter(lambda: f.read(1 << 16), b''):
            h.update(blk)
    return h.hexdigest()


def arch_path(rel):
    """归档里的实际路径：home 侧那批有一层 <name>/x 的套娃（隔离第 7 步的 mkdir 口径留下的）。"""
    p = os.path.join(ARCH, rel)
    if os.path.isfile(p):
        return p
    n = os.path.join(p, 'x')
    return n if os.path.isfile(n) else None


def copy(src, dst, dry):
    if os.path.exists(dst) and md5(src) != md5(dst):
        print('  !! 目标已存在且内容不同，不动：%s' % dst)
        return False
    if os.path.exists(dst):
        print('  已存在同内容：%s' % dst)
        return False
    if not dry:
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
        if dst.endswith(('.py', '.sh')):
            os.chmod(dst, 0o755)
        assert md5(src) == md5(dst), dst
    print('  %s：%s（%d 字节 md5=%s）' % ('已恢复' if not dry else '待恢复', dst,
                                        os.path.getsize(src), md5(src)[:12]))
    return True


def patch_cmake(p, dry):
    """在 install(PROGRAMS) 列表末尾补回 parking 那 6 行；etc 的行不补。"""
    with open(p, encoding='utf-8', newline='') as f:
        txt = f.read()
    eol = '\r\n' if '\r\n' in txt else '\n'
    have = [n for n in NODES if 'v2x_demo/' + n in txt]
    miss = [n for n in NODES if n not in have]
    print('  %s：已在册 %d，待补 %d %s' % (p, len(have), len(miss), miss))
    if not miss:
        return
    anchor = '    v2x_demo/speed_selftest.py'
    if anchor not in txt.replace('\r\n', '\n'):
        print('  !! 找不到锚点，不自动改，请手工插')
        return
    if dry:
        print('    DRY：会在 speed_selftest.py 之后插 %d 行' % len(miss))
        return
    bak = p + '.pre_restore308.bak'
    if not os.path.exists(bak):
        shutil.copy2(p, bak)
    add = eol.join(['    v2x_demo/' + n for n in miss])
    norm = txt.replace('\r\n', '\n')
    norm = norm.replace(anchor, anchor + eol + add.replace('\r\n', '\n'), 1)
    with open(p, 'w', encoding='utf-8', newline='') as f:
        f.write(norm.replace('\n', eol) if eol == '\r\n' else norm)
    print('    已补（原件备份 %s）' % os.path.basename(bak))


def restore_from_backup(name, dst, dry):
    src = os.path.join(ARCH, name)
    if not os.path.exists(src):
        print('  !! 归档里没有 %s' % name)
        return
    if os.path.exists(dst) and md5(src) == md5(dst):
        print('  %s 已是隔离前那份' % os.path.basename(dst))
        return
    cur = md5(dst)[:12] if os.path.exists(dst) else '不存在'
    print('  %s：现 md5=%s -> 回退为 md5=%s（%d 字节）' % (
        os.path.basename(dst), cur, md5(src)[:12], os.path.getsize(src)))
    if dry:
        print('    DRY：未写')
        return
    bak = dst + '.pre_restore308.bak'
    if os.path.exists(dst) and not os.path.exists(bak):
        shutil.copy2(dst, bak)
    shutil.copy2(src, dst)
    if dst.endswith('.sh'):
        os.chmod(dst, 0o755)
        print('    bash -n：%s' % ('OK' if subprocess.run(
            ['bash', '-n', dst], capture_output=True).returncode == 0 else '失败'))
    elif dst.endswith('.py'):
        print('    py_compile：%s' % ('OK' if subprocess.run(
            ['python3', '-m', 'py_compile', dst], capture_output=True).returncode == 0 else '失败'))


def main():
    dry = '--dry' in sys.argv
    print('模式：%s' % ('DRY 只点名' if dry else 'APPLY 复制回树'))

    print('===== 1) D 盘真值树：6 个节点/自测 + parking_zones.yaml =====')
    for n in NODES:
        s = arch_path('src/v2x_demo/v2x_demo/' + n)
        if s:
            copy(s, os.path.join(SIM, 'src/v2x_demo/v2x_demo', n), dry)
        else:
            print('  !! 归档缺 %s' % n)
    s = arch_path('src/v2x_demo/config/parking_zones.yaml')
    copy(s, os.path.join(SIM, 'src/v2x_demo/config/parking_zones.yaml'), dry)

    print('===== 2) D 盘 tools：19 个停车场脚本 =====')
    for n in TOOLS:
        s = arch_path('tools/' + n)
        if s:
            copy(s, os.path.join(SIM, 'tools', n), dry)
        else:
            print('  !! 归档缺 tools/%s' % n)

    print('===== 3) home 编译副本（colcon 编的是这一份）=====')
    for n in NODES:
        s = arch_path('wsl_home/v2x_demo/' + n)
        if s:
            copy(s, os.path.join(HOME_PKG, 'v2x_demo', n), dry)
        else:
            print('  !! 归档缺 home 侧 %s（D 版可当源，但行尾符会变）' % n)
    s = arch_path('wsl_home/config/parking_zones.yaml')
    copy(s, os.path.join(HOME_PKG, 'config', 'parking_zones.yaml'), dry)

    print('===== 4) 两份 CMakeLists 补回 parking 的 6 行（etc 不补）=====')
    patch_cmake(os.path.join(SIM, 'src/v2x_demo/CMakeLists.txt'), dry)
    patch_cmake(os.path.join(HOME_PKG, 'CMakeLists.txt'), dry)

    print('===== 5) 共享文件回退到隔离前那份 =====')
    restore_from_backup('vid_rec.py.pre_quarantine', os.path.join(SIM, 'tools/vid_rec.py'), dry)
    restore_from_backup('restart_clean_stack.sh.pre_quarantine',
                        os.path.join(SIM, 'tools/restart_clean_stack.sh'), dry)

    print('===== 6) 落盘后自检 =====')
    if dry:
        print('  DRY：跳过')
        return
    for n in NODES:
        r = subprocess.run(['python3', '-m', 'py_compile',
                            os.path.join(SIM, 'src/v2x_demo/v2x_demo', n)], capture_output=True)
        print('  py_compile %-32s %s' % (n, 'OK' if r.returncode == 0 else r.stderr.decode()[:200]))
    b = subprocess.run(['bash', '-lc',
                        'source /opt/ros/humble/setup.bash && cd /home/hzm/v2x_ws && '
                        'colcon build --packages-select v2x_demo 2>&1 | tail -8'],
                       capture_output=True, text=True)
    print('  colcon build 回包：')
    for ln in (b.stdout or b.stderr or '').splitlines():
        print('    ' + ln)
    lib = '/home/hzm/v2x_ws/install/v2x_demo/lib/v2x_demo'
    got = sorted(x for x in os.listdir(lib) if x.startswith('parking_'))
    print('  install 里的 parking_ 节点：%s' % got)
    cfg = '/home/hzm/v2x_ws/install/v2x_demo/share/v2x_demo/config'
    print('  install 里的 parking_zones.yaml：%s' % (
        '有' if os.path.exists(os.path.join(cfg, 'parking_zones.yaml')) else '无'))
    for p in (os.path.join(SIM, 'src/v2x_demo/CMakeLists.txt'), os.path.join(HOME_PKG, 'CMakeLists.txt')):
        c = subprocess.run(['grep', '-c', 'parking_', p], capture_output=True, text=True)
        e = subprocess.run(['grep', '-c', 'etc_', p], capture_output=True, text=True)
        print('  %s：parking_ %s 行、etc_ %s 行（etc 应为 0）' % (p, c.stdout.strip(), e.stdout.strip()))


if __name__ == '__main__':
    main()
