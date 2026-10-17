#!/usr/bin/env python3
# 恢复前的点名：归档里 263 项各自属于任务五（停车场）还是任务六（ETC），
# 以及"现在工程树里这个路径是不是空的""还有谁在引用它"。
# 只读，一个字节不写。用法：python3 tools/restore_pk_inventory_308.py
import os
import re
import subprocess

SIM = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim'
ARCH = os.path.join(SIM, '存档_弃用任务五六_1016')
HOME_PKG = '/home/hzm/v2x_ws/src/v2x_demo'

# 任务五专属：文件名带 parking_ 或 gen_parking，或内容只讲停车场
PK = re.compile(r'parking', re.I)
ET = re.compile(r'\betc|etc_', re.I)


def walk(root):
    out = []
    for dirpath, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if d not in ('.git',)]
        for fn in files:
            out.append(os.path.join(dirpath, fn))
    return out


def main():
    if not os.path.isdir(ARCH):
        print('归档不存在：%s' % ARCH)
        return
    entries = walk(ARCH)
    top = {}
    for p in entries:
        rel = os.path.relpath(p, ARCH).replace('\\', '/')
        bucket = rel.split('/')[0] if '/' in rel else '(顶层)'
        top.setdefault(bucket, []).append(rel)

    print('===== 归档顶层分布 =====')
    for b in sorted(top):
        print('  %-24s %d 项' % (b, len(top[b])))

    # 真正的"代码件"= 顶层 src/ tools/ wsl_home/ 里、非 .bak/.pyc 的文件
    code = []
    for p in entries:
        rel = os.path.relpath(p, ARCH).replace('\\', '/')
        if rel.endswith(('.bak', '.pyc', '.tgz', '.txt', '.md')) or '/pre_' in rel \
           or '.pre_' in rel or rel.startswith('_'):
            continue
        if rel.startswith(('src/', 'tools/', 'wsl_home/')):
            code.append(rel)
    print('\n===== 可恢复代码件（去掉备份/清单/归档包）：%d 项 =====' % len(code))

    pk, et, both, neither = [], [], [], []
    for rel in code:
        b = os.path.basename(rel)
        is_pk = bool(PK.search(b)) or 'parking_zones' in rel
        is_et = bool(re.match(r'^(etc_|gen_etc)', b)) or 'etc_zones' in rel
        # ETC 的 tools 脚本名常带 etc，但也可能只在扩展名前
        if not is_et:
            is_et = bool(re.search(r'(^|_)etc($|_)', b, re.I))
        if is_pk and is_et:
            both.append(rel)
        elif is_pk:
            pk.append(rel)
        elif is_et:
            et.append(rel)
        else:
            neither.append(rel)

    def dump(title, lst, need_path=True):
        print('\n----- %s：%d 项 -----' % (title, len(lst)))
        for rel in sorted(lst):
            src = os.path.join(ARCH, rel)
            size = os.path.getsize(src)
            note = ''
            if need_path:
                if rel.startswith('wsl_home/'):
                    tgt = os.path.join(HOME_PKG, rel.split('/', 1)[1])
                elif rel.startswith(('src/', 'tools/')):
                    tgt = os.path.join(SIM, rel)
                else:
                    tgt = None
                if tgt:
                    note = '  现路径:%s' % ('空（可直接放回）' if not os.path.exists(tgt) else '已被占用!!')
            print('  %-72s %7d%s' % (rel, size, note))

    dump('任务五（停车场）专属', pk)
    dump('任务六（ETC）专属', et)
    dump('两个都提到（要人工定性）', both)
    dump('文件名两样都不带（多为 tools 勘查脚本）', neither)

    # v2x_msgs / 插件是否被动过 —— 这两处若被搬走，恢复面比节点大
    print('\n===== 关键依赖体检：v2x_msgs 与 barrier 插件 =====')
    msgs = os.path.join(SIM, 'src/v2x_msgs')
    if os.path.isdir(msgs):
        have = sorted(os.listdir(os.path.join(msgs, 'msg'))) + sorted(os.listdir(os.path.join(msgs, 'srv'))) \
            if os.path.isdir(os.path.join(msgs, 'msg')) else []
        print('  v2x_msgs 现存接口 %d 项' % len(have))
        print('  其中 Parking/Barrier/Pk 字样：%s' % [h for h in have if PK.search(h) or 'Barrier' in h])
        print('  其中 Etc 字样：%s' % [h for h in have if 'Etc' in h])
        cm = open(os.path.join(msgs, 'CMakeLists.txt'), encoding='utf-8').read()
        print('  CMakeLists 里 Parking 行 %d 条、Etc 行 %d 条' %
              (len([l for l in cm.splitlines() if 'Parking' in l or 'Barrier' in l]),
               len([l for l in cm.splitlines() if 'Etc' in l])))
    arch_msgs = [r for r in code if 'v2x_msgs' in r]
    print('  归档里属于 v2x_msgs 的：%d 项' % len(arch_msgs))
    for r in sorted(arch_msgs):
        print('    %s' % r)

    plug = [r for r in code if 'gazebo_plugins' in r or 'barrier' in r.lower()]
    print('\n===== 归档里与 barrier 插件相关的：%d 项 =====' % len(plug))
    for r in sorted(plug):
        print('    %s' % r)
    so = os.path.join('/home/hzm/v2x_ws/install/v2x_gazebo_plugins/lib')
    if os.path.isdir(so):
        print('  install 插件目录：%s' % sorted(os.listdir(so)))

    print('\n===== world 现状（应未被隔离动过）=====')
    w = os.path.join(SIM, 'src/smart_car_sim/worlds/v2x_city_scene.world')
    r = subprocess.run(['grep', '-c', 'parking_gate', w], capture_output=True, text=True)
    print('  parking_gate 字样行数：%s｜md5=%s' % (
        r.stdout.strip(), subprocess.run(['md5sum', w], capture_output=True, text=True).stdout.split()[0]))


if __name__ == '__main__':
    main()
