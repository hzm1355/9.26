#!/usr/bin/env python3
# 把打包文件（v2x_demo 是 ament_cmake 包，引用在 CMakeLists.txt 的 install(PROGRAMS/FILES)
# 列表里；若哪天换成 ament_python 则是 setup.py）里指向已隔离脚本/配置的**整行**摘掉。
#
# 安全护栏（这是删东西，不是加东西）：
#   1) 只删"这一行里出现的每一个 .py/.yaml 都是 parking_/etc_"的行；
#      一旦某行同时带着别的文件（比如 config 一把梭的 install(FILES ...)），整行**不动**，
#      改成打印出来要人工改，绝不为了少改一行而把任务一~四的配置一起带走。
#   2) 改了哪几行全部打印，行数对账。
# 用法：python3 quarantine_patch_pack307.py <CMakeLists.txt 或 setup.py>
import re
import sys

ENTRY = re.compile(r'(?:v2x_demo/)?((?:parking_|etc_)[A-Za-z0-9_]*)\.(py|yaml)')
ANYFILE = re.compile(r'[A-Za-z0-9_]+\.(py|yaml|sh|cpp|so|msg|srv)')


def main():
    p = sys.argv[1]
    with open(p, encoding='utf-8') as f:
        lines = f.readlines()
    kept, cut, hold = [], [], []
    for ln in lines:
        names = [m.group(0) for m in ANYFILE.finditer(ln)]
        if not names:
            kept.append(ln)
            continue
        if not ENTRY.search(ln):
            kept.append(ln)
            continue
        all_iso = all(re.match(r'^(parking_|etc_)', n) for n in names)
        if all_iso:
            cut.append(ln)
        else:
            hold.append(ln)
            kept.append(ln)
    with open(p, 'w', encoding='utf-8') as f:
        f.writelines(kept)
    print('%s：原 %d 行 -> 现 %d 行，摘掉 %d 行' % (p.split('/')[-1], len(lines), len(kept), len(cut)))
    for ln in cut:
        print('    - ' + ln.strip())
    if hold:
        print('  !! 有 %d 行混着非 parking/etc 的文件，本脚本**没动**，要人工处理：' % len(hold))
        for ln in hold:
            print('    ? ' + ln.strip())


if __name__ == '__main__':
    main()
