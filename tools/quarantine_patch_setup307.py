#!/usr/bin/env python3
# 把 setup.py 的 console_scripts 里指向已隔离模块（parking_* / etc_*）的整行摘掉。
# 只按"等号右侧模块名命中"删行，别的行一个字不动；删掉的行数会打印出来供对账。
# 用法：python3 quarantine_patch_setup307.py <setup.py 路径>
import re
import sys

PAT = re.compile(r'=\s*(?:v2x_demo\.)?(?:parking_|etc_)[A-Za-z0-9_.]*:main')


def main():
    p = sys.argv[1]
    with open(p, encoding='utf-8') as f:
        lines = f.readlines()
    kept, cut = [], []
    for ln in lines:
        (cut if PAT.search(ln) else kept).append(ln)
    with open(p, 'w', encoding='utf-8') as f:
        f.writelines(kept)
    print('摘掉 %d 行 entry_point（原 %d 行 -> 现 %d 行）' % (len(cut), len(lines), len(kept)))
    for ln in cut:
        print('    - ' + ln.strip())


if __name__ == '__main__':
    main()
