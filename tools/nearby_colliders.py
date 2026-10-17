#!/usr/bin/env python3
# 按坐标粗筛 world 里落在某个位置附近的静态碰撞件。
#
# 为什么要独立成文件而不是写在 .sh 的内联 heredoc 里：memory
# 「feedback-write-script-files-not-heredoc」—— 内联 heredoc 在这条 WSL/Git Bash
# 管道上会挂死也会被分类器拦掉，之前 152 轮的 stuck_look152.sh 就因为这个没跑成。
#
# 判读口径：只看 <model> 级 <pose> 和"这个 model 里有没有 <collision>"。这是粗筛，
# 目的是回答"车旁边有哪几座设施"，而不是精确算接触；精确占地要再读那个 model 的
# link/geometry。挑出候选之后才谈得上"改路线绕开它"还是"它根本没碰车"。
import re
import sys


def main():
    if len(sys.argv) < 4:
        print('用法: nearby_colliders.py <world> <x> <y> [半径=3.5]')
        return 2
    path = sys.argv[1]
    cx, cy = float(sys.argv[2]), float(sys.argv[3])
    radius = float(sys.argv[4]) if len(sys.argv) > 4 else 3.5

    with open(path, encoding='utf-8', errors='replace') as f:
        txt = f.read()

    num = r'[-\d.eE+]+(?:\d)'
    hits = 0
    for m in re.finditer(r'<model name="([^"]+)">(.*?)</model>', txt, re.S):
        name, body = m.group(1), m.group(2)
        p = re.search(r'<pose>\s*(%s)\s+(%s)\s+(%s)\s+(%s)\s+(%s)\s+(%s)\s*</pose>'
                      % ((num,) * 6), body)
        if not p or '<collision' not in body:
            continue
        x, y = float(p.group(1)), float(p.group(2))
        dx, dy = x - cx, y - cy
        if abs(dx) <= radius and abs(dy) <= radius:
            ncol = body.count('<collision')
            print('  %-32s pose=(%7.2f,%7.2f) 距车 %.2f m  碰撞件 %d 个'
                  % (name, x, y, (dx * dx + dy * dy) ** 0.5, ncol))
            hits += 1
    print('  --- 命中 %d 座（半径 %.1f m 方形窗口内）---' % (hits, radius))
    return 0


if __name__ == '__main__':
    sys.exit(main())
