#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把 live_view_server.py 里的 PAGE 内联 <script> 抽成一个 .js 文件，交给 node --check 语法校验。
看板是单文件 HTML 内嵌 JS，浏览器里才报错，而这里没有 GUI 可点，所以先做静态语法关。
用法：python3 check_dashboard_js.py [输出路径]
"""
import io
import os
import re
import sys

# 从脚本自身位置推：写死 D:/... 在 WSL 里跑就是 FileNotFoundError（两边路径根不同）。
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
SRC = os.path.join(ROOT, 'live_view_server.py')
OUT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(ROOT, '_dash_page.js')

src = io.open(SRC, encoding='utf-8').read()
page = re.search(r"PAGE = r'''(.*?)'''", src, re.S)
if not page:
    sys.exit('没找到 PAGE 原始字符串块')
body = re.search(r'<script>(.*?)</script>', page.group(1), re.S)
if not body:
    sys.exit('PAGE 里没有 <script> 块')
js = body.group(1)
io.open(OUT, 'w', encoding='utf-8', newline='\n').write(js)
print('JS 行数 %d -> %s' % (js.count('\n') + 1, OUT))
for name in ('renderSpeed', 'spdzones', 'spdrows', 'spdal'):
    print('  含 %-11s : %s' % (name, name in js))
