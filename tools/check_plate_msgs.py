#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""校验任务三新增的 msg/srv 接口是否按定义生成、字段是否齐。"""
from v2x_msgs.msg import PlateAlert, PlateCapture, PlateResult
from v2x_msgs.srv import SetBlacklist, SetPlate

for name, t in [('PlateCapture', PlateCapture), ('PlateResult', PlateResult),
                ('PlateAlert', PlateAlert)]:
    print('%-14s %s' % (name, [f for f in t.get_fields_and_field_types()]))

for name, t in [('SetPlate', SetPlate), ('SetBlacklist', SetBlacklist)]:
    print('%-14s REQ  %s' % (name, [f for f in t.Request.get_fields_and_field_types()]))
    print('%-14s RESP %s' % ('', [f for f in t.Response.get_fields_and_field_types()]))

# 常量与默认值抽查：这些是下游节点直接依赖的字面量
assert PlateAlert.LEVEL_CRITICAL == 2 and PlateAlert.LEVEL_INFO == 0
assert SetBlacklist.Request.OP_LIST == 3 and SetBlacklist.Request.OP_ADD == 0
r = PlateResult()
assert r.plate_number == '' and r.matched is False and r.confidence == 0.0
print('常量/默认值抽查 OK')
