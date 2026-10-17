#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""car_7 的三处身份体检：路线（signal_drive_params.yaml）、位姿（etc_zones.yaml 的 testcar）、
台账（plates + accounts）。三份配置分开写，任何一处漏改都会在现场变成一种具体的假象：

  * 只有 testcar 没有路线 —— 车 spawn 出来了但没有任何发布者开它，画面里"ETC 取证车停着不动"；
  * 只有路线没有 testcar —— 驱动节点按路线给一辆不存在的模型发 cmd_vel，话题在、车不动；
  * 有路线有 testcar 但没台账 —— RSU 认不出 OBU，判"未办理 ETC"，杆不抬，而车身顶 0.45 m
    高于道闸放平下缘 0.27 m，车会物理顶杆并把 Ring A 西腿（任务四车流/任务五摆渡共用腿）堵死。

用法: car7_cfg_check.py [signal_drive_params.yaml] [etc_zones.yaml] [--negtest]
      --negtest 不查现场，只拿当前配置造 7 支负样本，逐条确认体检真的检得出来。
退出码 0 CAR7_CFG_OK（或负样本全部被检出）/ 1 缺身份、不一致、或有负样本检不出来
"""
import copy
import sys

import yaml

DEFAULT_DRIVE = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/config/signal_drive_params.yaml'
DEFAULT_ETC = '/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/v2x_demo/config/etc_zones.yaml'


def check(cars, etc):
    errs = []

    car = cars.get('car_7')
    if car is None:
        errs.append('signal_drive 的 cars 里没有 car_7')
        route = None
    else:
        route = car.get('route')
        if not route:
            errs.append('car_7 没有 route')
        if car.get('mode') != 'loop':
            errs.append('car_7 的 mode=%r，任务六要的是沿西腿每圈自发过闸一次，必须是 loop'
                        % car.get('mode'))
        if not car.get('drive', True):
            errs.append('car_7 被标了 drive:false，signal_drive 装载循环会跳过它')

    test = etc.get('testcar')
    if not test:
        errs.append('etc_zones.yaml 里没有 testcar 段')
    elif test.get('model') != 'car_7':
        errs.append('testcar.model=%r，不是 car_7' % test.get('model'))
    else:
        if len(test.get('spawn', [])) != 2:
            errs.append('testcar.spawn 不是一个 [x, y]')
        if test.get('yaw') is None or test.get('z') is None:
            errs.append('testcar 缺 yaw 或 z，spawn_etc_car7.sh 的 --pose 拿不到四个数')

    plate = (etc.get('plates') or {}).get('car_7')
    if not plate:
        errs.append('plates 里没有 car_7 的车牌')

    acc = [a for a in (etc.get('accounts') or []) if a.get('model') == 'car_7']
    if len(acc) != 1:
        errs.append('accounts 里 model=car_7 的条目 %d 条，应为 1 条' % len(acc))
    else:
        a = acc[0]
        if plate and a.get('plate') != plate:
            errs.append('台账车牌 %r 与 plates 台账 %r 不一致，RSU 会把两次识别绑不到同一辆车'
                        % (a.get('plate'), plate))
        if a.get('status') != 'signed':
            errs.append('car_7 的 OBU status=%r，非 signed 会按挂失/未签约处理，不自动抬杆'
                        % a.get('status'))
        # 余额窗口：扣费口径是 toll.fee_by_class[vehicle_class] 元/次（etc_manager_node.py:246
        # 读的就是这一项），记账卡欠费到 toll.arrears_limit 转拒入、储值卡余额不足直接拒。
        # 录五段视频要在 Ring A 上绕几十圈（一圈 ~113 s），这条不设限就会在录制中途把西腿拦死。
        toll = etc.get('toll') or {}
        # yaml 里 fee_by_class 的键是**裸数字**（{1: 5.00, 2: 10.00}），safe_load 会解析成 int；
        # 管理器也是按 int 键取的（etc_manager_node.py:246 `{int(k): float(v) ...}`）。
        # 这里先按 str 查会把一条写好的费率读成"没有费率"——所以两边都归一到 int。
        fees = {int(k): float(v) for k, v in (toll.get('fee_by_class') or {}).items()}
        fee = fees.get(int(a.get('vehicle_class') or 0), None)
        if fee is None:
            errs.append('toll.fee_by_class 里没有车种 %r 的费率，管理器会拿默认值收费'
                        % a.get('vehicle_class'))
        elif a.get('card_type') == '储值卡':
            bal = a.get('balance')
            if bal is None:
                errs.append('储值卡没有 balance 字段')
            elif float(bal) < float(fee):
                errs.append('储值卡余额 %s < 单次费用 %s，第一圈就被 insufficient_balance_action 拒掉'
                            % (bal, fee))
            else:
                print('  储值卡 %s：余额 %s / 单次 %s → 约 %d 次的通行余量'
                      % (a.get('obu_id'), bal, fee, int(float(bal) / float(fee))))
        elif a.get('card_type') == '记账卡':
            lim = toll.get('arrears_limit')
            if lim is None:
                errs.append('记账卡但 toll.arrears_limit 缺失，判不了欠费')
            else:
                bal = a.get('balance') or 0.0
                laps = (float(bal) - float(lim)) / float(fee)
                if laps < 30:
                    errs.append('记账卡 %s：余额 %s 距欠费线 %s 只够约 %d 圈，录制窗口不够'
                                % (a.get('obu_id'), bal, lim, int(laps)))

    # 几何耦合：识别触发点在车道的 y 上，占用框由车体 (L+W)/2 算，两处必须同向。
    lane = etc.get('lane') or {}
    if route and lane:
        cx = lane.get('center_x')
        if cx is not None and all(abs(p[0] - cx) > 0.5 for p in route):
            errs.append('car_7 的路线没有任何航点落在车道 center_x=%s 附近，过不了闸' % cx)

    return errs


def load(dp, ep):
    with open(dp, encoding='utf-8') as f:
        cars = yaml.safe_load(f)['signal_drive']['cars']
    with open(ep, encoding='utf-8') as f:
        etc = yaml.safe_load(f)['etc']   # 根键是 etc（etc_rsu_node.py:184 取的就是这一个）
    return cars, etc


# 负样本：每一支都只改一处，改完必须被检出。判据脚本拿已通过的数据校准之外，还要拿
# "故意做错"的样本校准一次 —— 否则一条永远为真的检查会给出完全相反的假绿。
NEGATIVES = [
    ('删掉 signal_drive 里的 car_7', 'cars', lambda c, e: c.pop('car_7')),
    ('car_7 被标 drive:false', 'cars', lambda c, e: c['car_7'].update({'drive': False})),
    ('testcar 段整段缺失', 'etc', lambda c, e: e.pop('testcar')),
    ('plates 里没有 car_7', 'etc', lambda c, e: e['plates'].pop('car_7')),
    ('台账车牌与 plates 台账不一致', 'etc',
     lambda c, e: [a for a in e['accounts'] if a['model'] == 'car_7'][0].update({'plate': '沪X00000'})),
    ('OBU 状态改成挂失', 'etc',
     lambda c, e: [a for a in e['accounts'] if a['model'] == 'car_7'][0].update({'status': 'lost'})),
    ('储值卡余额改成 0（第一圈就被拒）', 'etc',
     lambda c, e: [a for a in e['accounts'] if a['model'] == 'car_7'][0].update({'balance': 0.0})),
]


def negtest(cars, etc):
    bad = 0
    for label, which, mutate in NEGATIVES:
        c = copy.deepcopy(cars)
        e = copy.deepcopy(etc)
        try:
            mutate(c, e)
        except Exception as exc:
            print('  NEGERR %s —— 构造负样本本身失败：%s' % (label, exc))
            bad += 1
            continue
        errs = check(c, e)
        if errs:
            print('  NEG_OK   %-34s -> %s' % (label, errs[0][:78]))
        else:
            print('  NEG_MISS %-34s -> 检不出来（这条检查是摆设）' % label)
            bad += 1
    return bad


def main():
    # 开关与路径分开收：`--negtest` 混在位置参数里会被当成第二个路径（docstring 里的用法
    # 必须能真跑，所以这里不靠调用顺序）。
    paths = [a for a in sys.argv[1:] if not a.startswith('--')]
    dp = paths[0] if paths else DEFAULT_DRIVE
    ep = paths[1] if len(paths) > 1 else DEFAULT_ETC
    cars, etc = load(dp, ep)
    if '--negtest' in sys.argv:
        return 1 if negtest(cars, etc) else 0
    errs = check(cars, etc)
    if errs:
        for e in errs:
            print('  CFGERR %s' % e)
        print('car_7 身份体检没过')
        return 1
    print('CAR7_CFG_OK：car_7 的路线/位姿/车牌/台账四处齐备且互相一致（%s, %s）' % (dp, ep))
    return 0


if __name__ == '__main__':
    sys.exit(main())
