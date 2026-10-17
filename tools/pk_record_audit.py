#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务五（五）条目 5 的在环对账器：查询、导出、落盘账本三方必须互相对得上。

为什么不能只看"服务回了 success=True"：
  记录器有**三份账**——JSONL（真相，追加式，跨重启不丢）、内存环形表（查询和导出的数据源，
  上限 max_records_in_memory，超了丢最旧）、CSV（导出时按当前口径投影出来的快照）。
  三者本来就**不该相等**：溢出后 CSV 会比 JSONL 少，only_new 时 CSV 是增量。所以这里每一处
  比较都点名口径，并把"本应不等"的那几种情况写成独立判据，而不是笼统一句"行数一致"。

对账内容（全部是"节点说的数"对"我自己从 JSONL 重算的数"，不是自证）：
  A 账本可读且非空（条目 5 的证据必须是真过杆产生的记录，空账本直接 FAIL，不给"通过"）。
  B 全量查询 matched/returned/stored 三个数各自等于什么，含溢出分支。
  C 过滤矩阵：方向/结果/车牌子串/车辆/闸机/时段/组合，逐项与本地按**文档口径**重算的命中数比。
     这一项是专门用来咬 ParkingQuery 的 DIR_*/RESULT_* 常量比 ParkingPass 大 1 那件事的——
     漏了映射不会报错，只会把"只查放行"筛成一堆拒绝，而回包照样 success=True。
  D 排序与 limit：正序/倒序各自的 sim_ts 单调方向，returned==min(limit, matched)。
  E 全量导出：CSV 数据行数==内存条数，并逐行核对关键列与 JSONL 一致（用 csv.DictReader
     + utf-8-sig 读回，因为 Excel 友好就带 BOM，直接 split(',') 会把首列名读成 '\\ufeffseq'）。
  F 增量导出：游标推进后再 only_new 应为 0 行，且追加写不得在文件中间插第二行表头
     （插了的话 DictReader 会多出一条全空的假记录，条目 5 的"导出可复核"就假了）。
     口径：游标由**任何**导出推进（不分增量/全量），所以判据按"刚轮转出来的新文件里追加了
     几行"来核，不假设"之前为空"；--wait-new 先等账本长出至少一条再导，让"只追加不重写"
     被真演一遍，而不是两边都 0 混过去。
  G prefix 带目录分隔符必须被拒（这个 srv 的调用方是看板和 CLI，接受路径等于任意目录写文件）。
  H 动作账：一次到位配一次下发的闭合，但"到位"有三条来源（通行闭合／手动到位／收尾补记），
     只有第一类配下发指令，另两类的计数分别去对手动生效与故障清除；不加这一层分类，
     故障演出之后的收尾那一拍就会被读成"落杆没人下发就自己掉下来了"。

用法（WSL，需在环：gzserver + parking_recorder 起着）：
  source /opt/ros/humble/setup.bash && source ~/v2x_ws/install/setup.bash
  python3 tools/pk_record_audit.py [--config src/v2x_demo/config/parking_zones.yaml]
                                    [--prefix audit] [--skip-live]
  --skip-live 只离线核对已导出的 CSV 与 JSONL（没有 ROS 环境时用，判据会少 C/D/F/G）。
退出码 0=PASS 1=FAIL。
"""
import argparse
import csv
import json
import os
import sys
import time

CHECKS = []
FAILS = []


def ck(cond, msg):
    CHECKS.append((bool(cond), msg))
    print('  %s %s' % ('✓' if cond else '✗', msg))
    if not cond:
        FAILS.append(msg)


def bail():
    """提前退出也要把 FAIL 文本列出来：取证时只看退出码不知道挂在哪儿，而自校脚本
    正是靠'  FAIL: '这一行来判断某条判据到底有没有咬住。"""
    print('\n汇总：FAIL %d 条' % len(FAILS))
    for f in FAILS:
        print('  FAIL: ' + f)
    return 1


def read_jsonl(path):
    rows = []
    try:
        with open(path, encoding='utf-8') as f:
            for ln in f:
                ln = ln.strip()
                if ln:
                    rows.append(json.loads(ln))
    except OSError as e:
        print('  （读不到 %s：%s）' % (path, e))
    return rows


def csv_rows(path):
    """utf-8-sig 读回：BOM 不脱掉的话首列名是 '\\ufeffseq'，逐列比对会全线错配。
    path 为空（服务超时没回包、拿不到 CSV 路径）时按"没有行"处理，让判据去 FAIL，
    而不是把整轮审计炸成 traceback —— 退出码 1 之外还要留下 '  FAIL: ' 那几行。"""
    out = []
    if not path:
        return out
    try:
        with open(path, encoding='utf-8-sig', newline='') as f:
            for r in csv.DictReader(f):
                out.append(r)
    except OSError:
        pass
    return out


def csv_header_count(path, tail):
    """数表头行：表头是唯一以 tail（pass=cycle_s / act=note）结尾的行。"""
    if not path:
        return -1
    try:
        with open(path, encoding='utf-8-sig', newline='') as f:
            return sum(1 for ln in f if ln.rstrip('\r\n').endswith(tail))
    except OSError:
        return -1


def num(a, b, tol=2e-3):
    try:
        return abs(float(a) - float(b)) <= tol
    except (TypeError, ValueError):
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', default='')
    ap.add_argument('--prefix', default='audit')
    ap.add_argument('--dir', default='', help='覆盖 yaml 的 records.dir（自测用临时目录）')
    ap.add_argument('--wait-new', type=float, default=0.0,
                    help='F 段先等账本长出至少 1 条新通行再做增量导出（在环取证建议 220）')
    ap.add_argument('--skip-live', action='store_true')
    args = ap.parse_args()

    cfg_path = args.config
    if not cfg_path:
        try:
            from ament_index_python.packages import get_package_share_directory
            cfg_path = os.path.join(get_package_share_directory('v2x_demo'),
                                    'config', 'parking_zones.yaml')
        except Exception:
            cfg_path = 'src/v2x_demo/config/parking_zones.yaml'
    import yaml
    with open(cfg_path, encoding='utf-8') as fh:
        pk = yaml.safe_load(fh)['parking']
    rec = pk['records']
    # --dir 覆盖 yaml 的 records.dir：自测要把账本关进临时目录；在环取证时不传，
    # 就按配置走 /tmp/v2x_parking，与记录节点写的那一份是同一个文件。
    rdir = args.dir or rec['dir']
    p_jsonl = os.path.join(rdir, rec['pass_file'])
    a_jsonl = os.path.join(rdir, rec['action_file'])
    keep = int(rec.get('max_records_in_memory', 5000))
    print('对账开始 %s｜账本 %s' % (time.strftime('%H:%M:%S'), rdir))
    print('  配置 %s｜内存上限 %d 条' % (cfg_path, keep))

    from v2x_msgs.msg import ParkingPass
    from v2x_msgs.srv import ParkingExport, ParkingQuery

    passes = read_jsonl(p_jsonl)
    acts = read_jsonl(a_jsonl)

    print('=== A) 账本可读、而且是本轮的 ===')
    ck(len(passes) > 0, '通行账本 %s 行（必须 >0，空账本说明没有真过杆，条目 5 无从对账）'
       % len(passes))
    ck(len(acts) > 0, '动作账本 %s 行' % len(acts))
    # 新鲜度：记录节点开机时会把上一轮的 JSONL 改名成 <名>.<月日_时分秒> 再建空文件，
    # 所以文件 mtime 落在几分钟内就说明"本轮真有过杆"。少了这条，拿着昨天的账本
    # 也能把 C/D/E/F 全跑绿 —— 那是对账器最坏的一种通过。
    age = time.time() - os.path.getmtime(p_jsonl) if os.path.exists(p_jsonl) else -1
    ck(0 <= age < 600, '通行账本 %s 秒前还在写（>600 s 就是上一轮的遗留账本，不能当本轮证据）'
       % ('%.0f' % age if age >= 0 else '读不到'))
    if not passes:
        print('!! 无通行记录，终止')
        return bail()

    # ── 本地按文档口径重算，作为一切"节点说的数"的对照 ────────────────────
    # 参数名一律用 ParkingQuery 的字段名，因为 C 段是 local(**kw) 直接把请求字典摊进来
    # 重算的；这里改成 plate/vid/gate 这种简称，第一次跑就会 TypeError 而不是判错。
    def local(plate_number='', vehicle_id='', gate_id='', direction=0, result=0,
              t0=0.0, t1=0.0):
        want_dir = {ParkingQuery.Request.DIR_IN: ParkingPass.DIR_IN,
                    ParkingQuery.Request.DIR_OUT: ParkingPass.DIR_OUT}.get(direction)
        want_res = {ParkingQuery.Request.RESULT_ALLOWED: ParkingPass.RESULT_ALLOWED,
                    ParkingQuery.Request.RESULT_DENIED: ParkingPass.RESULT_DENIED,
                    ParkingQuery.Request.RESULT_PENDING: ParkingPass.RESULT_PENDING}.get(result)
        n = 0
        for r in passes:
            if plate_number and plate_number not in (r['plate_number'] or ''):
                continue
            if vehicle_id and vehicle_id != r['vehicle_id']:
                continue
            if gate_id and gate_id != r['gate_id']:
                continue
            if want_dir is not None and r['direction'] != want_dir:
                continue
            if want_res is not None and r['result'] != want_res:
                continue
            if t0 > 0 and r['sim_ts'] < t0:
                continue
            if t1 > 0 and r['sim_ts'] > t1:
                continue
            n += 1
        return n

    if args.skip_live:
        print('=== 跳过在环项（--skip-live）===')
        live = None
    else:
        import rclpy
        from rclpy.node import Node

        class Audit(Node):
            def __init__(self):
                super().__init__('pk_record_audit')
                self.q = self.create_client(ParkingQuery, '/parking/query')
                self.e = self.create_client(ParkingExport, '/parking/export')

        rclpy.init()
        live = Audit()
        okq = live.q.wait_for_service(timeout_sec=15.0)
        oke = live.e.wait_for_service(timeout_sec=15.0)
        ck(okq and oke, '两个服务 /parking/query %s、/parking/export %s 可达'
           % ('✓' if okq else '✗', '✓' if oke else '✗'))
        if not (okq and oke):
            print('!! 服务不可达，终止在环判据')
            return bail()

        def call(client, req, timeout=30.0):
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(live, fut, timeout_sec=timeout)
            if not fut.done():
                return None
            return fut.result()

        def query(**kw):
            req = ParkingQuery.Request()
            for k, v in kw.items():
                setattr(req, k, v)
            r = call(live.q, req)
            if r is None:
                print('    （查询 %s 超时）' % kw)
            return r

        print('=== B) 全量查询的三口径 ===')
        r = query(limit=0)
        n_js, n_mem = len(passes), r.stored
        ck(r.success, '回包 success=True')
        print('    JSONL %d 行 / 内存 stored=%d / matched=%d / returned=%d'
              % (n_js, r.stored, r.matched, r.returned))
        overflow = n_js > n_mem
        if overflow:
            print('    （环形溢出：%d>%d，本次启动以来丢最旧 %d 条，CSV 少于 JSONL 才是对的）'
                  % (n_js, n_mem, n_js - n_mem))
        ck(r.matched == r.stored, 'matched(%d)==stored(%d)：命中数不受 limit 影响，等于内存保留数'
           % (r.matched, r.stored))
        ck(r.returned == r.stored, 'limit=0 时 returned(%d)==stored(%d)' % (r.returned, r.stored))
        ck((r.matched == n_js) == (not overflow),
           '未溢出时 matched==JSONL 行数；溢出时必须承认不等（当前 overflow=%s）' % overflow)

        print('=== C) 过滤矩阵（节点命中数 vs 本地按文档口径重算）===')
        first = passes[0]
        plate_all = [p['plate_number'] for p in passes if p['plate_number']]
        specs = [
            ('方向=进场', dict(direction=ParkingQuery.Request.DIR_IN)),
            ('方向=出场', dict(direction=ParkingQuery.Request.DIR_OUT)),
            ('方向=不限', dict(direction=ParkingQuery.Request.DIR_ANY)),
            ('结果=放行', dict(result=ParkingQuery.Request.RESULT_ALLOWED)),
            ('结果=拒绝', dict(result=ParkingQuery.Request.RESULT_DENIED)),
            ('结果=挂起', dict(result=ParkingQuery.Request.RESULT_PENDING)),
            ('闸=gate_in', dict(gate_id='gate_in')),
            ('闸=gate_out', dict(gate_id='gate_out')),
            ('车辆=%s' % first['vehicle_id'], dict(vehicle_id=first['vehicle_id'])),
        ]
        if plate_all:
            pl = plate_all[0]
            specs += [('车牌全串=%s' % pl, dict(plate_number=pl)),
                      ('车牌后5位=%s' % pl[-5:], dict(plate_number=pl[-5:])),
                      ('车牌乱串=ZZZ9', dict(plate_number='ZZZ9'))]
        ts = sorted(p['sim_ts'] for p in passes)
        mid = ts[len(ts) // 2]
        specs += [('时段[t0=中位]', dict(t0=mid)), ('时段[t1=中位]', dict(t1=mid))]
        if plate_all:
            specs += [('组合 闸in+放行+牌', dict(gate_id='gate_in',
                                                result=ParkingQuery.Request.RESULT_ALLOWED,
                                                plate_number=plate_all[0]))]
        for name, kw in specs:
            r = query(limit=0, **kw)
            if r is None:
                ck(False, '%s：查询超时' % name)
                continue
            exp = local(**kw)
            got = r.matched
            # 命中数还必须等于返回行数（limit=0），且每一条都被同一过滤真正满足
            bad = 0
            want_dir = {ParkingQuery.Request.DIR_IN: ParkingPass.DIR_IN,
                        ParkingQuery.Request.DIR_OUT: ParkingPass.DIR_OUT}.get(kw.get('direction', 0))
            want_res = {ParkingQuery.Request.RESULT_ALLOWED: ParkingPass.RESULT_ALLOWED,
                        ParkingQuery.Request.RESULT_DENIED: ParkingPass.RESULT_DENIED,
                        ParkingQuery.Request.RESULT_PENDING: ParkingPass.RESULT_PENDING}.get(kw.get('result', 0))
            for m in r.passes:
                if want_dir is not None and m.direction != want_dir:
                    bad += 1
                if want_res is not None and m.result != want_res:
                    bad += 1
                if kw.get('gate_id') and m.gate_id != kw['gate_id']:
                    bad += 1
                if kw.get('plate_number') and kw['plate_number'] not in (m.plate_number or ''):
                    bad += 1
            ck(got == exp and r.returned == got and bad == 0,
               '%s：节点 matched=%d 本地重算=%d 返回=%d 违例=%d' % (name, got, exp, r.returned, bad))

        print('=== D) 排序与 limit ===')
        for nf, tag in ((False, '正序'), (True, '倒序')):
            r = query(limit=2, newest_first=nf)
            got = [m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for m in r.passes]
            # on_query 里是 hits.sort(key=sim_ts, reverse=newest_first)：
            # 倒序 = 后一条不大于前一条，正序 = 后一条不小于前一条。写反了这两条
            # 判据会在每一轮真实数据上都 FAIL，而看起来像"记录器排序坏了"。
            mono = all(b <= a for a, b in zip(got, got[1:])) if nf else \
                all(b >= a for a, b in zip(got, got[1:]))
            ck(r.returned == min(2, r.matched) and mono,
               '%s limit=2：returned=%d（应 min(2,%d)），sim_ts 单调方向 %s'
               % (tag, r.returned, r.matched, '对' if mono else '反'))
        r = query(limit=1, newest_first=True)
        newest = max(p['sim_ts'] for p in passes)
        ck(num(r.passes[0].header.stamp.sec + r.passes[0].header.stamp.nanosec * 1e-9, newest, 1.05),
           '倒序第一条就是最新一次通行（%.2f vs JSONL 最大 %.2f，容差 1.05 s：'
           'sim_ts 是通行成立那一刻，ts 是记录器收到那一刻，中间就是摆渡车过杆那几秒）'
           % (r.passes[0].header.stamp.sec + r.passes[0].header.stamp.nanosec * 1e-9, newest))

        print('=== E/F/G) 导出 ===')
        def export(**kw):
            req = ParkingExport.Request()
            for k, v in kw.items():
                setattr(req, k, v)
            return call(live.e, req, timeout=60.0)

        # G 先做：拒绝带目录分隔符的 prefix，且不能写出任何文件
        for badp in ('../evil', 'sub/dir', 'a\\b'):
            r = export(prefix=badp)
            ck(r is not None and not r.success and 'prefix' in (r.message or ''),
               'prefix=%r 被拒（success=%s）：%s'
               % (badp, None if r is None else r.success, '' if r is None else r.message[:60]))

        # F 增量。口径（2026-10-14 修过两轮）：
        #  1) 记录器的游标是"任何一次导出"都推进的（on_export 末尾无条件 exp_pass=pass_seen，
        #     不分增量/全量），所以第 1 次 only_new 回多少行，取决于**上一次导出发生在什么时候**，
        #     跟"等待期新增几条"没有必然关系。第一版拿条数去比条数，只要取证脚本的 5e 全量导出
        #     与这里的快照之间又落一条，就假挂一条 FAIL。
        #  2) prefix 的 CSV 是追加式的，上一轮审计留下的同名文件里那些行不属于本轮。
        # 处置：先把审计自己的那两份 prefix 产物轮转走，再**立刻做一次全量导出把游标钉死**，
        #     记下此刻文件里的 pass_id 集合；之后"追加了哪些行"按 pass_id 逐个点名核，
        #     不按条数猜。--wait-new 保证等待期确实落了账，"只追加、不重写整表"是被演出来的。
        for f in (os.path.join(rdir, args.prefix + '_parking_pass.csv'),
                  os.path.join(rdir, args.prefix + '_barrier_actions.csv')):
            if os.path.exists(f) and os.path.getsize(f) > 0:
                gone = '%s.pre_%s' % (f, time.strftime('%m%d_%H%M%S'))
                os.rename(f, gone)
                print('    审计先轮转自己的旧产物 -> %s（%d 字节）'
                      % (os.path.basename(gone), os.path.getsize(gone)))
        rb = export(prefix=args.prefix)
        p1 = None if rb is None else rb.pass_csv
        ids_before = set(x['pass_id'] for x in csv_rows(p1)) if p1 else set()
        ck(rb is not None and rb.success and len(ids_before) == rb.pass_rows,
           '钉游标：轮转后全量导出 %d 行，新文件里 pass_id 去重 %d 个（不等就是重写或漏行）'
           % (-1 if rb is None else rb.pass_rows, len(ids_before)))
        added_ids = []
        if args.wait_new > 0:
            n0 = len(ids_before)
            waited = 0.0
            while waited < args.wait_new:
                cur = read_jsonl(p_jsonl)
                added_ids = [x['pass_id'] for x in cur if x['pass_id'] not in ids_before]
                if added_ids:
                    print('    等了 %.0f s：通行账本 %d -> %d 条，等待期新落 %s，对照表随之刷新'
                          % (waited, n0, len(cur), '、'.join(added_ids)))
                    passes[:] = cur
                    acts[:] = read_jsonl(a_jsonl)
                    break
                time.sleep(4)
                waited += 4
            if not added_ids:
                print('    !! 等了 %.0f s 账本没长（--wait-new 落空，下面按"追加 0 行"口径核）'
                      % args.wait_new)

        r1 = export(prefix=args.prefix, only_new=True)
        ck(r1.success, '第 1 次 only_new：回包 通行 %d 行 / 动作 %d 行'
                          '（游标已由上面那次钉位导出推进，这里回的就是等待期那几条）'
           % (r1.pass_rows, r1.action_rows))
        rows_after = csv_rows(p1)
        appended = [x['pass_id'] for x in rows_after if x['pass_id'] not in ids_before]
        ck(len(rows_after) == len(ids_before) + r1.pass_rows,
           '增量是追加不是重写：文件从 %d 行变 %d 行，多出的正好等于回包 pass_rows=%d'
           % (len(ids_before), len(rows_after), r1.pass_rows))
        if args.wait_new > 0:
            ck(len(added_ids) >= 1 and sorted(appended) == sorted(added_ids),
               '等待期新落 %d 条（%s），增量追加的 %d 行按 pass_id 逐个对上（%s）'
               % (len(added_ids), '、'.join(added_ids) or '无',
                  len(appended), '、'.join(appended) or '无'))
        else:
            # 没开 --wait-new 时不要求"必须等到新行"，但身份核对照旧：钉位导出之后
            # 又落进文件的那些行，必须正好是钉位与这次增量之间新来的（通常就是 0 行）。
            ck(sorted(appended) == sorted(added_ids),
               '未开 --wait-new：钉位导出后文件里新出现的 %d 行（回包 pass_rows=%d），'
               '身份核对按"追加 0 行"口径' % (len(appended), r1.pass_rows))
        r2 = export(prefix=args.prefix, only_new=True)
        ck(r2.pass_rows == 0 and r2.action_rows == 0,
           '第 2 次 only_new：0 行（游标已推进，实际回包 %d/%d）' % (r2.pass_rows, r2.action_rows))
        ck(csv_header_count(p1, 'cycle_s') == 1,
           '追加写没在文件中间插第二行表头（表头计数=%d，必须 1）'
           % csv_header_count(p1, 'cycle_s'))
        n2 = len(csv_rows(p1))
        ck(n2 == len(rows_after), '第 2 次增量没写空行也没重写：CSV 仍 %d 行（读回 %d）'
                                   % (len(rows_after), n2))

        # E 全量导出到正式名（无 prefix），并逐行核对
        r3 = export(prefix='')
        ck(r3.success, '全量导出：回包 pass_rows=%d action_rows=%d' % (r3.pass_rows, r3.action_rows))
        # 对照表在导出之后再读一次：账本只会追加，所以"CSV 的行数 <= JSONL 的行数"恒成立，
        # 反过来若按导出之前的快照比，等待期间新落的那一条会让下面这行差 1 而假挂。
        passes[:] = read_jsonl(p_jsonl)
        acts[:] = read_jsonl(a_jsonl)
        p_csv, a_csv = r3.pass_csv, r3.action_csv
        crow = csv_rows(p_csv)
        # CSV 是"当前内存环形表"的投影，JSONL 是"本轮全部"的账。溢出时 CSV 只有最新
        # keep 条，所以对照一律按 CSV 的条数从 JSONL 尾部取 —— 拿整个 JSONL 去 zip
        # 会让 seq 从第二条起全部错位，看上去像导出坏了。
        ck(len(crow) == min(len(passes), keep),
           'CSV 数据行 %d == min(JSONL %d, 内存上限 %d)' % (len(crow), len(passes), keep))
        cols = ['seq', 'pass_id', 'direction', 'gate_id', 'plate_number', 'recognized',
                'plate_confidence', 'vehicle_id', 'kind', 'result', 'ticket_id',
                'whitelist_hit', 'arrive_s', 'raise_s', 'clear_s', 'lower_s',
                'raise_dur_s', 'pass_dur_s', 'lower_dur_s', 'cycle_s', 'sim_ts']
        arith = ['plate_confidence', 'arrive_s', 'raise_s', 'clear_s', 'lower_s',
                 'raise_dur_s', 'pass_dur_s', 'lower_dur_s', 'cycle_s', 'sim_ts', 'seq',
                 'direction', 'kind', 'result', 'whitelist_hit']
        bad = []
        tail = passes[-len(crow):] if crow else []
        for cr, jr in zip(crow, tail):
            for c in cols:
                jv, cv = jr.get(c), cr.get(c)
                if c in arith:
                    if not num(cv, jv):
                        bad.append('%s.%s csv=%r jsonl=%r' % (jr['pass_id'], c, cv, jv))
                else:
                    if str(cv) != str(jv):
                        bad.append('%s.%s csv=%r jsonl=%r' % (jr['pass_id'], c, cv, jv))
        ck(not bad, '逐列核对 %d 行 × %d 列：违例 %d 处 %s'
           % (len(tail), len(cols), len(bad), ('｜' + '; '.join(bad[:4])) if bad else ''))
        # 相减列必须真是那两个时刻之差（导出侧算的，不是原样搬运，最容易出错的一组）
        bad = []
        for cr in tail:
            # 这里必须走 JSONL 那侧（数值），CSV 侧全是字符串，'1.35'-'0.75' 会直接 TypeError。
            if num(cr['arrive_s'], -1) or num(cr['lower_s'], -1):
                if not num(cr['cycle_s'], -1):
                    bad.append('%s cycle_s=%s 但时刻含 -1' % (cr['pass_id'], cr['cycle_s']))
            elif not num(cr['cycle_s'], float(cr['lower_s']) - float(cr['arrive_s'])):
                bad.append('%s cycle_s=%s != lower-arrive' % (cr['pass_id'], cr['cycle_s']))
        ck(not bad, 'cycle_s 恒等于 lower_s-arrive_s（含 -1 时也必须 -1）：%s'
           % ('; '.join(bad[:3]) if bad else '闭合'))

        r4 = export(prefix=args.prefix)
        # 断言按 args.prefix 取，不写死 'audit_' —— 2026-10-16 拿 --prefix calib191 校准这一版
        # 对账器时，57 项全过、只剩这一项假挂：写死的字符串与它自己刚传出去的前缀不是一回事，
        # 换个前缀跑就把"导出正常"读成 FAIL。判据也要把两半分开写：endswith('parking_pass.csv')
        # 只证明名字像，证明不了目录没被前缀带走（prefix 里塞路径就是任意目录写文件）。
        want = (args.prefix + '_' if args.prefix else '') + 'parking_pass.csv'
        ck(os.path.basename(r4.pass_csv) == want
           and os.path.dirname(r4.pass_csv) == rdir,
           'prefix 只改文件名前缀（→%s）、不改目录（仍 %s）：%s' % (want, rdir, r4.pass_csv))

    print('=== H) 动作账（离线口径）===')
    from collections import Counter
    cn = Counter(a['action_name'] for a in acts)
    print('    动作分布：' + '、'.join('%s=%d' % (k, v) for k, v in sorted(cn.items())))
    # "一次到位配一次下发"是闭合的总原则，但到位那行日志有三条来源，只有第一类才配下发指令
    # （都写在 parking_manager_node.on_state 的分支注释里）：
    #   通行闭合  —— 在途通行归零时的 on_action_done，配一条 下发抬杆 / 下发落杆
    #   手动到位  —— 手动占权期间目标角到位（note 以"手动目标角到位"开头），配的是 手动生效
    #   收尾补记  —— 没有对应在途通行却落到位（note 含"无在途"），是故障清除/交还自动后
    #               补的那一拍，天生没有下发指令；不补这一行，动作账就成"抬了没落"
    def lineage(a):
        note = a.get('note') or ''
        if '无在途' in note:
            return '收尾补记'
        if note.startswith('手动目标角到位'):
            return '手动到位'
        return '通行闭合'

    per = {}
    for a in acts:
        per.setdefault(a['gate_id'], Counter())[a['action_name']] += 1
    lin = {}
    for a in acts:
        nm = a['action_name']
        if nm in ('抬杆到位', '落杆到位'):
            lin.setdefault(a['gate_id'], Counter())[(nm, lineage(a))] += 1
    for g, c in sorted(per.items()):
        L = lin.get(g, Counter())
        print('    %s：抬杆到位 %d（闭合 %d／手动 %d）/ 下发抬杆 %d　'
              '落杆到位 %d（闭合 %d／手动 %d／收尾补记 %d）/ 下发落杆 %d　'
              '手动生效 %d／故障清除 %d／防砸重抬 %d／拒绝动作 %d'
              % (g, c.get('抬杆到位', 0), L.get(('抬杆到位', '通行闭合'), 0),
                 L.get(('抬杆到位', '手动到位'), 0), c.get('下发抬杆', 0),
                 c.get('落杆到位', 0), L.get(('落杆到位', '通行闭合'), 0),
                 L.get(('落杆到位', '手动到位'), 0), L.get(('落杆到位', '收尾补记'), 0),
                 c.get('下发落杆', 0), c.get('手动生效', 0), c.get('故障清除', 0),
                 c.get('防砸重抬', 0), c.get('拒绝动作', 0)))
    for g, c in sorted(per.items()):
        L = lin.get(g, Counter())
        cyc_up = L.get(('抬杆到位', '通行闭合'), 0)
        cyc_dn = L.get(('落杆到位', '通行闭合'), 0)
        man_up = L.get(('抬杆到位', '手动到位'), 0)
        man_dn = L.get(('落杆到位', '手动到位'), 0)
        tail_dn = L.get(('落杆到位', '收尾补记'), 0)
        ck(cyc_up <= c.get('下发抬杆', 0),
           '%s 通行闭合的抬杆到位 %d <= 下发抬杆 %d' % (g, cyc_up, c.get('下发抬杆', 0)))
        ck(cyc_dn <= c.get('下发落杆', 0),
           '%s 通行闭合的落杆到位 %d <= 下发落杆 %d' % (g, cyc_dn, c.get('下发落杆', 0)))
        # 下发了却没到位的那些，必须能被"防砸挡回/核验拒绝"解释掉，不能是日志漏记。
        ck(0 <= c.get('下发落杆', 0) - cyc_dn <= c.get('防砸重抬', 0) + c.get('拒绝动作', 0),
           '%s 下发落杆 %d 比通行闭合 %d 多的 %d 发没到位，落在防砸重抬 %d／拒绝动作 %d 里'
           % (g, c.get('下发落杆', 0), cyc_dn, c.get('下发落杆', 0) - cyc_dn,
              c.get('防砸重抬', 0), c.get('拒绝动作', 0)))
        ck(c.get('抬杆到位', 0) == cyc_up + man_up
           and c.get('落杆到位', 0) == cyc_dn + man_dn + tail_dn,
           '%s 到位行按三条来源分类后不多不少（抬 %d=%d+%d，落 %d=%d+%d+%d）'
           % (g, c.get('抬杆到位', 0), cyc_up, man_up,
              c.get('落杆到位', 0), cyc_dn, man_dn, tail_dn))
        ck(man_up + man_dn <= c.get('手动生效', 0),
           '%s 手动到位 %d 次都有对应的手动下发（手动生效 %d 条，含撤销占权那一发）'
           % (g, man_up + man_dn, c.get('手动生效', 0)))
        ck(tail_dn <= c.get('故障清除', 0) + c.get('手动生效', 0),
           '%s 收尾补记的落杆 %d 条 <= 故障清除 %d + 手动生效 %d（这一拍只可能由那两类引出）'
           % (g, tail_dn, c.get('故障清除', 0), c.get('手动生效', 0)))
    seqs = [a['seq'] for a in acts]
    ck(seqs == list(range(1, len(seqs) + 1)), '动作 seq 连续无洞（1..%d）' % len(seqs))
    pseqs = [p['seq'] for p in passes]
    ck(pseqs == list(range(1, len(pseqs) + 1)), '通行 seq 连续无洞（1..%d）' % len(pseqs))
    ids = [p['pass_id'] + '|' + p['gate_id'] for p in passes]
    dup = sorted({k for k in ids if ids.count(k) > 1})
    ck(len(ids) == len(set(ids)),
       'pass_id+gate 唯一（编号是管理器进程内计数，账本却是追加式的；管理器中途重启必须由 '
       'resume_pass_seq 续号，否则撞号 %s）' % (dup[:4] if dup else '无'))

    if not args.skip_live:
        acrow = csv_rows(r3.action_csv)
        # 同一口径：动作 CSV 也是内存投影，溢出时只应有最新 keep 条。
        ck(len(acrow) == min(len(acts), keep),
           '动作 CSV 行 %d == min(JSONL %d, 内存上限 %d)' % (len(acrow), len(acts), keep))
        ck(csv_header_count(r3.action_csv, 'note') == 1, '动作 CSV 表头 1 行')

    print('\n=== 汇总 ===')
    print('判定 %d 条，通过 %d，失败 %d'
          % (len(CHECKS), len(CHECKS) - len(FAILS), len(FAILS)))
    for f in FAILS:
        print('  FAIL: ' + f)
    return 0 if not FAILS else 1


if __name__ == '__main__':
    sys.exit(main())
