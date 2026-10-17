#!/usr/bin/env python3
# 离线核对"进角限速"在换段前后是否连续。
# 口径要点名清楚，这条脚本只证机制方向，不证运行期真的走了这条路：
#   1) 世代闸：直接读源码，确认限速用的是"到最近角点顶点的欧氏距离"，而不是旧的
#      leg_len - along。改没改进去，用字节判，不靠记忆。
#   2) 机制闸：拿 yaml 里四条真实折线，按 0.05 m 步长沿弧长走一遍，分别用新旧两个
#      度量算 limit=min(cruise, max(0.12, max_ang*d))，断言在"角点前 2.2 m 到角点后
#      2.2 m"整段窗口里新度量始终 <2.2（限速一直生效），而旧度量在换段那一帧
#      （角点前 turn_lead=1.2 m）就跳到 leg_len+lead 从而失效。
# 用法：python3 tools/pk219_cornerlimit320.py <config yaml>
import math
import sys

import yaml

CFG = sys.argv[1]
STEP = 0.05
MAX_ANG = 0.45
CRUISE = 0.70
LEAD = 1.2
WIN = 2.2
SRC = 'src/v2x_demo/v2x_demo/signal_drive_node.py'

# ---- 1) 世代闸 ----
txt = open('/mnt/d/desktop1/9.26.1/v2x_traffic_sim/' + SRC,
           encoding='utf-8').read()
new_form = ('to_corner = min(math.hypot(ag.x - tx, ag.y - ty),' in txt
            and 'cx, cy = ag.route[(ag.idx - 1) % len(ag.route)]' in txt
            and 'self.max_ang * la / (2.0 * math.sin(ea))' in txt)
old_form = 'to_corner = leg_len - along' in txt
print('  世代闸: 新度量在位=%s 旧度量残留=%s' % (new_form, old_form))
if not new_form or old_form:
    print('PK219FAIL generation')
    sys.exit(4)

# ---- 2) 机制闸 ----
c = yaml.safe_load(open(CFG, encoding='utf-8'))
cars = c['cars'] if 'cars' in c else c['signal_drive']['cars']
print('  路线数=%d：%s' % (len(cars), ','.join(cars)))


def limit_for(d):
    return CRUISE if d >= WIN else min(CRUISE, max(0.12, MAX_ANG * max(d, 0.25)))


bad = []
for name, cfg in cars.items():
    route = [tuple(p) for p in cfg.get('route', [])]
    if len(route) < 3:
        continue
    # 车在角点 k 前后的窗口采样点：角点前 s_pre 用旧段度量，角点后 s_post 用新段度量
    worst_new = 0.0
    worst_old_off = 0.0
    for k in range(len(route)):
        p = route[k]                       # 正在绕的角
        prev = route[(k - 1) % len(route)]
        nxt = route[(k + 1) % len(route)]
        l_prev = math.hypot(p[0] - prev[0], p[1] - prev[1])
        l_next = math.hypot(nxt[0] - p[0], nxt[1] - p[1])
        ux, uy = (p[0] - prev[0]) / l_prev, (p[1] - prev[1]) / l_prev
        for off in [x * STEP for x in range(int(WIN / STEP))]:   # 不含 2.2 那一端：
            # 代码判据是 to_corner < 2.2，正好落在等式上就不该限速，把边界点算成反例
            # 是判据脚本自己的错（判据脚本先用已通过数据校准）。
            # 角点前 off：车在旧段上，距离角点 off
            x, y = p[0] - ux * off, p[1] - uy * off
            # 新度量 = min(到下一个角 route[idx]=p, 到本段起点 route[idx-1]=prev)
            d_new = min(math.hypot(x - p[0], y - p[1]),
                        math.hypot(x - prev[0], y - prev[1]))
            # 旧度量 = 旧段剩余里程 = off
            if not (d_new < WIN):
                bad.append(f'{name} k{k} 角前{off:.2f} 新度量{d_new:.2f}>= {WIN}')
            # 角点后 off：换段已发生，车在新段上，along = off - lead
            x2, y2 = p[0] + (nxt[0] - p[0]) / l_next * off, \
                     p[1] + (nxt[1] - p[1]) / l_next * off
            along2 = off - LEAD
            d_new2 = min(math.hypot(x2 - nxt[0], y2 - nxt[1]),
                         math.hypot(x2 - p[0], y2 - p[1]))
            d_old2 = l_next - along2                      # 旧公式：leg_len - along
            if off <= LEAD:                               # 还在转向窗口内
                if not (d_new2 < WIN):
                    bad.append(f'{name} k{k} 角后{off:.2f} 新度量{d_new2:.2f}>= {WIN}')
                if d_old2 < WIN:
                    bad.append(f'{name} k{k} 角后{off:.2f} 旧度量竟仍生效{d_old2:.2f}')
                worst_old_off = max(worst_old_off, d_old2 - WIN)
            worst_new = max(worst_new, max(d_new, d_new2))
        # 换段那一帧（角点前 lead）旧度量立刻失效，这是病灶本体，必须复现出来
        after = l_next + LEAD
        if not (after >= WIN):
            bad.append(f'{name} k{k} 换段后旧度量{after:.2f}本该失效却没失效')
        switch_off = after
    print('  %-8s 转向窗口内新度量最大 %.2f m (<%.1f 一直生效) / 旧度量换段那一帧跳到 %.2f m'
          % (name, worst_new, WIN, switch_off))

if bad:
    print('  ## 反例 %d 条' % len(bad))
    for b in bad[:12]:
        print('    ' + b)
    print('PK219FAIL mechanism')
    sys.exit(5)

print('  限速曲线（新度量）: 角点 2.20/1.50/1.20/0.60/0.25 m -> %.2f/%.2f/%.2f/%.2f/%.2f m/s'
      % (limit_for(2.3), limit_for(1.5), limit_for(1.2), limit_for(0.6), limit_for(0.25)))
print('  对比 旧度量换段后: d=leg_len+1.2>=2.2 -> 直接放回 cruise %.2f m/s（这就是冲角的原因）'
      % CRUISE)

# ---- 3) 曲率可行性闸：换段那一帧到底要多慢 ----
# 换段发生在角点前 LEAD=1.2 m，此时 along=-1.2，前视点 la = along + max(2.5, 3*cruise)=1.3 m
# 在新段上；车到该前视点的视线与车头方向夹角 err=atan(1.3/1.2)。纯追踪要的曲率
# kappa=2*sin(err)/la，可实现的上限是 omega/v，解出 v<=omega*la/(2 sin err)。
la = -LEAD + max(2.5, 3 * CRUISE)
err = math.atan2(la, LEAD)
cap_d = max(0.12, MAX_ANG * LEAD)
cap_k = max(0.12, MAX_ANG * la / (2.0 * math.sin(err)))
r_need = la / (2.0 * math.sin(err))
print('  换段帧: err=%.0f° 前视 la=%.2f m 要的半径 r=%.2f m（ω·d 版只压到 %.2f m/s，'
      '曲率版压到 %.2f m/s）' % (math.degrees(err), la, r_need, cap_d, cap_k))
print('  对照: cruise %.2f 时轮子能实现的半径只有 v/ω=%.2f m，粗了 %.2f 倍 -> 必然甩外切'
      % (CRUISE, CRUISE / MAX_ANG, (CRUISE / MAX_ANG) / r_need))
if not (0.12 < cap_k < cap_d <= CRUISE):
    print('PK219FAIL curvature 曲率闸没有比 ω·d 更紧（cap_k=%.3f cap_d=%.3f）' % (cap_k, cap_d))
    sys.exit(6)
if 'and not ag.stopped and ag.vmag < 0.05' not in txt:
    print('PK219FAIL curvature 顶住死区的第二条判据不在源码里')
    sys.exit(6)
print('PK219PASS')
