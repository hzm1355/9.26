# Blender 无头勘查脚本 #199：把 车模.fbx 分成"整车簇"与"散件"，量出真实比例并试算缩放。
#
# 为什么要这一趟：#198 的表显示这份 FBX 不是"一台干净的轿车"，而是一个 **CAD 装配导出**
# （零件名是 3ds Max 的 GBK 乱码：删除-拉伸/移动面/转轴合页/前轮固定件，还有
# 主板+转接板.stp、语音模块 M260C 环形六轴.stp），总量包围盒 X 4.161 × Y 3.887 × Z 0.670
# 里有大半是**摆在远处的散件**（ÒÆ¶¯Ãæ62 在 y=-7.66、ÇÐ³ý-À­Éì22 在 x=2.83）。
# 总量不能用来定 scale —— 得先把"整车那一簇"圈出来，否则缩放会把散件一起拉进来，
# 画面就是一辆车旁边飘着几块板。
#
# 量出来要回答的三个数（#127 的分叉就卡在这里）：
#   1) 整车簇的 L/W/H 比例 —— 它决定"等比放进 1.60×0.80 的碰撞包络"是否可能；
#   2) 面数 —— 六台车 × N，N 太大 gzclient 掉帧，取证画面直接糊掉；
#   3) 那四个 default04x 是不是轮子（是的话可以拆成独立轮网格挂到 wheel link，
#      让轮子跟着转；否则网格轮子永远不转，看着像滑行）。
import os

import bpy
import mathutils

FBX = r'/mnt/d/desktop1/9.26.1/车模.fbx'
OUT = r'/mnt/d/desktop1/9.26.1/v2x_traffic_sim/_diag/fbx_cluster.txt'

# 整车簇的判据：零件包围盒中心落在主群那一格内（#198 表里绝大多数零件都在
# x -0.98..-0.66、y -4.05..-3.85，散件则跑到 x>1、y<-6）。留 0.3 m 的余量把
# 车身外的后视镜/天线这一类也算进簇内。
CLX = (-1.30, -0.30)
CLY = (-4.35, -3.55)


def patch_colorspace():
    """见 tools/pk197_fbxinspect.py 同名函数：这台 Blender 没有 Non-Color 枚举。"""
    import bpy_extras.node_shader_utils as nsu
    cls = nsu.ShaderImageTextureWrapper
    orig_set = cls.image_set

    def safe_set(self, image):
        want = self.colorspace_name
        if want is not ... and want not in [
                i.name for i in image.colorspace_settings.bl_rna.properties['name'].enum_items]:
            self.colorspace_name = 'sRGB'
        try:
            orig_set(self, image)
        finally:
            self.colorspace_name = want

    cls.image = property(cls.image_get, safe_set)


os.makedirs(os.path.dirname(OUT), exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
patch_colorspace()
bpy.ops.import_scene.fbx(filepath=FBX)

L = []
meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH']


def box(o):
    ws = [o.matrix_world @ mathutils.Vector(c) for c in o.bound_box]
    xs = [w.x for w in ws]; ys = [w.y for w in ws]; zs = [w.z for w in ws]
    return (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs),
            sum(xs) / 8.0, sum(ys) / 8.0, sum(zs) / 8.0)


cluster, stray = [], []
for o in meshes:
    b = box(o)
    (cluster if CLX[0] <= b[7 - 1] <= CLX[1] and CLY[0] <= b[8 - 1] <= CLY[1]
     else stray).append((o, b))


def agg(items, tag):
    if not items:
        L.append('  %s：0 个' % tag)
        return None
    x0 = min(b[0] for _, b in items); x1 = max(b[1] for _, b in items)
    y0 = min(b[2] for _, b in items); y1 = max(b[3] for _, b in items)
    z0 = min(b[4] for _, b in items); z1 = max(b[5] for _, b in items)
    tri = sum(len(o.data.polygons) for o, _ in items)
    L.append('  %s：%d 个零件 / 面 %d   L(X)=%.3f  W(Y)=%.3f  H(Z)=%.3f' % (
        tag, len(items), tri, x1 - x0, y1 - y0, z1 - z0))
    L.append('      世界范围 X %.3f..%.3f  Y %.3f..%.3f  Z %.3f..%.3f' % (
        x0, x1, y0, y1, z0, z1))
    return (x1 - x0, y1 - y0, z1 - z0, tri)


L.append('=== 1) 分簇 ===')
cl = agg(cluster, '整车簇')
st = agg(stray, '散件（要删掉）')
L.append('  散件清单：' + ', '.join(sorted(o.name for o, _ in stray)[:14]) +
         ('…' if len(stray) > 14 else ''))

if cl:
    lg, wd, hg, tri = cl
    L.append('')
    L.append('=== 2) 缩放试算（现车型包络：碰撞盒 1.60×0.80×0.40 在 z 0.05..0.45，'
             'cabin 0.72×0.68×0.24 顶到 0.69；车牌片在 x=±0.81）===')
    L.append('  原比例 L:W:H = 1 : %.3f : %.3f' % (wd / lg, hg / lg))
    s_fit = min(1.60 / lg, 0.80 / wd)
    L.append('  甲) 等比放到"footprint 尽量大" s=%.3f -> %.2f × %.2f × %.2f m'
             '（高 %.2f m，超包络 %s）' % (
                 s_fit, lg * s_fit, wd * s_fit, hg * s_fit, hg * s_fit,
                 '是' if hg * s_fit > 0.69 else '否'))
    s_h = 0.69 / hg
    L.append('  乙) 等比按"高不超过 cabin 顶 0.69" s=%.3f -> %.2f × %.2f × %.2f m'
             '（占地只有碰撞盒的 %.0f%%，牌片会浮在车身外）' % (
                 s_h, lg * s_h, wd * s_h, hg * s_h, 100.0 * (lg * s_h * wd * s_h) / (1.6 * 0.8)))
    L.append('  丙) 非等比 sx=%.3f sy=%.3f sz=%.3f 正好填满 1.60×0.80×0.69'
             '（长被拉 %.0f%%，看着不像同一台车）' % (
                 1.60 / lg, 0.80 / wd, 0.69 / hg, 100.0 * (1.60 / lg) / (0.80 / wd)))
    ROOM = r'/mnt/d/desktop1/9.26.1/v2x_traffic_sim/src/smart_car_sim/models/room_33/meshes/room_33.obj'
    room_tri = 0
    try:
        with open(ROOM, encoding='utf-8', errors='replace') as fh:
            for ln in fh:
                if ln.startswith('f '):
                    k = ln.count(' ') - 1
                    room_tri += max(1, k)
    except OSError:
        room_tri = -1
    L.append('  面数：整车簇 %d tri；六台车共约 %d tri。'
             '同场景参照：room_33.obj（沙盘本体）= %d 面' % (tri, tri * 6, room_tri))

L.append('')
L.append('=== 3) 轮子候选（default04x：四个角、直径 ~0.067、厚 0.029）===')
for o, b in sorted(cluster, key=lambda it: it[0].name):
    if o.name.lower().startswith('default'):
        L.append('  %-16s 中心(%.3f,%.3f,%.3f) 尺寸 %.3f×%.3f×%.3f 面 %d 材质 %s' % (
            o.name, b[6], b[7], b[8], b[1] - b[0], b[3] - b[2], b[5] - b[4],
            len(o.data.polygons),
            ','.join(m.material.name if m.material else '-' for m in o.material_slots)))
    for m in o.modifiers:
        L.append('  %s 带修改器 %s' % (o.name, m.type))

txt = '\n'.join(L)
with open(OUT, 'w', encoding='utf-8', newline='\n') as fh:
    fh.write(txt + '\n')
print(txt)
print('FBX_CLUSTER_DONE')
