# Blender 无头勘查脚本 #198：把 车模.fbx 逐零件列成表，写进 _diag/fbx_parts.txt。
#
# 为什么还要第二趟：#197 只给了总量（X 4.161 / Y 3.887 / Z 0.670、10 个材质、
# 引用到的是 D:\desktop1\9.26.1\渲染\ 下三张微信截图而不是 材质\2\ 那 43 张）。
# 这三个数决定 #127 的三条设计，一条都还判不了：
#   1) scale 是多少（要装进 1.60×0.80×0.40 的碰撞包络，且车头顶到 x=+0.81 的牌位）；
#   2) 网格自带轮子吗（自带就要把 world 里那两个 cylinder 轮子的 **visual** 摘掉、
#      只留 link+joint+collision，否则画面是双层轮；反过来如果轮子中要能转，就得把
#      网格的轮子拆出来挂到 wheel link 下，代价完全不同）；
#   3) 三角面总数（六台车 × N，N 太大 gzclient 直接把帧率吃掉，取证画面就糊了）。
# 所以这一趟要的是**逐零件**的名字、尺寸、中心、材质与面数，不是总量。
import os

import bpy
import mathutils

FBX = r'/mnt/d/desktop1/9.26.1/车模.fbx'
OUT = r'/mnt/d/desktop1/9.26.1/v2x_traffic_sim/_diag/fbx_parts.txt'


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

lines = []
meshes = [o for o in bpy.context.scene.objects if o.type == 'MESH']
tot_tri = sum(len(o.data.polygons) for o in meshes)
tot_v = sum(len(o.data.vertices) for o in meshes)
lines.append('MESH 对象数 %d   顶点 %d   面 %d（triangulated 约 %d）' % (
    len(meshes), tot_v, tot_tri, tot_tri))
lines.append('')
lines.append('对象名'.ljust(26) + 'X长'.rjust(7) + 'Y宽'.rjust(7) + 'Z高'.rjust(7) +
             '  中心(x,y,z)'.ljust(28) + 'zmin'.rjust(7) + ' 面'.rjust(7) + '  材质')
for o in sorted(meshes, key=lambda m: -len(m.data.polygons)):
    ws = [o.matrix_world @ mathutils.Vector(c) for c in o.bound_box]
    xs = [w.x for w in ws]; ys = [w.y for w in ws]; zs = [w.z for w in ws]
    cx, cy, cz = sum(xs) / 8.0, sum(ys) / 8.0, sum(zs) / 8.0
    mats = [ms.material.name if ms.material else '(空)' for ms in o.material_slots]
    lines.append('%-26s%7.3f%7.3f%7.3f  (%6.2f,%6.2f,%5.2f)%7.3f%7d  %s' % (
        o.name[:26], max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs),
        cx, cy, cz, min(zs), len(o.data.polygons), ','.join(mats)[:40]))

# 按材质聚合：哪个材质覆盖多大一块、多少个零件 —— 决定"车身换色"还是"整体贴图"。
lines.append('')
lines.append('=== 按材质聚合（零件数 / 面数 / 该材质覆盖的世界包围盒）===')
bymat = {}
for o in meshes:
    ws = [o.matrix_world @ mathutils.Vector(c) for c in o.bound_box]
    for i, ms in enumerate(o.material_slots):
        nm = ms.material.name if ms.material else '(空材质)'
        d = bymat.setdefault(nm, {'n': 0, 'f': 0, 'x': [], 'y': [], 'z': []})
        d['n'] += 1
        d['f'] += len(o.data.polygons)
        d['x'] += [w.x for w in ws]; d['y'] += [w.y for w in ws]; d['z'] += [w.z for w in ws]
for nm, d in sorted(bymat.items(), key=lambda kv: -kv[1]['f']):
    lines.append('  %-24s 零件%3d 面%8d  X %.2f..%.2f  Y %.2f..%.2f  Z %.2f..%.2f' % (
        nm[:24], d['n'], d['f'], min(d['x']), max(d['x']),
        min(d['y']), max(d['y']), min(d['z']), max(d['z'])))

# 贴图：这台 FBX 到底引用了哪些图，在不在盘上（#197 说它在 渲染\ 不在 材质\2\）。
lines.append('')
lines.append('=== 材质 -> 贴图文件 ===')
for m in bpy.data.materials:
    tex = []
    if m.use_nodes and m.node_tree:
        for nd in m.node_tree.nodes:
            if nd.type == 'TEX_IMAGE' and nd.image:
                fp = bpy.path.abspath(nd.image.filepath)
                tex.append('%s%s %dx%d' % (os.path.basename(fp), '' if os.path.isfile(fp)
                                           else ' [缺失]', nd.image.size[0], nd.image.size[1]))
    lines.append('  %-24s %s' % (m.name[:24], ' | '.join(tex) if tex else '无贴图'))

# 层级：组001 有 58 个孩子，看清楚是"一台车的零件"还是"好几台车摆在一起"。
lines.append('')
lines.append('=== 顶层节点与子代数（判断是不是多辆车同场）===')
for o in sorted(bpy.context.scene.objects, key=lambda x: x.name):
    if o.parent is None:
        lines.append('  %-24s type=%-9s children=%d' % (o.name, o.type, len(o.children)))

txt = '\n'.join(lines)
with open(OUT, 'w', encoding='utf-8', newline='\n') as fh:
    fh.write(txt + '\n')
print(txt)
print('FBX_PARTS_DONE ->', OUT)
