# Blender 无头勘查脚本：把 车模.fbx 的结构、材质、贴图引用与真实尺寸打出来。
#
# 为什么先勘查再转换（不是"导出一次看看行不行"）：#127 的硬约束是**只换外观、不动碰撞**。
# world 里 car_1..car_6 的 collision 盒是 1.6×0.8×0.4（+ cabin 0.72×0.68×0.24），
# 而停车场/卡口的全部标定都挂在这套尺寸上：
#   * 道闸杆高、机箱偏心、原地转扫掠半径 hypot(1.6,0.8)/2=0.894、
#   * 车牌面片 plate_front/plate_rear 在 x=±0.81（比 1.6 的半长多 0.01，正好贴尾）、
#   * 卡口相机的 near_radius 1.4 / far_radius 2.5 按"底盘原点"判，
# 网格只要有一维超出这个包络，画面上就是"车穿杆/牌扎进车身/拍不到牌"，而体检脚本量的是
# collision，不会报红。所以必须先量出 FBX 的包围盒与朝向，再决定 scale 与 roll。
import math
import os

import bpy
import mathutils

FBX = r'/mnt/d/desktop1/9.26.1/车模.fbx'
MAT_DIR = r'/mnt/d/desktop1/9.26.1/材质/2'


def patch_colorspace():
    """把 FBX 导入器写死的 'Non-Color' 降级成这台 Blender 有的枚举。

    Ubuntu 打包的 blender-data 没带 colormanagement（启动即报 "using fallback mode for
    management"、"could not find role data role"），于是 image.colorspace_settings 的枚举
    只剩 ('Linear','sRGB')；而 io_scene_fbx 在 3.0.1 里对每张贴图（含 Alpha/法线）都走
    node_shader_utils 的 colorspace_name='Non-Color'，赋值直接 TypeError，**整个导入回滚**
    （现场第一次跑就是这一条栈）。这里只在枚举缺失时降级，不影响几何、UV 和
    "哪个材质引用哪张贴图" —— 而 #127 要的正是这三样，色彩管理只作用于 Blender 预览。
    """
    import bpy_extras.node_shader_utils as nsu
    cls = nsu.ShaderImageTextureWrapper      # 3.0.1 里的类名（不是 ImageTexture）
    orig_set = cls.image_set

    def safe_set(self, image):
        want = self.colorspace_name
        had = want is not ... and want not in [
            i.name for i in image.colorspace_settings.bl_rna.properties['name'].enum_items]
        if had:
            self.colorspace_name = 'sRGB'
        try:
            orig_set(self, image)
        finally:
            self.colorspace_name = want

    cls.image = property(cls.image_get, safe_set)


# 先清场再打补丁：read_factory_settings 会重载脚本模块，补丁在它之后才不会被抹掉。
bpy.ops.wm.read_factory_settings(use_empty=True)
patch_colorspace()

print('=== 0) Blender 版本 / FBX 文件 ===')
print('  bpy', bpy.app.version_string)
print('  fbx bytes', os.path.getsize(FBX))
print('  材质目录文件数', len(os.listdir(MAT_DIR)))

print('=== 1) 导入（清场与补丁已在文件头做过；默认 global up=Z，importer 会做 Y-up→Z-up）===')
bpy.ops.import_scene.fbx(filepath=FBX)
objs = [o for o in bpy.context.scene.objects]
print('  对象总数', len(objs))
meshes = [o for o in objs if o.type == 'MESH']
print('  MESH %d / EMPTY %d / ARMATURE %d / 其它 %d' % (
    len(meshes), sum(o.type == 'EMPTY' for o in objs),
    sum(o.type == 'ARMATURE' for o in objs),
    sum(o.type not in ('MESH', 'EMPTY', 'ARMATURE') for o in objs)))

print('=== 2) 每个 MESH 的对象名 / 顶点数 / 局部包围盒 / 世界包围盒 ===')
for o in meshes:
    ws = [o.matrix_world @ mathutils.Vector(c) for c in o.bound_box]
    xs = [w.x for w in ws]; ys = [w.y for w in ws]; zs = [w.z for w in ws]
    print('  %-28s v=%-7d f=%-6d 世界尺寸 X=%.3f Y=%.3f Z=%.3f  zmin=%.3f' % (
        o.name, len(o.data.vertices), len(o.data.polygons),
        max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs), min(zs)))

print('=== 3) 合并后的世界总包围盒（这就是要缩放进 1.6×0.8 的那个数）===')
allw = []
for o in meshes:
    for c in o.bound_box:
        allw.append(o.matrix_world @ mathutils.Vector(c))
if allw:
    ax = [w.x for w in allw]; ay = [w.y for w in allw]; az = [w.z for w in allw]
    print('  X %.3f..%.3f (=%.3f)' % (min(ax), max(ax), max(ax) - min(ax)))
    print('  Y %.3f..%.3f (=%.3f)' % (min(ay), max(ay), max(ay) - min(ay)))
    print('  Z %.3f..%.3f (=%.3f)' % (min(az), max(az), max(az) - min(az)))
    print('  长边在 %s 轴' % ('X' if (max(ax) - min(ax)) >= (max(ay) - min(ay)) else 'Y'))

print('=== 4) 材质槽与贴图引用（只看引用到哪些文件，能不能对上 材质/2）===')
mats = {}
for o in meshes:
    for ms in o.material_slots:
        m = ms.material
        if m is None:
            continue
        mats[m.name] = m
print('  材质数', len(mats))
hit = miss = 0
for name, m in sorted(mats.items()):
    nodes = [nd for nd in (m.node_tree.nodes if m.use_nodes and m.node_tree else [])
             if nd.type == 'TEX_IMAGE']
    bsdf = next((nd for nd in (m.node_tree.nodes if m.use_nodes and m.node_tree else [])
                 if nd.type == 'BSDF_PRINCIPLED'), None)
    basecol = None
    if bsdf and not bsdf.inputs['Base Color'].is_linked:
        basecol = tuple(round(v, 3) for v in bsdf.inputs['Base Color'].default_value)
    files = []
    for nd in nodes:
        img = nd.image
        if img is None:
            continue
        fp = bpy.path.abspath(img.filepath) if img.filepath else '(内嵌/无路径)'
        files.append((nd.location if False else '', fp, img.size[:]))
    tag = '  %-26s base=%-22s ' % (name[:26], str(basecol))
    if not files:
        print(tag + '无贴图节点')
        continue
    for _, fp, sz in files:
        ok = os.path.isfile(fp) if fp and not fp.startswith('(') else False
        hit += ok; miss += (not ok)
        print(tag + '%s -> %s %s %s' % (fp[-58:], sz, '存在' if ok else '缺失', ''))
        tag = ' ' * len(tag)
print('  贴图命中 %d / 缺失 %d' % (hit, miss))

print('=== 5) FBX 里是否存在 ARMATURE（带骨骼的话必须 bake 成静态网格）===')
for o in objs:
    if o.type in ('ARMATURE', 'EMPTY') and o.parent is None:
        print('  %-24s type=%s children=%d' % (o.name, o.type, len(o.children)))

print('=== 6) 材质目录里有哪些文件（转换时只拷 FBX 真正引用的那些）===')
for f in sorted(os.listdir(MAT_DIR))[:50]:
    p = os.path.join(MAT_DIR, f)
    print('  %-46s %8d' % (f[:46], os.path.getsize(p) if os.path.isfile(p) else -1))
print('FBX_INSPECT_DONE')
