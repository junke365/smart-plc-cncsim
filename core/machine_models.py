"""机床过程化模型 - 移植自 LinuxCNC vismach.py

每种机型定义一个 draw_xxx(gl_view) 函数，直接用 OpenGL 图元绘制。
坐标尺寸完全参照 LinuxCNC 的 vismach gui 文件。

参考文件:
  - configs/sim/axis/vismach/VMC_toolchange/vmcgui  (VMC三轴)
  - src/hal/user_comps/vismach/xyzac-trt-gui.py     (XYZAC)
  - src/hal/user_comps/vismach/xyzbc-trt-gui.py     (XYZBC)
  - src/hal/user_comps/vismach/5axisgui.py           (XYZBCW龙门)
  - src/hal/user_comps/vismach/xyzab-tdr-gui.py     (XYZAB)
  - src/hal/user_comps/vismach/max5gui.py            (MaxNC5轴)
"""
import math
from OpenGL.GL import *
from OpenGL.GLU import *


# ============================================================
# 辅助绘图函数 (对应 vismach 的 Box / CylinderX/Y/Z)
# ============================================================

_quadric = None

def _getQuadric():
    global _quadric
    if _quadric is None:
        _quadric = gluNewQuadric()
        gluQuadricNormals(_quadric, GLU_SMOOTH)
    return _quadric


def drawBox(x1, y1, z1, x2, y2, z2):
    """绘制长方体 - 对应 vismach Box"""
    xmin, xmax = min(x1, x2), max(x1, x2)
    ymin, ymax = min(y1, y2), max(y1, y2)
    zmin, zmax = min(z1, z2), max(z1, z2)

    glBegin(GL_QUADS)
    # -Z
    glNormal3f(0, 0, -1)
    glVertex3f(xmin, ymin, zmin)
    glVertex3f(xmax, ymin, zmin)
    glVertex3f(xmax, ymax, zmin)
    glVertex3f(xmin, ymax, zmin)
    # +Z
    glNormal3f(0, 0, 1)
    glVertex3f(xmin, ymin, zmax)
    glVertex3f(xmin, ymax, zmax)
    glVertex3f(xmax, ymax, zmax)
    glVertex3f(xmax, ymin, zmax)
    # -Y
    glNormal3f(0, -1, 0)
    glVertex3f(xmin, ymin, zmin)
    glVertex3f(xmin, ymin, zmax)
    glVertex3f(xmax, ymin, zmax)
    glVertex3f(xmax, ymin, zmin)
    # +Y
    glNormal3f(0, 1, 0)
    glVertex3f(xmin, ymax, zmin)
    glVertex3f(xmax, ymax, zmin)
    glVertex3f(xmax, ymax, zmax)
    glVertex3f(xmin, ymax, zmax)
    # -X
    glNormal3f(-1, 0, 0)
    glVertex3f(xmin, ymin, zmin)
    glVertex3f(xmin, ymax, zmin)
    glVertex3f(xmin, ymax, zmax)
    glVertex3f(xmin, ymin, zmax)
    # +X
    glNormal3f(1, 0, 0)
    glVertex3f(xmax, ymin, zmin)
    glVertex3f(xmax, ymin, zmax)
    glVertex3f(xmax, ymax, zmax)
    glVertex3f(xmax, ymax, zmin)
    glEnd()


def drawCylX(x1, r1, x2, r2):
    """X轴圆柱 - 对应 vismach CylinderX"""
    if x1 > x2:
        x1, x2 = x2, x1
        r1, r2 = r2, r1
    q = _getQuadric()
    glPushMatrix()
    glRotatef(90, 0, 1, 0)
    glTranslatef(0, 0, x1)
    gluCylinder(q, r1, r2, x2 - x1, 32, 1)
    glRotatef(180, 1, 0, 0)
    gluDisk(q, 0, r1, 32, 1)
    glRotatef(180, 1, 0, 0)
    glPushMatrix()
    glTranslatef(0, 0, x2 - x1)
    gluDisk(q, 0, r2, 32, 1)
    glPopMatrix()
    glPopMatrix()


def drawCylY(y1, r1, y2, r2):
    """Y轴圆柱 - 对应 vismach CylinderY"""
    if y1 > y2:
        y1, y2 = y2, y1
        r1, r2 = r2, r1
    q = _getQuadric()
    glPushMatrix()
    glRotatef(-90, 1, 0, 0)
    glTranslatef(0, 0, y1)
    gluCylinder(q, r1, r2, y2 - y1, 32, 1)
    glRotatef(180, 1, 0, 0)
    gluDisk(q, 0, r1, 32, 1)
    glRotatef(180, 1, 0, 0)
    glPushMatrix()
    glTranslatef(0, 0, y2 - y1)
    gluDisk(q, 0, r2, 32, 1)
    glPopMatrix()
    glPopMatrix()


def drawCylZ(z1, r1, z2, r2):
    """Z轴圆柱 - 对应 vismach CylinderZ"""
    if z1 > z2:
        z1, z2 = z2, z1
        r1, r2 = r2, r1
    q = _getQuadric()
    glPushMatrix()
    glTranslatef(0, 0, z1)
    gluCylinder(q, r1, r2, z2 - z1, 32, 1)
    glRotatef(180, 1, 0, 0)
    gluDisk(q, 0, r1, 32, 1)
    glRotatef(180, 1, 0, 0)
    glPushMatrix()
    glTranslatef(0, 0, z2 - z1)
    gluDisk(q, 0, r2, 32, 1)
    glPopMatrix()
    glPopMatrix()


def drawSphere(x, y, z, r):
    """球体"""
    q = _getQuadric()
    glPushMatrix()
    glTranslatef(x, y, z)
    gluSphere(q, r, 32, 16)
    glPopMatrix()


# ============================================================
# 颜色设置函数
# ============================================================

def glColor(r, g, b, a=1.0):
    """设置颜色 (RGBA 0~1)"""
    glColor4f(r, g, b, a)


# ============================================================
# VMC 三轴铣床
# 参考: configs/sim/axis/vismach/VMC_toolchange/vmcgui
# 使用我们自己的 STL 模型文件 (来自 LinuxCNC)
# ============================================================

def draw_vmc(gl_view):
    """VMC三轴立式加工中心 - 带自动换刀

    运动链:
      base (不动)
        ├── head (Z轴)
        ├── saddle (Y轴)
        │    └── table (X轴)
        └── arm (换刀机械臂旋转)
             └── carousel (刀库旋转)
    """
    pos = gl_view._axisPositions
    x = pos.get('X', 0.0)
    y = pos.get('Y', 0.0)
    z = pos.get('Z', 0.0)

    # 加载 STL 模型 (如果已加载)
    vmcParts = gl_view._vmcParts
    if vmcParts:
        # 使用已加载的STL模型 + 轴绑定渲染
        _drawVmcStlParts(gl_view)
        return

    # 回退到过程化绘制 (使用 Box/Cylinder)
    # 床身
    glColor(0.5, 0.5, 0.5)
    drawBox(-120, -100, -250, 120, 160, -100)
    # 立柱
    glColor(0.5, 0.5, 0.5)
    drawBox(-50, 100, -250, 50, 200, 260)

    # 主轴箱 + 主轴头 (Z轴)
    glPushMatrix()
    glTranslatef(0, 0, z)
    glColor(0.3, 0.3, 0.3)
    drawBox(-30, -30, 60, 30, 240, 135)
    # 主轴
    glColor(0.0, 0.5, 0.5)
    drawCylZ(20, 20, 135, 20)
    # 主轴鼻端
    glColor(0.0, 0.5, 0.5)
    drawCylZ(0, 10, 20, 15)
    # 电机
    drawCylZ(135, 30, 200, 30)
    glTranslatef(0, 200, 0)
    glPopMatrix()

    # 滑座 (Y轴)
    glPushMatrix()
    glTranslatef(0, y, 0)
    glColor(0.8, 0.8, 0.8)
    drawBox(-75, -53, -105, 75, 53, -73)

    # 工作台 (X轴)
    glPushMatrix()
    glTranslatef(x, 0, 0)
    glColor(0.4, 0.4, 0.4)
    drawBox(-150, -50, -69, 150, 50, -52)
    drawBox(-150, -40, -75, 150, 40, -69)
    glPopMatrix()

    glPopMatrix()

    # 刀库 + 换刀臂 (简化 - 不动)
    glColor(0.6, 0.2, 0.0)
    drawCylZ(-80, 40, -30, 40)


def _drawVmcStlParts(gl_view):
    """使用已加载的STL模型渲染VMC - 构建父子树并应用轴变换

    vmcParts 格式:
        (model, offset, axis_bind, parent_name, part_name, rot_axis)
    axis_bind: 关节轴名称 (如 "X","Y","Z") 或 None(不动)
    rot_axis: 旋转轴 "X"/"Y"/"Z" 或 None(平移关节)
    """
    parts = gl_view._vmcParts
    pos = gl_view._axisPositions

    # 构建名称→部件映射
    name_to_part = {}
    for i, pd in enumerate(parts):
        name = pd[4] if len(pd) > 4 else f"part_{i}"
        name_to_part[name] = pd

    # 构建父子关系
    children_of = {}
    roots = []
    for i, pd in enumerate(parts):
        name = pd[4] if len(pd) > 4 else f"part_{i}"
        parent = pd[3] if len(pd) > 3 else None
        if parent and parent in name_to_part:
            children_of.setdefault(parent, []).append(name)
        else:
            roots.append(name)

    rendered = set()

    def renderPart(partName):
        if partName in rendered:
            return
        rendered.add(partName)
        if partName not in name_to_part:
            return
        pd = name_to_part[partName]
        model = pd[0]
        base_off = pd[1] if len(pd) > 1 else [0, 0, 0]
        axis_bind = pd[2] if len(pd) > 2 else None
        parent_name = pd[3] if len(pd) > 3 else None
        rot_axis = pd[5] if len(pd) > 5 else None

        # 先渲染父部件
        if parent_name:
            renderPart(parent_name)

        glPushMatrix()

        # 应用基础偏移
        glTranslatef(base_off[0], base_off[1], base_off[2])

        # 应用关节变换
        if axis_bind and axis_bind in pos:
            angle = pos[axis_bind]
            if rot_axis:
                # 旋转关节
                if rot_axis == "X":
                    glRotatef(angle, 1, 0, 0)
                elif rot_axis == "Y":
                    glRotatef(angle, 0, 1, 0)
                elif rot_axis == "Z":
                    glRotatef(angle, 0, 0, 1)
            else:
                # 平移关节
                if axis_bind == "X":
                    glTranslatef(angle, 0, 0)
                elif axis_bind == "Y":
                    glTranslatef(0, angle, 0)
                elif axis_bind == "Z":
                    glTranslatef(0, 0, angle)

        model.draw()
        glPopMatrix()

    for name in roots:
        renderPart(name)
    # 渲染遗漏的部件
    all_names = set(name_to_part.keys())
    for name in all_names:
        if name not in rendered:
            renderPart(name)


# ============================================================
# XYZAC 转台型 (标准Hermle结构)
#
# 结构: Z移动主轴头, XY移动工作台, A+C在工作台上
# Z=0 = 工作台面
#
# 运动链:
#   固定: 床身 + 立柱 + Z导轨
#   Z轴: 立柱上的滑座 → 主轴箱 → 主轴 → 刀具
#   Y轴: 床身上的鞍座
#   X轴: 鞍座上的工作台
#   A轴: 工作台上的耳轴旋转 (绕X轴)
#   C轴: A轴上的旋转台面 (绕Z轴)
# ============================================================

def draw_xyzac(gl_view):
    """XYZAC 五轴铣床 (A+C 转台型, 标准Hermle结构)

    Z=0 在工作台面
    Z轴只移动主轴头, X/Y移动工作台, A/C在工作台上旋转
    """
    pos = gl_view._axisPositions
    x_val = pos.get('X', 0.0)
    y_val = pos.get('Y', 0.0)
    z_val = pos.get('Z', 0.0)
    a_val = pos.get('A', 0.0)
    c_val = pos.get('C', 0.0)

    # === 床身 (固定不动) ===
    glColor(0.5, 0.5, 0.5)
    drawBox(-200, -150, -200, 200, 150, -100)

    # === 立柱 (固定不动) ===
    glColor(0.55, 0.55, 0.55)
    drawBox(-60, 100, -200, 60, 160, 350)

    # === Z导轨 (固定在立柱上) ===
    glColor(0.7, 0.7, 0.7)
    drawCylZ(-100, 8, 340, 8)
    drawBox(-40, 105, -100, -30, 130, 340)
    drawBox(30, 105, -100, 40, 130, 340)

    # === 主轴头 (Z轴平移, 沿立柱上下) ===
    glPushMatrix()
    glTranslatef(0, 0, z_val)

    # 主轴箱 (灰色)
    glColor(0.6, 0.6, 0.65)
    drawBox(-35, 105, 100, 35, 150, 180)

    # 主轴 (青色)
    glColor(0, 0.6, 0.6)
    drawCylZ(50, 12, 100, 12)
    drawCylZ(30, 18, 50, 18)

    # 刀具 (红色) - 从主轴鼻端向下伸出到工作台面
    glColor(1, 0, 0)
    drawCylZ(0, 5, 30, 5)    # 刀柄: Z=0到Z=30 (主轴鼻端)
    drawCylZ(-20, 1, 0, 5)   # 刀尖锥: Z=-20到Z=0

    # 主轴电机 (青色)
    glColor(0, 0.5, 0.5)
    drawCylZ(180, 25, 230, 25)

    glPopMatrix()

    # === 鞍座 (Y轴平移) ===
    glPushMatrix()
    glTranslatef(0, y_val, 0)

    # 鞍座体 (浅灰)
    glColor(0.75, 0.75, 0.75)
    drawBox(-100, -60, -110, 100, 60, -95)

    # === 工作台 (X轴平移) ===
    glPushMatrix()
    glTranslatef(x_val, 0, 0)

    # 工作台体 (深灰)
    glColor(0.5, 0.5, 0.5)
    drawBox(-160, -50, -100, 160, 50, -85)
    # 工作台面 (Z=0 略下方)
    glColor(0.55, 0.55, 0.6)
    drawBox(-160, -50, -85, 160, 50, -80)

    # A轴支架 (左右耳轴座)
    glColor(0.6, 0.6, 0.6)
    drawBox(-85, -45, -80, -70, 45, -55)
    drawBox(70, -45, -80, 85, 45, -55)
    # 支架连接板
    drawBox(-85, 45, -82, 85, 55, -55)

    # A轴旋转体 (绕X轴旋转)
    glPushMatrix()
    glRotatef(a_val, 1, 0, 0)

    # 耳轴 (trunnion) - 橙色
    glColor(1, 0.5, 0)
    drawCylX(-82, 15, -70, 15)
    drawCylX(70, 12, 82, 12)

    # 轭板 (yoke) - 橙色
    glColor(1, 0.5, 0)
    drawBox(-60, -35, -65, 60, 35, -50)

    # C轴旋转台座 (蓝色)
    glColor(0.3, 0.5, 1)
    drawBox(-45, -45, -50, 45, 45, -38)

    # C轴旋转 (绕Z轴)
    glPushMatrix()
    glRotatef(c_val, 0, 0, 1)

    # 圆形工作台面 (紫色) - Z=0 是台面
    glColor(0.8, 0.3, 0.8)
    drawCylZ(-38, 48, 0, 48)
    # 十字标记
    glColor(1, 1, 1)
    drawCylX(-48, 1, 48, 1)
    drawCylY(-48, 1, 48, 1)

    glPopMatrix()  # C轴
    glPopMatrix()  # A轴
    glPopMatrix()  # X轴
    glPopMatrix()  # Y轴


# ============================================================
# XYZBC 转台型 (标准Hermle结构)
#
# 结构同XYZAC, 区别: B轴绕Y轴旋转
# B轴轭板需先绕Z旋转90度
# ============================================================

def draw_xyzbc(gl_view):
    """XYZBC 五轴铣床 (B+C 转台型)

    与XYZAC区别: B轴绕Y轴旋转
    Z=0在工作台面, Z只动主轴头
    """
    pos = gl_view._axisPositions
    x_val = pos.get('X', 0.0)
    y_val = pos.get('Y', 0.0)
    z_val = pos.get('Z', 0.0)
    b_val = pos.get('B', 0.0)
    c_val = pos.get('C', 0.0)

    # === 床身 (固定) ===
    glColor(0.5, 0.5, 0.5)
    drawBox(-200, -150, -200, 200, 150, -100)

    # === 立柱 (固定) ===
    glColor(0.55, 0.55, 0.55)
    drawBox(-60, 100, -200, 60, 160, 350)

    # === Z导轨 (固定) ===
    glColor(0.7, 0.7, 0.7)
    drawCylZ(-100, 8, 340, 8)
    drawBox(-40, 105, -100, -30, 130, 340)
    drawBox(30, 105, -100, 40, 130, 340)

    # === 主轴头 (Z轴平移) ===
    glPushMatrix()
    glTranslatef(0, 0, z_val)

    glColor(0.6, 0.6, 0.65)
    drawBox(-35, 105, 100, 35, 150, 180)

    glColor(0, 0.6, 0.6)
    drawCylZ(50, 12, 100, 12)
    drawCylZ(30, 18, 50, 18)

    # 刀具 (红色) - 从主轴鼻端向下伸出到工作台面
    glColor(1, 0, 0)
    drawCylZ(0, 5, 30, 5)    # 刀柄: Z=0到Z=30
    drawCylZ(-20, 1, 0, 5)   # 刀尖锥: Z=-20到Z=0

    glColor(0, 0.5, 0.5)
    drawCylZ(180, 25, 230, 25)

    glPopMatrix()

    # === 鞍座 (Y轴) ===
    glPushMatrix()
    glTranslatef(0, y_val, 0)

    glColor(0.75, 0.75, 0.75)
    drawBox(-100, -60, -110, 100, 60, -95)

    # === 工作台 (X轴) ===
    glPushMatrix()
    glTranslatef(x_val, 0, 0)

    glColor(0.5, 0.5, 0.5)
    drawBox(-160, -50, -100, 160, 50, -85)
    glColor(0.55, 0.55, 0.6)
    drawBox(-160, -50, -85, 160, 50, -80)

    # B轴支架
    glColor(0.6, 0.6, 0.6)
    drawBox(-85, -45, -80, -70, 45, -55)
    drawBox(70, -45, -80, 85, 45, -55)
    drawBox(-85, 45, -82, 85, 55, -55)

    # B轴旋转体 (绕Y轴旋转, 需先绕Z旋转90度)
    glPushMatrix()
    glRotatef(90, 0, 0, 1)
    glRotatef(b_val, 0, 1, 0)

    # 耳轴
    glColor(1, 0.5, 0)
    drawCylX(-82, 15, -70, 15)
    drawCylX(70, 12, 82, 12)

    # 轭板
    glColor(1, 0.5, 0)
    drawBox(-60, -35, -65, 60, 35, -50)

    # C轴台座
    glColor(0.3, 0.5, 1)
    drawBox(-45, -45, -50, 45, 45, -38)

    # C轴旋转
    glPushMatrix()
    glRotatef(c_val, 0, 0, 1)

    # 圆形工作台面
    glColor(0.8, 0.3, 0.8)
    drawCylZ(-38, 48, 0, 48)
    glColor(1, 1, 1)
    drawCylX(-48, 1, 48, 1)
    drawCylY(-48, 1, 48, 1)

    glPopMatrix()  # C轴
    glPopMatrix()  # B轴
    glPopMatrix()  # X轴
    glPopMatrix()  # Y轴


# ============================================================
# XYZBCW 龙门铣 (摆头+旋转工作台)
# 参考: src/hal/user_comps/vismach/5axisgui.py
#
# B轴 = 摆头绕Y轴旋转, C轴 = 主轴绕Z轴旋转
# W = Z滑枕轴, Y = 龙门横梁
# ============================================================

def draw_xyzbcw(gl_view):
    """XYZBCW 龙门五轴铣床 (摆头型)

    运动链:
      固定支腿
        └── jy: 龙门横梁 (Y轴)
             └── jx: 滑座 (X轴)
                  └── jz: 滑枕 (Z/W轴)
                       └── wrist: C轴旋转
                            └── tool: B轴摆头
                                 └── 刀具
      table: 工作台 (固定)
    """
    pos = gl_view._axisPositions
    x_val = pos.get('X', 0.0)
    y_val = pos.get('Y', 0.0)
    z_val = pos.get('Z', 0.0)
    b_val = pos.get('B', 0.0)
    c_val = pos.get('C', 0.0)

    pivot_len = 250

    # 固定支腿
    glColor(0.4, 0.4, 0.4)
    drawBox(-1000, 1000, -1000, -800, -1000, -900)
    drawBox(800, 1000, -1000, 1000, -1000, -900)

    # 龙门横梁 (Y轴)
    glPushMatrix()
    glTranslatef(0, y_val, 0)
    # 横梁
    glColor(0.5, 0.5, 0.5)
    drawBox(-800, 200, 350, 800, 400, 550)
    # 左右立柱
    drawBox(-1000, 200, -900, -800, 400, 550)
    drawBox(800, 200, -900, 1000, 400, 550)

    # 滑座 (X轴)
    glPushMatrix()
    glTranslatef(x_val, 0, 0)
    glColor(0.6, 0.6, 0.6)
    drawBox(-100, 100, 350, 100, 200, 550)

    # 滑枕 (Z轴)
    glPushMatrix()
    glTranslatef(0, 0, z_val)
    glColor(0.7, 0.7, 0.7)
    drawBox(-100, -100, 200, 100, 100, 900)
    # 初始位置偏移
    glTranslatef(0, 0, 150)

    # C轴旋转 (绕Z轴)
    glPushMatrix()
    glRotatef(c_val, 0, 0, 1)

    # 腕部
    glColor(0.3, 0.3, 0.8)
    drawBox(-50, -100, 0, 50, -25, 100)
    drawBox(-100, -100, 100, 100, 100, 150)
    drawCylZ(150, 75, 200, 75)

    # B轴摆头 (绕Y轴旋转)
    glPushMatrix()
    glRotatef(-b_val, 0, 1, 0)
    # B轴法兰
    glColor(0.3, 0.3, 0.8)
    drawCylY(-100, 75, -10, 75)
    drawCylY(-50, 60, 50, 60)

    # 刀具 (红色) - 从B轴法兰向下伸出
    glColor(1, 0, 0)
    drawCylZ(-(pivot_len), 5, -(pivot_len - 70), 5)  # 刀柄: 半径5
    drawCylZ(-(pivot_len + 20), 1, -(pivot_len), 5)   # 刀尖锥

    glPopMatrix()  # B轴
    glPopMatrix()  # C轴
    glPopMatrix()  # Z轴
    glPopMatrix()  # X轴
    glPopMatrix()  # Y轴

    # 工作台 (固定)
    glColor(0.4, 0.4, 0.4)
    drawBox(-500, -500, -450, 500, 500, -400)


# ============================================================
# XYZAB 双旋转工作台
# 参考: src/hal/user_comps/vismach/xyzab-tdr-gui.py
#
# A轴 = 绕X轴旋转, B轴 = 绕Y轴旋转
# 两个旋转轴都在工作台侧
# ============================================================

def draw_xyzab(gl_view):
    """XYZAB 五轴铣床 (双旋转工作台)

    Z=0 在工作台面
    主轴头: XYZ平移 (沿立柱/滑座)
    工作台: B轴(绕Y) + A轴(绕X) 双旋转
    """
    pos = gl_view._axisPositions
    x_val = pos.get('X', 0.0)
    y_val = pos.get('Y', 0.0)
    z_val = pos.get('Z', 0.0)
    a_val = pos.get('A', 0.0)
    b_val = pos.get('B', 0.0)

    # === 床身 (固定) ===
    glColor(0.5, 0.5, 0.5)
    drawBox(-200, -150, -200, 200, 150, -100)

    # === 立柱 (固定) ===
    glColor(0.55, 0.55, 0.55)
    drawBox(-60, 100, -200, 60, 160, 350)

    # === Z导轨 ===
    glColor(0.7, 0.7, 0.7)
    drawCylZ(-100, 8, 340, 8)
    drawBox(-40, 105, -100, -30, 130, 340)
    drawBox(30, 105, -100, 40, 130, 340)

    # === 主轴头 (Z轴平移) ===
    glPushMatrix()
    glTranslatef(0, 0, z_val)

    glColor(0.6, 0.6, 0.65)
    drawBox(-35, 105, 100, 35, 150, 180)

    glColor(0, 0.6, 0.6)
    drawCylZ(50, 12, 100, 12)
    drawCylZ(30, 18, 50, 18)

    # 刀具 (红色) - 从主轴鼻端向下伸出
    glColor(1, 0, 0)
    drawCylZ(0, 5, 30, 5)    # 刀柄: Z=0到Z=30
    drawCylZ(-20, 1, 0, 5)   # 刀尖锥: Z=-20到Z=0

    glColor(0, 0.5, 0.5)
    drawCylZ(180, 25, 230, 25)

    glPopMatrix()

    # === 双旋转工作台 (固定在床身前方) ===
    # B轴底座 (固定)
    glColor(0.6, 0.6, 0.6)
    drawBox(-60, -60, -100, 60, 60, -80)

    # B轴旋转 (绕Y轴)
    glPushMatrix()
    glRotatef(b_val, 0, 1, 0)

    # B轴耳轴
    glColor(1, 0.5, 0)
    drawCylX(-55, 12, 55, 12)
    # B轴轭板
    glColor(1, 0.5, 0)
    drawBox(-45, -30, -65, 45, 30, -45)

    # A轴旋转 (绕X轴)
    glPushMatrix()
    glRotatef(a_val, 1, 0, 0)

    # A轴旋转体
    glColor(1, 0.5, 0)
    drawBox(-40, -40, -45, 40, 40, -30)

    # 圆形工作台面 (紫色) - Z=0 是台面
    glColor(0.8, 0.3, 0.8)
    drawCylZ(-30, 45, 0, 45)
    # 十字标记
    glColor(1, 1, 1)
    drawCylX(-45, 1, 45, 1)
    drawCylY(-45, 1, 45, 1)

    glPopMatrix()  # A轴
    glPopMatrix()  # B轴


# ============================================================
# MaxNC 5轴 (XYZBC)
# 参考: src/hal/user_comps/vismach/max5gui.py
#
# 工作台上 C轴旋转, 主轴头 B轴摆头
# ============================================================

def draw_maxnc(gl_view):
    """MaxNC 5轴铣床 (XYZBC 摆头+转台)

    运动链:
      base: 床身
        └── saddle: 鞍座 (Y轴)
             └── table: 工作台 (X轴)
                  └── crotary: C轴旋转
                       └── work: 工件
      column: 立柱
        └── zslide: Z滑座 (Z轴)
             └── brotary: B轴旋转
                  └── head: 主轴头
    """
    pos = gl_view._axisPositions
    x_val = pos.get('X', 0.0)
    y_val = pos.get('Y', 0.0)
    z_val = pos.get('Z', 0.0)
    b_val = pos.get('B', 0.0)
    c_val = pos.get('C', 0.0)

    # === 床身 ===
    glColor(0, 0, 1)
    drawBox(-50, -150, 25, 50, 150, 44)
    drawBox(-40, -150, 44, 40, 150, 50)
    drawBox(-70, -150, 0, 70, -135, 25)
    drawBox(-23, -150, 27, 23, -210, 73)
    drawBox(-50, 125, 0, 50, 300, 25)
    drawBox(-50, 275, 25, 50, 300, 50)

    # === 鞍座 (Y轴) ===
    glPushMatrix()
    glTranslatef(0, -y_val, 0)
    glColor(1, 1, 0)
    drawBox(-53, -53, 44, 53, 53, 81)

    # === 工作台 (X轴) ===
    glPushMatrix()
    glTranslatef(-x_val, 0, 0)
    glColor(0, 1, 1)
    drawBox(-150, -50, 81, 150, 50, 100)
    drawBox(-150, -40, 75, 150, 40, 81)
    drawBox(150, -23, 52, 210, 23, 98)

    # C轴旋转
    glPushMatrix()
    glRotatef(-c_val, 0, 0, -1)
    # 圆形工作台
    glColor(1, 0, 1)
    drawCylZ(130, 50, 150, 50)
    # 侧面凸块
    drawBox(40, -5, 131, 54, 5, 149)
    glPopMatrix()

    # C轴台座
    glColor(1, 0, 1)
    drawBox(-53, -53, 100, 53, 53, 130)
    drawBox(53, -85, 100, 73, 53, 148)
    drawBox(22, -85, 102, 68, -145, 146)

    glPopMatrix()  # X轴
    glPopMatrix()  # Y轴

    # === 立柱 + Z滑座 ===
    glPushMatrix()
    glTranslatef(0, 0, z_val)

    # Z滑座
    glColor(1, 1, 0)
    drawBox(-53, 230, -53, 53, 256, 53)

    # B轴旋转
    glPushMatrix()
    glTranslatef(0, 0, 200)

    # B轴旋转体
    glColor(0, 1, 0)
    drawCylY(161, 50, 185, 50)

    glRotatef(-b_val, 0, -1, 0)

    # 主轴头
    glColor(0, 1, 0)
    drawBox(-25, 10, 60, 25, 125, 135)
    drawBox(-45, 125, 60, 45, 160, 135)
    drawBox(-50, 160, 0, 50, 170, 135)

    # 主轴
    glPushMatrix()
    glTranslatef(0, 40, 0)
    glColor(0, 0.5, 0.5)
    drawCylZ(55, 20, 140, 20)
    drawCylZ(45, 12, 55, 15)
    # 刀具
    glColor(1, 0, 0)
    drawCylZ(0, 5, 45, 5)    # 刀柄: Z=0到Z=45
    drawCylZ(-20, 1, 0, 5)   # 刀尖锥: Z=-20到Z=0
    glPopMatrix()

    glPopMatrix()  # B轴
    glPopMatrix()  # Z轴

    # 立柱本体
    glColor(0, 0, 1)
    drawBox(-50, 256, 15, 50, 275, 315)
    drawBox(-40, 250, 15, 40, 256, 315)
    drawBox(-23, 227, 315, 23, 273, 375)


# ============================================================
# SCARA 机器人
# ============================================================

def draw_scara(gl_view):
    """SCARA水平关节机器人 4轴

    运动链:
      base → link1(J1) → link2(J2) → link3(J3) → link4(J4)
    所有关节绕Z轴旋转
    """
    pos = gl_view._axisPositions
    j1 = pos.get('J1', 0.0)
    j2 = pos.get('J2', 0.0)
    j3 = pos.get('J3', 0.0)
    j4 = pos.get('J4', 0.0)

    # 底座
    glColor(0.5, 0.5, 0.5)
    drawBox(-25, -25, 0, 25, 25, 15)
    drawCylZ(15, 18, 20, 18)

    # Link1 (J1绕Z旋转)
    glPushMatrix()
    glRotatef(j1, 0, 0, 1)
    glColor(0.2, 0.6, 0.8)
    drawBox(0, -8, 20, 80, 8, 30)

    # Link2 (J2绕Z旋转)
    glPushMatrix()
    glTranslatef(80, 0, 0)
    glRotatef(j2, 0, 0, 1)
    glColor(0.3, 0.7, 0.4)
    drawBox(0, -6, 22, 70, 6, 30)
    drawCylZ(20, 8, 32, 8)

    # Link3 (J3绕Z旋转 - 滑杆)
    glPushMatrix()
    glTranslatef(70, 0, 0)
    glRotatef(j3, 0, 0, 1)
    glColor(0.8, 0.3, 0.3)
    drawBox(-5, -5, 10, 5, 5, 35)

    # Link4 (J4绕Z旋转 - 末端)
    glPushMatrix()
    glTranslatef(0, 0, 0)
    glRotatef(j4, 0, 0, 1)
    glColor(0.9, 0.9, 0.2)
    drawCylZ(8, 10, 12, 10)
    # 刀具/末端执行器 (红色)
    glColor(1, 0, 0)
    drawCylZ(-15, 4, 8, 4)    # 刀柄: Z=-15到Z=8, 半径4
    drawCylZ(-25, 2, -15, 4)  # 刀尖锥: Z=-25到Z=-15

    glPopMatrix()  # J4
    glPopMatrix()  # J3
    glPopMatrix()  # J2
    glPopMatrix()  # J1


# ============================================================
# Delta 并联机器人
# ============================================================

def draw_delta(gl_view):
    """Delta并联机器人 3轴 - 参照 LinuxCNC lineardelta.py

    结构: 三根竖直导轨 + 滑块 + 连杆 + 动平台
    参照: src/hal/user_comps/vismach/lineardelta.py

    运动链:
      固定顶部平台 (不动)
        ├── 三根导轨 (90°, 210°, 330°)
        │    └── 滑块 (随J1/J2/J3上下)
        ├── 连杆对 (滑块→动平台)
        └── 动平台 + 末端执行器
    """
    pos = gl_view._axisPositions
    j1 = pos.get('J1', 0.0)
    j2 = pos.get('J2', 0.0)
    j3 = pos.get('J3', 0.0)

    # 结构参数 (参照 lineardelta.py)
    EFF_OFF = 30       # 动平台偏移
    CARR_OFF = 30      # 滑块偏移
    RAIL_MIN = -375    # 导轨底端
    RAIL_MAX = 0       # 导轨顶端
    RAIL_R = 5         # 导轨半径
    TOP_R = 300        # 顶盘半径
    q = _getQuadric()

    # 关节值(-90~0°)映射到导轨位置
    def jointToZ(jval):
        t = (jval + 90) / 90.0  # -90→0, 0→1
        return RAIL_MIN + (RAIL_MAX - RAIL_MIN) * max(0, min(1, t))

    # ===== 顶盘 (固定平台) =====
    glColor(0.45, 0.45, 0.5)
    drawCylZ(20, TOP_R, 40, TOP_R)
    drawCylZ(10, TOP_R * 0.7, 20, TOP_R)
    # 加强筋
    glColor(0.4, 0.4, 0.45)
    drawCylZ(0, TOP_R * 0.5, 10, TOP_R * 0.7)

    # 三根导轨颜色
    rail_colors = [(1, 0.5, 0.5), (0.5, 1, 0.5), (0.5, 0.5, 1)]
    angles_deg = [90, 210, 330]
    joint_vals = [j1, j2, j3]

    # 计算滑块位置
    carriage_positions = []
    for angle_d, jval in zip(angles_deg, joint_vals):
        a = math.radians(angle_d)
        cz = jointToZ(jval)
        carriage_positions.append((a, cz))

    # ===== 动平台位置 (正运动学) =====
    from .kinematics import DeltaKinematics
    kin = DeltaKinematics()
    eff_pos, ok = kin.forward([j1, j2, j3])
    if ok:
        ex, ey, ez = eff_pos.tran.x, eff_pos.tran.y, eff_pos.tran.z
    else:
        ex, ey, ez = 0, 0, RAIL_MIN * 0.5

    # ===== 绘制导轨+滑块+连杆 =====
    for idx, ((a, cz), jval, col) in enumerate(
            zip(carriage_positions, joint_vals, rail_colors)):
        cx = CARR_OFF * math.cos(a)
        cy = CARR_OFF * math.sin(a)

        # 导轨 (竖直圆柱)
        glColor(0.5, 0.5, 0.5)
        drawCylZ(RAIL_MIN - 15, RAIL_R, 25, RAIL_R)
        # 导轨顶部固定座
        glColor(0.6, 0.6, 0.6)
        drawCylZ(15, RAIL_R + 4, 25, RAIL_R + 4)

        # 滑块
        glPushMatrix()
        glTranslatef(cx, cy, cz)
        glColor(*col)
        drawBox(-CARR_OFF, -12, -15, 0, 12, 15)
        # 滑块轴承
        glColor(0.7, 0.7, 0.7)
        drawCylZ(-12, RAIL_R + 2, 12, RAIL_R + 2)
        glPopMatrix()

        # 两根平行连杆 (滑块→动平台)
        # 连杆端点: 滑块上
        x0, y0, z0 = cx, cy, cz
        # 动平台上对应角点
        ea = a  # 角度同导轨
        x1 = ex + EFF_OFF * math.cos(ea)
        y1 = ey + EFF_OFF * math.sin(ea)
        z1 = ez

        dx = x1 - x0
        dy = y1 - y0
        dz = z1 - z0
        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        if length < 1:
            continue

        phi = math.degrees(math.atan2(dy, dx))
        theta = math.degrees(math.atan2(dz, math.hypot(dx, dy)))

        for sign in [1, -1]:
            # 连杆偏移 (垂直于导轨平面)
            sx = sign * CARR_OFF * 0.4 * (-math.sin(a))
            sy = sign * CARR_OFF * 0.4 * math.cos(a)

            glPushMatrix()
            glTranslatef(x0 + sx, y0 + sy, z0)
            glRotatef(phi, 0, 0, 1)
            glRotatef(90 - theta, 0, 1, 0)

            glColor(*col)
            gluCylinder(q, 3, 3, length, 12, 1)
            # 端盖
            glRotatef(180, 1, 0, 0)
            gluDisk(q, 0, 3, 12, 1)
            glRotatef(180, 1, 0, 0)
            glTranslatef(0, 0, length)
            gluDisk(q, 0, 3, 12, 1)

            glPopMatrix()

    # ===== 动平台 (末端执行器) =====
    glPushMatrix()
    glTranslatef(ex, ey, ez)

    # 三角形棱柱平台 (参照 TriangularPrismZ)
    h = EFF_OFF
    tri_h = 5
    glColor(0.6, 0.6, 0.65)
    glBegin(GL_TRIANGLES)
    # 顶面 (+Z)
    glNormal3f(0, 0, 1)
    glVertex3f(0, h, tri_h)
    glVertex3f(-h * math.sin(math.pi/3), -h * math.cos(math.pi/3), tri_h)
    glVertex3f(h * math.sin(math.pi/3), -h * math.cos(math.pi/3), tri_h)
    # 底面 (-Z)
    glNormal3f(0, 0, -1)
    glVertex3f(h * math.sin(math.pi/3), -h * math.cos(math.pi/3), 0)
    glVertex3f(-h * math.sin(math.pi/3), -h * math.cos(math.pi/3), 0)
    glVertex3f(0, h, 0)
    glEnd()

    # 中心安装柱
    glColor(0.4, 0.7, 0.4)
    drawCylZ(-25, 6, 0, 6)

    # 末端执行器 (吸盘)
    glColor(0.8, 0.3, 0.1)
    drawCylZ(-45, 10, -25, 10)
    glColor(1, 0.2, 0.1)
    drawCylZ(-55, 4, -45, 10)

    glPopMatrix()

    # ===== 底部地面 =====
    glColor(0.25, 0.25, 0.3)
    drawCylZ(RAIL_MIN - 30, 400, RAIL_MIN - 25, 400)


# ============================================================
# 模型构建器注册表
# ============================================================

MODEL_BUILDERS = {
    "VMC三轴铣床": draw_vmc,
    "五轴铣床(XYZAC)": draw_xyzac,
    "五轴铣床(XYZBC)": draw_xyzbc,
    "五轴铣床(XYZBCW)": draw_xyzbcw,
    "五轴铣床(XYZAB)": draw_xyzab,
    "五轴铣床(MaxNC)": draw_maxnc,
    "SCARA机器人": draw_scara,
    "Delta并联机器人": draw_delta,
    # 机器人类型使用STL/OBJ模型 + 旋转关节
    "PUMA六轴机器人": None,
    "Fanuc机器人": None,
    "Router开料机": None,
}


def getModelBuilder(machineName):
    """获取机型的过程化模型构建器"""
    return MODEL_BUILDERS.get(machineName)
