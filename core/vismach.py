"""vismach 图元 - 移植自 LinuxCNC lib/python/vismach.py

提供 Box、CylinderX/Y/Z、Sphere 等几何图元，用于过程化构建机床3D模型。
每个图元生成三角面片数据，可通过 OpenGL 显示列表渲染，也可导出为 STL。
"""
import math
from typing import List, Tuple
from OpenGL.GL import *
from OpenGL.GLU import *


# ============================================================
# 几何图元 - 生成三角面片列表
# ============================================================

def box(x1, y1, z1, x2, y2, z2):
    """长方体 - 两对角点定义

    返回: [(normal, [v1, v2, v3]), ...] 三角面片列表
    """
    xmin, xmax = min(x1, x2), max(x1, x2)
    ymin, ymax = min(y1, y2), max(y1, y2)
    zmin, zmax = min(z1, z2), max(z1, z2)

    # 8个顶点
    v = [
        (xmin, ymin, zmin),  # 0
        (xmax, ymin, zmin),  # 1
        (xmax, ymax, zmin),  # 2
        (xmin, ymax, zmin),  # 3
        (xmin, ymin, zmax),  # 4
        (xmax, ymin, zmax),  # 5
        (xmax, ymax, zmax),  # 6
        (xmin, ymax, zmax),  # 7
    ]

    tris = []
    # 6个面，每面2个三角形
    faces = [
        ([0, 0, -1], [0, 1, 2, 3]),  # 底面 -Z
        ([0, 0, 1],  [7, 6, 5, 4]),  # 顶面 +Z
        ([0, -1, 0], [4, 5, 1, 0]),  # 前面 -Y
        ([0, 1, 0],  [6, 7, 3, 2]),  # 后面 +Y
        ([-1, 0, 0], [0, 3, 7, 4]),  # 左面 -X
        ([1, 0, 0],  [2, 6, 5, 1]),  # 右面 +X
    ]

    for normal, indices in faces:
        for i in range(1, len(indices) - 1):
            tris.append((normal, [list(v[indices[0]]),
                                   list(v[indices[i]]),
                                   list(v[indices[i + 1]])]))
    return tris


def cylinderX(x1, r1, x2, r2, segments=24):
    """X轴方向圆柱/圆锥 - 端点+半径

    返回: 三角面片列表
    """
    if x1 > x2:
        x1, x2 = x2, x1
        r1, r2 = r2, r1

    tris = []
    for i in range(segments):
        a1 = 2 * math.pi * i / segments
        a2 = 2 * math.pi * (i + 1) / segments

        # 侧面点 (Y-Z平面展开)
        y1s, z1s = r1 * math.cos(a1), r1 * math.sin(a1)
        y2s, z2s = r1 * math.cos(a2), r1 * math.sin(a2)
        y1e, z1e = r2 * math.cos(a1), r2 * math.sin(a1)
        y2e, z2e = r2 * math.cos(a2), r2 * math.sin(a2)

        # 法向量 (径向)
        ny1, nz1 = math.cos(a1), math.sin(a1)
        ny2, nz2 = math.cos(a2), math.sin(a2)

        # 侧面 - 两个三角形
        tris.append(([0, ny1, nz1], [[x1, y1s, z1s], [x2, y1e, z1e], [x2, y2e, z2e]]))
        tris.append(([0, ny1, nz1], [[x1, y1s, z1s], [x2, y2e, z2e], [x1, y2s, z2s]]))

    # 端盖
    for i in range(segments):
        a1 = 2 * math.pi * i / segments
        a2 = 2 * math.pi * (i + 1) / segments

        # 起始端盖 (-X方向法向)
        tris.append(([-1, 0, 0], [[x1, 0, 0],
                                    [x1, r1 * math.cos(a2), r1 * math.sin(a2)],
                                    [x1, r1 * math.cos(a1), r1 * math.sin(a1)]]))
        # 终止端盖 (+X方向法向)
        tris.append(([1, 0, 0], [[x2, 0, 0],
                                   [x2, r2 * math.cos(a1), r2 * math.sin(a1)],
                                   [x2, r2 * math.cos(a2), r2 * math.sin(a2)]]))

    return tris


def cylinderY(y1, r1, y2, r2, segments=24):
    """Y轴方向圆柱/圆锥"""
    if y1 > y2:
        y1, y2 = y2, y1
        r1, r2 = r2, r1

    tris = []
    for i in range(segments):
        a1 = 2 * math.pi * i / segments
        a2 = 2 * math.pi * (i + 1) / segments

        x1s, z1s = r1 * math.cos(a1), r1 * math.sin(a1)
        x2s, z2s = r1 * math.cos(a2), r1 * math.sin(a2)
        x1e, z1e = r2 * math.cos(a1), r2 * math.sin(a1)
        x2e, z2e = r2 * math.cos(a2), r2 * math.sin(a2)

        nx1, nz1 = math.cos(a1), math.sin(a1)
        nx2, nz2 = math.cos(a2), math.sin(a2)

        tris.append(([nx1, 0, nz1], [[x1s, y1, z1s], [x1e, y2, z1e], [x2e, y2, z2e]]))
        tris.append(([nx1, 0, nz1], [[x1s, y1, z1s], [x2e, y2, z2e], [x2s, y1, z2s]]))

    for i in range(segments):
        a1 = 2 * math.pi * i / segments
        a2 = 2 * math.pi * (i + 1) / segments

        tris.append(([0, -1, 0], [[0, y1, 0],
                                    [r1 * math.cos(a2), y1, r1 * math.sin(a2)],
                                    [r1 * math.cos(a1), y1, r1 * math.sin(a1)]]))
        tris.append(([0, 1, 0], [[0, y2, 0],
                                   [r2 * math.cos(a1), y2, r2 * math.sin(a1)],
                                   [r2 * math.cos(a2), y2, r2 * math.sin(a2)]]))

    return tris


def cylinderZ(z1, r1, z2, r2, segments=24):
    """Z轴方向圆柱/圆锥"""
    if z1 > z2:
        z1, z2 = z2, z1
        r1, r2 = r2, r1

    tris = []
    for i in range(segments):
        a1 = 2 * math.pi * i / segments
        a2 = 2 * math.pi * (i + 1) / segments

        x1s, y1s = r1 * math.cos(a1), r1 * math.sin(a1)
        x2s, y2s = r1 * math.cos(a2), r1 * math.sin(a2)
        x1e, y1e = r2 * math.cos(a1), r2 * math.sin(a1)
        x2e, y2e = r2 * math.cos(a2), r2 * math.sin(a2)

        nx1, ny1 = math.cos(a1), math.sin(a1)
        nx2, ny2 = math.cos(a2), math.sin(a2)

        tris.append(([nx1, ny1, 0], [[x1s, y1s, z1], [x1e, y1e, z2], [x2e, y2e, z2]]))
        tris.append(([nx1, ny1, 0], [[x1s, y1s, z1], [x2e, y2e, z2], [x2s, y2s, z1]]))

    for i in range(segments):
        a1 = 2 * math.pi * i / segments
        a2 = 2 * math.pi * (i + 1) / segments

        tris.append(([0, 0, -1], [[0, 0, z1],
                                    [r1 * math.cos(a1), r1 * math.sin(a1), z1],
                                    [r1 * math.cos(a2), r1 * math.sin(a2), z1]]))
        tris.append(([0, 0, 1], [[0, 0, z2],
                                   [r2 * math.cos(a2), r2 * math.sin(a2), z2],
                                   [r2 * math.cos(a1), r2 * math.sin(a1), z2]]))

    return tris


# ============================================================
# ProceduralModel - 可渲染的过程化模型
# ============================================================

class ProceduralModel:
    """过程化3D模型 - 通过图元函数构建，兼容 StlModel/ObjModel 的 draw() 接口"""

    def __init__(self, triangles=None):
        self._triangles = triangles or []
        self._displayList = None

    def draw(self):
        """渲染模型 - 创建/调用显示列表"""
        if self._displayList is None:
            self._displayList = glGenLists(1)
            glNewList(self._displayList, GL_COMPILE)
            glBegin(GL_TRIANGLES)
            for normal, verts in self._triangles:
                glNormal3f(*normal)
                glVertex3f(*verts[0])
                glVertex3f(*verts[1])
                glVertex3f(*verts[2])
            glEnd()
            glEndList()
            # 渲染完释放原始数据
            self._triangles = []
        glCallList(self._displayList)

    def destroy(self):
        """释放显示列表"""
        if self._displayList is not None:
            glDeleteLists(self._displayList, 1)
            self._displayList = None
