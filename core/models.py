"""3D模型加载器 - 移植自 LinuxCNC vismach.py

支持加载 STL (ASCII/二进制) 和 OBJ 格式的3D模型文件。
提供 OpenGL 渲染接口，使用显示列表缓存加速渲染。
"""
import os
import struct
from typing import List, Tuple, Optional, Dict
from OpenGL.GL import *


class StlModel:
    """STL 模型加载和渲染 (支持 ASCII 和二进制格式)

    移植自 LinuxCNC lib/python/vismach.py 的 AsciiSTL 类。
    自动检测文件格式（通过检查首行或文件头），解析三角面片数据。
    """

    def __init__(self, filename: str = None, data: str = None):
        self._displayList: Optional[int] = None
        self._triangles: List[Tuple[List[float], List[List[float]]]] = []

        if data is not None:
            lines = data.split("\n")
            self._parseAsciiStl(lines)
        elif filename is not None:
            # 先用文本读取首行判断格式
            with open(filename, 'r', encoding='utf-8', errors='replace') as f:
                first_line = f.readline().strip()
            if first_line.startswith('solid'):
                self._parseAsciiFile(filename)
            else:
                self._parseBinaryStl(filename)

    def _parseAsciiFile(self, filepath: str):
        """从文件读取并解析 ASCII STL"""
        with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
            lines = f.readlines()
        self._parseAsciiStl(lines)

    def _parseBinaryStl(self, filepath: str):
        """解析二进制 STL 文件"""
        with open(filepath, 'rb') as f:
            f.read(80)  # 跳过80字节文件头
            num_tri = struct.unpack('<I', f.read(4))[0]
            for _ in range(num_tri):
                nx, ny, nz = struct.unpack('<3f', f.read(12))
                v1 = list(struct.unpack('<3f', f.read(12)))
                v2 = list(struct.unpack('<3f', f.read(12)))
                v3 = list(struct.unpack('<3f', f.read(12)))
                f.read(2)  # 跳过属性字节计数
                self._triangles.append(([nx, ny, nz], [v1, v2, v3]))
        print(f"[模型] 二进制STL加载: {os.path.basename(filepath)} ({len(self._triangles)} 三角面)")

    def _parseAsciiStl(self, lines):
        """解析 ASCII STL 数据"""
        t = []
        n = [0.0, 0.0, 0.0]

        for line in lines:
            if "normal" in line:
                parts = line.split()
                n = [float(x) for x in parts[-3:]]
            elif "vertex" in line:
                parts = line.split()
                x, y, z = float(parts[-3]), float(parts[-2]), float(parts[-1])
                t.append([x, y, z])
                if len(t) == 3:
                    if n == [0.0, 0.0, 0.0]:
                        # 自动计算法向量
                        dx1 = t[1][0] - t[0][0]
                        dy1 = t[1][1] - t[0][1]
                        dz1 = t[1][2] - t[0][2]
                        dx2 = t[2][0] - t[0][0]
                        dy2 = t[2][1] - t[0][1]
                        dz2 = t[2][2] - t[0][2]
                        n = [
                            dy1 * dz2 - dy2 * dz1,
                            dz1 * dx2 - dz2 * dx1,
                            dx1 * dy2 - dx1 * dy2
                        ]
                        # 重新计算正确的叉积
                        n[0] = dy1 * dz2 - dz1 * dy2
                        n[1] = dz1 * dx2 - dx1 * dz2
                        n[2] = dx1 * dy2 - dy1 * dx2
                    self._triangles.append((n[:], [v[:] for v in t]))
                    t = []
                    n = [0.0, 0.0, 0.0]

    def draw(self):
        """渲染模型 - 首次调用创建显示列表，后续直接调用"""
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
            # 释放原始数据节省内存
            self._triangles = []
        glCallList(self._displayList)

    def destroy(self):
        """释放OpenGL显示列表资源"""
        if self._displayList is not None:
            glDeleteLists(self._displayList, 1)
            self._displayList = None

    @property
    def triangleCount(self) -> int:
        """三角面片数量"""
        return len(self._triangles)


class ObjModel:
    """ASCII OBJ 模型加载和渲染

    移植自 LinuxCNC lib/python/vismach.py 的 AsciiOBJ 类。
    解析 OBJ 文件中的顶点、法向量和面数据。
    """

    def __init__(self, filename: str = None, data: str = None):
        self._displayList: Optional[int] = None
        self._vertices: List[List[float]] = []
        self._normals: List[List[float]] = []
        self._faces: List[List[Tuple[Optional[int], Optional[int], Optional[int]]]] = []

        if data is not None:
            lines = data.split("\n")
        elif filename is not None:
            with open(filename, 'r', encoding='utf-8', errors='replace') as f:
                lines = f.readlines()
        else:
            return

        self._parseAsciiObj(lines)

    def _parseInt(self, s: str) -> Optional[int]:
        """解析整数，空字符串返回 None"""
        if s == '':
            return None
        return int(s)

    def _parseSlash(self, word: str) -> Tuple[Optional[int], Optional[int], Optional[int]]:
        """解析 v/vt/vn 格式的面顶点引用"""
        parts = word.split("/")
        result = []
        for p in parts:
            result.append(self._parseInt(p))
        while len(result) < 3:
            result.append(None)
        return (result[0], result[1], result[2])

    def _parseAsciiObj(self, lines):
        """解析 ASCII OBJ 数据"""
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if parts[0] == 'vn':
                self._normals.append([float(x) for x in parts[1:]])
            elif parts[0] == 'v':
                self._vertices.append([float(x) for x in parts[1:]])
            elif parts[0] == 'f':
                face = [self._parseSlash(w) for w in parts[1:]]
                self._faces.append(face)

    def draw(self):
        """渲染模型"""
        if self._displayList is None:
            self._displayList = glGenLists(1)
            glNewList(self._displayList, GL_COMPILE)
            glDisable(GL_CULL_FACE)
            glBegin(GL_TRIANGLES)
            for face in self._faces:
                for v_idx, vt_idx, n_idx in face:
                    if v_idx is not None and n_idx is not None and n_idx <= len(self._normals):
                        glNormal3f(*self._normals[n_idx - 1])
                    if v_idx is not None and v_idx <= len(self._vertices):
                        glVertex3f(*self._vertices[v_idx - 1])
            glEnd()
            glEndList()
            # 释放原始数据
            self._vertices = []
            self._normals = []
            self._faces = []
        glCallList(self._displayList)

    def destroy(self):
        """释放OpenGL显示列表资源"""
        if self._displayList is not None:
            glDeleteLists(self._displayList, 1)
            self._displayList = None


class ModelManager:
    """模型管理器 - 加载和管理多个3D模型

    提供统一的接口加载 STL/OBJ 文件，管理模型缓存。
    """

    def __init__(self, modelsDir: str = None):
        if modelsDir is None:
            # 默认模型目录
            modelsDir = os.path.join(
                os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                'models'
            )
        self._modelsDir = modelsDir
        self._models: Dict[str, object] = {}

    def loadStl(self, name: str, filepath: str = None) -> Optional[StlModel]:
        """加载 STL 模型

        参数:
            name: 模型名称（用于缓存查找）
            filepath: STL 文件路径。若为 None，则在 modelsDir 中查找
        返回:
            加载成功的 StlModel，失败返回 None
        """
        if name in self._models:
            return self._models[name]

        if filepath is None:
            filepath = self._findFile(name, '.stl')
        if filepath is None or not os.path.exists(filepath):
            return None

        try:
            model = StlModel(filename=filepath)
            self._models[name] = model
            return model
        except Exception:
            return None

    def loadObj(self, name: str, filepath: str = None) -> Optional[ObjModel]:
        """加载 OBJ 模型"""
        if name in self._models:
            return self._models[name]

        if filepath is None:
            filepath = self._findFile(name, '.obj')
        if filepath is None or not os.path.exists(filepath):
            return None

        try:
            model = ObjModel(filename=filepath)
            self._models[name] = model
            return model
        except Exception:
            return None

    def getModel(self, name: str):
        """获取已加载的模型"""
        return self._models.get(name)

    def _findFile(self, name: str, ext: str) -> Optional[str]:
        """在模型目录中搜索文件"""
        for root, dirs, files in os.walk(self._modelsDir):
            for f in files:
                if f.endswith(ext):
                    full_path = os.path.join(root, f)
                    # 按名称匹配（不含扩展名）
                    if name.lower() in f.lower():
                        return full_path
        return None

    def listModels(self) -> Dict[str, List[str]]:
        """列出所有可用模型"""
        result = {}
        for root, dirs, files in os.walk(self._modelsDir):
            rel = os.path.relpath(root, self._modelsDir)
            if rel == '.':
                rel = 'root'
            for f in files:
                ext = os.path.splitext(f)[1].lower()
                if ext in ('.stl', '.obj'):
                    if rel not in result:
                        result[rel] = []
                    result[rel].append(f)
        return result
