"""3D OpenGL 仿真视图 - 显示机床、刀具路径、当前位置和3D模型"""
import math
from typing import List, Tuple, Optional, Dict
try:
    from PySide6.QtOpenGLWidgets import QOpenGLWidget
except ImportError:
    from PySide6.QtWidgets import QOpenGLWidget
from PySide6.QtCore import Qt, QTimer, QPoint
from PySide6.QtGui import QMouseEvent, QWheelEvent
from OpenGL.GL import *
from OpenGL.GLU import *

from ..core.models import ModelManager, StlModel, ObjModel

class OpenGLView(QOpenGLWidget):
    """3D OpenGL 视图控件"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 300)

        # 视图变换
        self._rotX: float = -30.0
        self._rotY: float = 45.0
        self._zoom: float = -200.0
        self._panX: float = 0.0
        self._panY: float = 0.0

        # 鼠标交互
        self._lastMousePos: Optional[QPoint] = None
        self._mouseButton: int = 0

        # 刀具路径
        self._rapidPath: List[Tuple[float, float, float, float, float, float]] = []
        self._feedPath: List[Tuple[float, float, float, float, float, float]] = []
        self._arcPath: List[List[Tuple[float, float, float]]] = []

        # 当前位置
        self._currentX: float = 0.0
        self._currentY: float = 0.0
        self._currentZ: float = 0.0

        # 机床范围
        self._machineXRange: Tuple[float, float] = (-500, 500)
        self._machineYRange: Tuple[float, float] = (-500, 500)
        self._machineZRange: Tuple[float, float] = (-300, 100)

        # 网格
        self._showGrid: bool = True
        self._gridSize: float = 10.0

        # 3D模型
        self._modelManager: ModelManager = ModelManager()
        self._loadedModels: List[Tuple[object, List[float], List[float]]] = []
        self._vmcParts: List = []
        self._machineX: float = 0.0
        self._machineY: float = 0.0
        self._machineZ: float = 0.0
        self._showWorktable: bool = True

    def initializeGL(self):
        """OpenGL初始化"""
        glClearColor(0.15, 0.15, 0.2, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_NORMALIZE)
        glShadeModel(GL_SMOOTH)

        # 光照设置
        glLightfv(GL_LIGHT0, GL_POSITION, [100.0, 100.0, 200.0, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.3, 0.3, 0.3, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.8, 0.8, 0.8, 1.0])

    def resizeGL(self, w, h):
        """窗口大小改变时更新视口和投影"""
        glViewport(0, 0, w, h)
        self._updateProjection(w, h)

    def _updateProjection(self, w, h):
        """更新投影矩阵"""
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        aspect = w / max(h, 1)
        gluPerspective(45.0, aspect, 0.1, 5000.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        """绘制场景"""
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        # 视图变换
        glTranslatef(self._panX, self._panY, self._zoom)
        glRotatef(self._rotX, 1.0, 0.0, 0.0)
        glRotatef(self._rotY, 0.0, 0.0, 1.0)

        # 绘制坐标轴指示器
        self._drawAxisIndicator()

        # 绘制网格和工件台
        if self._showGrid:
            self._drawGrid()
        if self._showWorktable:
            self._drawWorktable()

        # 绘制3D模型
        self._drawModels()

        # 绘制刀具路径
        self._drawPaths()

        # 绘制当前刀具位置
        self._drawTool()

    def _drawGrid(self):
        """绘制XY平面网格"""
        glDisable(GL_LIGHTING)
        glBegin(GL_LINES)
        glColor3f(0.3, 0.3, 0.3)
        xMin, xMax = self._machineXRange
        yMin, yMax = self._machineYRange
        step = self._gridSize

        x = xMin
        while x <= xMax:
            glVertex3f(x, yMin, 0)
            glVertex3f(x, yMax, 0)
            x += step

        y = yMin
        while y <= yMax:
            glVertex3f(xMin, y, 0)
            glVertex3f(xMax, y, 0)
            y += step

        glEnd()
        glEnable(GL_LIGHTING)

    def _drawWorktable(self):
        """绘制工件台"""
        glEnable(GL_LIGHTING)
        glColor3f(0.4, 0.4, 0.45)
        xMin, xMax = self._machineXRange
        yMin, yMax = self._machineYRange
        z = -5.0
        thickness = 5.0

        glPushMatrix()

        glNormal3f(0, 0, 1)
        glBegin(GL_QUADS)
        glVertex3f(xMin, yMin, z)
        glVertex3f(xMax, yMin, z)
        glVertex3f(xMax, yMax, z)
        glVertex3f(xMin, yMax, z)
        glEnd()

        glColor3f(0.35, 0.35, 0.4)
        glBegin(GL_QUADS)
        glNormal3f(0, -1, 0)
        glVertex3f(xMin, yMin, z)
        glVertex3f(xMax, yMin, z)
        glVertex3f(xMax, yMin, z - thickness)
        glVertex3f(xMin, yMin, z - thickness)
        glEnd()

        glBegin(GL_QUADS)
        glNormal3f(0, 1, 0)
        glVertex3f(xMin, yMax, z)
        glVertex3f(xMax, yMax, z)
        glVertex3f(xMax, yMax, z - thickness)
        glVertex3f(xMin, yMax, z - thickness)
        glEnd()

        glBegin(GL_QUADS)
        glNormal3f(-1, 0, 0)
        glVertex3f(xMin, yMin, z)
        glVertex3f(xMin, yMax, z)
        glVertex3f(xMin, yMax, z - thickness)
        glVertex3f(xMin, yMin, z - thickness)
        glEnd()

        glBegin(GL_QUADS)
        glNormal3f(1, 0, 0)
        glVertex3f(xMax, yMin, z)
        glVertex3f(xMax, yMax, z)
        glVertex3f(xMax, yMax, z - thickness)
        glVertex3f(xMax, yMin, z - thickness)
        glEnd()

        glPopMatrix()

    def _drawModels(self):
        """绘制所有已加载的3D模型（支持VMC轴绑定动态更新）"""
        has_vmc = hasattr(self, '_vmcParts') and self._vmcParts
        if not self._loadedModels and not has_vmc:
            return
        glEnable(GL_LIGHTING)
        glEnable(GL_COLOR_MATERIAL)

        # 优先使用 VMC 轴绑定部件（有动态位移）
        if has_vmc:
            axis_pos = [getattr(self, '_machineX', 0.0),
                        getattr(self, '_machineY', 0.0),
                        getattr(self, '_machineZ', 0.0)]
            for model, base_off, rot, axis_bind in self._vmcParts:
                glPushMatrix()
                off_x = base_off[0]
                off_y = base_off[1]
                off_z = base_off[2]
                if isinstance(axis_bind, int) and 0 <= axis_bind <= 2:
                    off_x += axis_pos[0] if axis_bind == 0 else 0
                    off_y += axis_pos[1] if axis_bind == 1 else 0
                    off_z += axis_pos[2] if axis_bind == 2 else 0
                glTranslatef(off_x, off_y, off_z)
                glRotatef(rot[0], 1, 0, 0)
                glRotatef(rot[1], 0, 1, 0)
                glRotatef(rot[2], 0, 0, 1)
                model.draw()
                glPopMatrix()

        # 普通模型（无轴绑定）
        for model, pos, rot in self._loadedModels:
            glPushMatrix()
            glTranslatef(pos[0], pos[1], pos[2])
            glRotatef(rot[0], 1, 0, 0)
            glRotatef(rot[1], 0, 1, 0)
            glRotatef(rot[2], 0, 0, 1)
            model.draw()
            glPopMatrix()

    def _drawPaths(self):
        """绘制刀具路径"""
        glDisable(GL_LIGHTING)
        glLineWidth(2.0)

        if self._rapidPath:
            glColor3f(0.2, 0.5, 1.0)
            glBegin(GL_LINES)
            for seg in self._rapidPath:
                glVertex3f(seg[0], seg[1], seg[2])
                glVertex3f(seg[3], seg[4], seg[5])
            glEnd()

        if self._feedPath:
            glColor3f(1.0, 0.2, 0.2)
            glBegin(GL_LINES)
            for seg in self._feedPath:
                glVertex3f(seg[0], seg[1], seg[2])
                glVertex3f(seg[3], seg[4], seg[5])
            glEnd()

        if self._arcPath:
            glColor3f(0.2, 1.0, 0.2)
            for arcPts in self._arcPath:
                if len(arcPts) >= 2:
                    glBegin(GL_LINE_STRIP)
                    for pt in arcPts:
                        glVertex3f(pt[0], pt[1], pt[2])
                    glEnd()

        glLineWidth(1.0)
        glEnable(GL_LIGHTING)

    def _drawTool(self):
        """绘制当前刀具位置"""
        glPushMatrix()
        glTranslatef(self._currentX, self._currentY, self._currentZ)

        glColor3f(1.0, 1.0, 0.0)
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)
        glPushMatrix()
        glRotatef(-90, 1, 0, 0)
        gluCylinder(quad, 3.0, 3.0, 20.0, 16, 1)
        glTranslatef(0, 0, 20.0)
        gluCylinder(quad, 3.0, 0.5, 15.0, 16, 1)
        glPopMatrix()

        glColor3f(1.0, 0.0, 0.0)
        gluSphere(quad, 2.0, 16, 16)
        gluDeleteQuadric(quad)

        glPopMatrix()

    def _drawAxisIndicator(self):
        """绘制坐标轴指示器 (左下角)"""
        glPushMatrix()
        glTranslatef(
            -self._machineXRange[1] * 0.9,
            -self._machineYRange[1] * 0.9,
            0
        )
        glScalef(30, 30, 30)

        glDisable(GL_LIGHTING)
        glLineWidth(3.0)

        glColor3f(1.0, 0.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(1, 0, 0)
        glEnd()

        glColor3f(0.0, 1.0, 0.0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 1, 0)
        glEnd()

        glColor3f(0.0, 0.0, 1.0)
        glBegin(GL_LINES)
        glVertex3f(0, 0, 0)
        glVertex3f(0, 0, 1)
        glEnd()

        glLineWidth(1.0)
        glEnable(GL_LIGHTING)
        glPopMatrix()

    # ==================== 公共接口 ====================

    def updatePosition(self, x: float, y: float, z: float):
        """更新刀具位置"""
        self._currentX = x
        self._currentY = y
        self._currentZ = z
        self.update()

    def update_position(self, pos_dict_or_x, y=None, z=None):
        """更新刀具位置 - 支持字典或XYZ参数"""
        if isinstance(pos_dict_or_x, dict):
            self._currentX = pos_dict_or_x.get('X', 0.0)
            self._currentY = pos_dict_or_x.get('Y', 0.0)
            self._currentZ = pos_dict_or_x.get('Z', 0.0)
        else:
            self._currentX = pos_dict_or_x
            self._currentY = y or 0.0
            self._currentZ = z or 0.0
        self.update()

    def addRapidMove(self, x1, y1, z1, x2, y2, z2):
        """添加快移路径段"""
        self._rapidPath.append((x1, y1, z1, x2, y2, z2))
        self.update()

    def addFeedMove(self, x1, y1, z1, x2, y2, z2):
        """添加切削路径段"""
        self._feedPath.append((x1, y1, z1, x2, y2, z2))
        self.update()

    def addArcMove(self, points: List[Tuple[float, float, float]]):
        """添加圆弧路径段"""
        self._arcPath.append(points)
        self.update()

    def addArcFromParams(self, startX: float, startY: float, startZ: float,
                         endX: float, endY: float, endZ: float,
                         cx: float, cy: float, clockwise: bool = True,
                         segments: int = 32):
        """根据圆弧参数生成插值点并添加路径"""
        dx = startX - cx
        dy = startY - cy
        radius = math.sqrt(dx * dx + dy * dy)
        if radius < 0.0001:
            return

        startAngle = math.atan2(dy, dx)
        endAngle = math.atan2(endY - cy, endX - cx)

        if clockwise:
            if endAngle > startAngle:
                endAngle -= 2 * math.pi
        else:
            if endAngle < startAngle:
                endAngle += 2 * math.pi

        arcLen = abs(endAngle - startAngle)
        if arcLen < 0.001:
            return

        points = []
        for i in range(segments + 1):
            t = i / segments
            angle = startAngle + (endAngle - startAngle) * t
            x = cx + radius * math.cos(angle)
            y = cy + radius * math.sin(angle)
            z = startZ + (endZ - startZ) * t
            points.append((x, y, z))

        self._arcPath.append(points)
        self.update()

    def clearPaths(self):
        """清空所有路径"""
        self._rapidPath.clear()
        self._feedPath.clear()
        self._arcPath.clear()
        self.update()

    def clear_paths(self):
        """清空所有路径 (snake_case 别名)"""
        self.clearPaths()

    def resetView(self):
        """重置视图"""
        self._rotX = -30.0
        self._rotY = 45.0
        self._zoom = -200.0
        self._panX = 0.0
        self._panY = 0.0
        self.update()

    def reset_view(self):
        """重置视图 (snake_case 别名)"""
        self.resetView()

    # ==================== 默认5轴VMC机床 ====================

    def loadDefaultMachine(self):
        """加载默认5轴VMC立式加工中心模型

        参照 LinuxCNC configs/sim/axis/vismach/VMC_toolchange/vmcgui 的布局：
        - base: 底座（不动）
        - head: 主轴头，随Z轴移动
        - table: 工作台，随X轴移动
        - saddle: 滑座，随Y轴移动
        - carousel: 刀库
        - arm: 机械臂
        """
        import os
        models_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'models', 'vmc'
        )
        if not os.path.isdir(models_dir):
            print("[GL] 未找到VMC模型目录")
            return

        self._loadedModels.clear()

        required = ['base.stl', 'head.stl', 'table.stl', 'saddle.stl', 'carousel.stl', 'arm.stl']
        for name in required:
            if not os.path.exists(os.path.join(models_dir, name)):
                print(f"[GL] 缺少模型文件: {name}")
                return

        base = StlModel(filename=os.path.join(models_dir, 'base.stl'))
        head = StlModel(filename=os.path.join(models_dir, 'head.stl'))
        table = StlModel(filename=os.path.join(models_dir, 'table.stl'))
        saddle = StlModel(filename=os.path.join(models_dir, 'saddle.stl'))
        carousel = StlModel(filename=os.path.join(models_dir, 'carousel.stl'))
        arm = StlModel(filename=os.path.join(models_dir, 'arm.stl'))

        print("[GL] VMC模型加载完成: base, head, table, saddle, carousel, arm")

        # 各部件标识，axis_binding: 0=X, 1=Y, 2=Z, -1=不动
        self._vmcParts = [
            (base,      [0, 0, 0],   [0, 0, 0], -1),  # 底座不动
            (head,      [0, 0, 4],   [0, 0, 0], 2),    # 主轴头随Z
            (table,     [0, 8, 0],   [0, 0, 0], 0),    # 工作台随X
            (saddle,    [0, 7, 0],   [0, 0, 0], 1),    # 滑座随Y
            (carousel,  [0, 0, 0],   [0, 0, 0], -1),   # 刀库不动
            (arm,       [0, 0, 0],   [0, 0, 0], -1),   # 机械臂不动
        ]

        self._zoom = -300.0
        self._rotX = -25.0
        self._rotY = 225.0
        print("[GL] VMC机床就绪, 视图已调整")
        self.update()

    def updateMachinePosition(self, x: float, y: float, z: float):
        """更新VMC机床各部件位置（根据轴坐标）

        工作台随X、滑座随Y、主轴头随Z移动
        """
        self._machineX = x
        self._machineY = y
        self._machineZ = z
        self.update()

    def setMachineView(self, grid_range, zoom, rotX, rotY, show_worktable=True):
        """切换机型时更新视图参数

        参数:
            grid_range: (min, max) 网格范围
            zoom: 缩放距离
            rotX, rotY: 旋转角度
            show_worktable: 是否显示工件台
        """
        lo, hi = grid_range
        self._machineXRange = (lo, hi)
        self._machineYRange = (lo, hi)
        self._gridSize = max(10.0, (hi - lo) / 50.0)
        self._zoom = zoom
        self._rotX = rotX
        self._rotY = rotY
        self._panX = 0.0
        self._panY = 0.0
        self._showWorktable = show_worktable
        self.update()
        print(f"[GL] 视图已更新: grid=[{lo},{hi}] zoom={zoom} worktable={show_worktable}")

    # ==================== 3D模型接口 ====================

    def loadStlModel(self, name: str, filepath: str = None,
                     pos: List[float] = None, rot: List[float] = None) -> bool:
        """加载STL模型到场景

        参数:
            name: 模型名称
            filepath: STL文件路径（可选）
            pos: [x, y, z] 平移偏移（可选）
            rot: [rx, ry, rz] 旋转角度（可选）
        返回:
            是否加载成功
        """
        model = self._modelManager.loadStl(name, filepath)
        if model is None:
            return False
        self._loadedModels.append((model, pos or [0, 0, 0], rot or [0, 0, 0]))
        self.update()
        return True

    def loadObjModel(self, name: str, filepath: str = None,
                     pos: List[float] = None, rot: List[float] = None) -> bool:
        """加载OBJ模型到场景"""
        model = self._modelManager.loadObj(name, filepath)
        if model is None:
            return False
        self._loadedModels.append((model, pos or [0, 0, 0], rot or [0, 0, 0]))
        self.update()
        return True

    def loadModel(self, filepath: str, pos: List[float] = None,
                  rot: List[float] = None) -> bool:
        """根据文件扩展名自动加载模型"""
        import os
        ext = os.path.splitext(filepath)[1].lower()
        name = os.path.splitext(os.path.basename(filepath))[0]
        if ext == '.stl':
            return self.loadStlModel(name, filepath, pos, rot)
        elif ext == '.obj':
            return self.loadObjModel(name, filepath, pos, rot)
        return False

    def load_model(self, filepath: str, pos: List[float] = None,
                   rot: List[float] = None) -> bool:
        """加载模型 (snake_case 别名)"""
        return self.loadModel(filepath, pos, rot)

    def clearModels(self):
        """清空所有模型"""
        self._loadedModels.clear()
        self._vmcParts.clear()
        self.update()

    def clear_models(self):
        """清空所有模型 (snake_case 别名)"""
        self.clearModels()

    def getModelManager(self) -> ModelManager:
        """获取模型管理器"""
        return self._modelManager

    def listAvailableModels(self) -> Dict[str, List[str]]:
        """列出所有可用模型"""
        return self._modelManager.listModels()

    # ==================== 鼠标交互 ====================

    def mousePressEvent(self, event: QMouseEvent):
        """鼠标按下事件"""
        self._lastMousePos = event.position().toPoint() if hasattr(event, 'position') else event.pos()
        self._mouseButton = event.button()

    def mouseMoveEvent(self, event: QMouseEvent):
        """鼠标移动事件 - 旋转/平移/缩放"""
        if self._lastMousePos is None:
            return

        pos = event.position().toPoint() if hasattr(event, 'position') else event.pos()
        dx = pos.x() - self._lastMousePos.x()
        dy = pos.y() - self._lastMousePos.y()
        self._lastMousePos = pos

        if self._mouseButton == Qt.LeftButton:
            self._rotY += dx * 0.5
            self._rotX += dy * 0.5
            self._rotX = max(-90, min(90, self._rotX))
        elif self._mouseButton == Qt.MiddleButton:
            self._panX += dx * 0.5
            self._panY -= dy * 0.5
        elif self._mouseButton == Qt.RightButton:
            self._zoom += dy * 0.5
            self._zoom = min(-10, max(-2000, self._zoom))

        self.update()

    def wheelEvent(self, event: QWheelEvent):
        """鼠标滚轮事件 - 缩放"""
        delta = event.angleDelta().y()
        self._zoom += delta * 0.1
        self._zoom = min(-10, max(-2000, self._zoom))
        self.update()

    def mouseDoubleClickEvent(self, event: QMouseEvent):
        """双击重置视图"""
        self.resetView()
