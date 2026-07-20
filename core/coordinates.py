import math
from dataclasses import dataclass, field
from typing import List, Optional
from enum import Enum, IntEnum


@dataclass
class PmCartesian:
  """三维笛卡尔点/向量"""
  x: float = 0.0
  y: float = 0.0
  z: float = 0.0

  def mag(self) -> float:
    """向量长度"""
    return math.sqrt(self.x * self.x + self.y * self.y + self.z * self.z)

  def unit(self) -> 'PmCartesian':
    """返回单位向量"""
    m = self.mag()
    if m == 0.0:
      return PmCartesian(0.0, 0.0, 0.0)
    return PmCartesian(self.x / m, self.y / m, self.z / m)

  def __add__(self, other: 'PmCartesian') -> 'PmCartesian':
    """向量加法"""
    return PmCartesian(self.x + other.x, self.y + other.y, self.z + other.z)

  def __sub__(self, other: 'PmCartesian') -> 'PmCartesian':
    """向量减法"""
    return PmCartesian(self.x - other.x, self.y - other.y, self.z - other.z)

  def __mul__(self, scalar: float) -> 'PmCartesian':
    """标量乘法"""
    return PmCartesian(self.x * scalar, self.y * scalar, self.z * scalar)

  def __rmul__(self, scalar: float) -> 'PmCartesian':
    """标量乘法（标量在前）"""
    return self.__mul__(scalar)

  def __neg__(self) -> 'PmCartesian':
    """取反"""
    return PmCartesian(-self.x, -self.y, -self.z)

  def __eq__(self, other: object) -> bool:
    """相等比较"""
    if not isinstance(other, PmCartesian):
      return NotImplemented
    return (math.isclose(self.x, other.x) and
            math.isclose(self.y, other.y) and
            math.isclose(self.z, other.z))

  def dot(self, other: 'PmCartesian') -> float:
    """点积"""
    return self.x * other.x + self.y * other.y + self.z * other.z

  def cross(self, other: 'PmCartesian') -> 'PmCartesian':
    """叉积"""
    return PmCartesian(
      self.y * other.z - self.z * other.y,
      self.z * other.x - self.x * other.z,
      self.x * other.y - self.y * other.x
    )

  def distance_to(self, other: 'PmCartesian') -> float:
    """两点距离"""
    return (self - other).mag()

  def to_list(self) -> List[float]:
    """转为列表"""
    return [self.x, self.y, self.z]

  def copy(self) -> 'PmCartesian':
    """返回副本"""
    return PmCartesian(self.x, self.y, self.z)


@dataclass
class EmcPose:
  """9轴位姿: XYZ + ABC + UVW"""
  tran: PmCartesian = field(default_factory=PmCartesian)
  a: float = 0.0
  b: float = 0.0
  c: float = 0.0
  u: float = 0.0
  v: float = 0.0
  w: float = 0.0

  def copy(self) -> 'EmcPose':
    """返回副本"""
    return EmcPose(
      tran=self.tran.copy(),
      a=self.a, b=self.b, c=self.c,
      u=self.u, v=self.v, w=self.w
    )

  def __eq__(self, other: object) -> bool:
    """相等比较"""
    if not isinstance(other, EmcPose):
      return NotImplemented
    return (self.tran == other.tran and
            math.isclose(self.a, other.a) and
            math.isclose(self.b, other.b) and
            math.isclose(self.c, other.c) and
            math.isclose(self.u, other.u) and
            math.isclose(self.v, other.v) and
            math.isclose(self.w, other.w))


@dataclass
class PmRotationMatrix:
  """3x3旋转矩阵"""
  s: List[List[float]] = field(
    default_factory=lambda: [[0.0] * 3 for _ in range(3)]
  )

  def copy(self) -> 'PmRotationMatrix':
    """返回副本"""
    mat = PmRotationMatrix()
    for i in range(3):
      for j in range(3):
        mat.s[i][j] = self.s[i][j]
    return mat

  @staticmethod
  def identity() -> 'PmRotationMatrix':
    """返回单位矩阵"""
    mat = PmRotationMatrix()
    mat.s[0][0] = 1.0
    mat.s[1][1] = 1.0
    mat.s[2][2] = 1.0
    return mat


@dataclass
class PmRpy:
  """横滚/俯仰/偏航"""
  r: float = 0.0  # roll  绕X轴
  p: float = 0.0  # pitch 绕Y轴
  y: float = 0.0  # yaw   绕Z轴


@dataclass
class PmCartLine:
  """3D直线段"""
  start: PmCartesian = field(default_factory=PmCartesian)
  end: PmCartesian = field(default_factory=PmCartesian)


@dataclass
class PmCircle:
  """3D圆弧"""
  center: PmCartesian = field(default_factory=PmCartesian)
  normal: PmCartesian = field(default_factory=lambda: PmCartesian(0, 0, 1))
  radius: float = 0.0
  start_angle: float = 0.0
  end_angle: float = 0.0
  spiral: float = 0.0  # 螺旋高度


@dataclass
class PmLine9:
  """9轴直线段"""
  xyz: PmCartLine = field(default_factory=PmCartLine)
  abc: PmCartLine = field(default_factory=PmCartLine)
  uvw: PmCartLine = field(default_factory=PmCartLine)


# ============================================================
# 工具函数
# ============================================================

def LINEAR_INTERPOLATION(a: float, b: float, t: float) -> float:
  """线性插值: a + (b - a) * t"""
  return a + (b - a) * t


def pmMatRotX(angle: float) -> PmRotationMatrix:
  """绕X轴旋转矩阵"""
  c = math.cos(angle)
  s = math.sin(angle)
  mat = PmRotationMatrix()
  mat.s[0][0] = 1.0
  mat.s[0][1] = 0.0
  mat.s[0][2] = 0.0
  mat.s[1][0] = 0.0
  mat.s[1][1] = c
  mat.s[1][2] = -s
  mat.s[2][0] = 0.0
  mat.s[2][1] = s
  mat.s[2][2] = c
  return mat


def pmMatRotY(angle: float) -> PmRotationMatrix:
  """绕Y轴旋转矩阵"""
  c = math.cos(angle)
  s = math.sin(angle)
  mat = PmRotationMatrix()
  mat.s[0][0] = c
  mat.s[0][1] = 0.0
  mat.s[0][2] = s
  mat.s[1][0] = 0.0
  mat.s[1][1] = 1.0
  mat.s[1][2] = 0.0
  mat.s[2][0] = -s
  mat.s[2][1] = 0.0
  mat.s[2][2] = c
  return mat


def pmMatRotZ(angle: float) -> PmRotationMatrix:
  """绕Z轴旋转矩阵"""
  c = math.cos(angle)
  s = math.sin(angle)
  mat = PmRotationMatrix()
  mat.s[0][0] = c
  mat.s[0][1] = -s
  mat.s[0][2] = 0.0
  mat.s[1][0] = s
  mat.s[1][1] = c
  mat.s[1][2] = 0.0
  mat.s[2][0] = 0.0
  mat.s[2][1] = 0.0
  mat.s[2][2] = 1.0
  return mat


def pmMatMult(a: PmRotationMatrix, b: PmRotationMatrix) -> PmRotationMatrix:
  """3x3矩阵乘法: result = a * b"""
  result = PmRotationMatrix()
  for i in range(3):
    for j in range(3):
      result.s[i][j] = 0.0
      for k in range(3):
        result.s[i][j] += a.s[i][k] * b.s[k][j]
  return result


def pmMatVecMult(mat: PmRotationMatrix, vec: PmCartesian) -> PmCartesian:
  """矩阵乘向量: result = mat * vec"""
  return PmCartesian(
    mat.s[0][0] * vec.x + mat.s[0][1] * vec.y + mat.s[0][2] * vec.z,
    mat.s[1][0] * vec.x + mat.s[1][1] * vec.y + mat.s[1][2] * vec.z,
    mat.s[2][0] * vec.x + mat.s[2][1] * vec.y + mat.s[2][2] * vec.z
  )


def pmRpyMatConvert(rpy: PmRpy) -> PmRotationMatrix:
  """RPY转旋转矩阵 (ZYX顺序: 先yaw再pitch再roll)"""
  matZ = pmMatRotZ(rpy.y)
  matY = pmMatRotY(rpy.p)
  matX = pmMatRotX(rpy.r)
  # Z * Y * X
  return pmMatMult(matZ, pmMatMult(matY, matX))


def pmMatRpyConvert(mat: PmRotationMatrix) -> PmRpy:
  """旋转矩阵转RPY"""
  # 从旋转矩阵提取yaw、pitch、roll
  if math.isclose(mat.s[2][0], -1.0):
    # 万向锁情况
    r = math.atan2(-mat.s[0][1], mat.s[0][2])
    p = math.pi / 2.0
    y = 0.0
  elif math.isclose(mat.s[2][0], 1.0):
    r = math.atan2(mat.s[0][1], mat.s[0][2])
    p = -math.pi / 2.0
    y = 0.0
  else:
    p = math.atan2(-mat.s[2][0], math.sqrt(mat.s[2][1] ** 2 + mat.s[2][2] ** 2))
    r = math.atan2(mat.s[2][1], mat.s[2][2])
    y = math.atan2(mat.s[1][0], mat.s[0][0])
  return PmRpy(r=r, p=p, y=y)


def pmCartLineLength(line: PmCartLine) -> float:
  """直线段长度"""
  return line.start.distance_to(line.end)


def pmCircleLength(circle: PmCircle) -> float:
  """圆弧长度（含螺旋分量）"""
  angle_diff = circle.end_angle - circle.start_angle
  # 弧长 = 半径 * 角度差
  arc_length = abs(circle.radius * angle_diff)
  # 若有螺旋，加上垂直分量
  spiral = circle.spiral
  if spiral != 0.0:
    arc_length = math.sqrt(arc_length * arc_length + spiral * spiral)
  return arc_length


def pmLine9Init(start: EmcPose, end: EmcPose) -> PmLine9:
  """从两个EmcPose初始化9轴直线"""
  line = PmLine9()
  line.xyz.start = start.tran.copy()
  line.xyz.end = end.tran.copy()
  line.abc.start = PmCartesian(start.a, start.b, start.c)
  line.abc.end = PmCartesian(end.a, end.b, end.c)
  line.uvw.start = PmCartesian(start.u, start.v, start.w)
  line.uvw.end = PmCartesian(end.u, end.v, end.w)
  return line


def pmCircle9Init(
  center: EmcPose,
  start: EmcPose,
  end: EmcPose,
  radius: float,
  normal: PmCartesian,
  start_angle: float,
  end_angle: float,
  spiral: float
) -> PmCircle:
  """初始化9轴圆弧（取XYZ部分）"""
  circle = PmCircle()
  circle.center = center.tran.copy()
  circle.normal = normal.copy()
  circle.radius = radius
  circle.start_angle = start_angle
  circle.end_angle = end_angle
  circle.spiral = spiral
  return circle


def pmCartesianListToEmcPose(
  xyz: Optional[PmCartesian] = None,
  abc: Optional[PmCartesian] = None,
  uvw: Optional[PmCartesian] = None
) -> EmcPose:
  """从分量列表构建EmcPose"""
  pose = EmcPose()
  if xyz is not None:
    pose.tran = xyz.copy()
  if abc is not None:
    pose.a = abc.x
    pose.b = abc.y
    pose.c = abc.z
  if uvw is not None:
    pose.u = uvw.x
    pose.v = uvw.y
    pose.w = uvw.z
  return pose


def pmPoseDistance(a: EmcPose, b: EmcPose) -> float:
  """两个位姿间的XYZ距离"""
  return a.tran.distance_to(b.tran)


def pmPoseInverseSolve(start: EmcPose, end: EmcPose, t: float) -> EmcPose:
  """位姿线性插值"""
  result = EmcPose()
  result.tran = PmCartesian(
    LINEAR_INTERPOLATION(start.tran.x, end.tran.x, t),
    LINEAR_INTERPOLATION(start.tran.y, end.tran.y, t),
    LINEAR_INTERPOLATION(start.tran.z, end.tran.z, t)
  )
  result.a = LINEAR_INTERPOLATION(start.a, end.a, t)
  result.b = LINEAR_INTERPOLATION(start.b, end.b, t)
  result.c = LINEAR_INTERPOLATION(start.c, end.c, t)
  result.u = LINEAR_INTERPOLATION(start.u, end.u, t)
  result.v = LINEAR_INTERPOLATION(start.v, end.v, t)
  result.w = LINEAR_INTERPOLATION(start.w, end.w, t)
  return result
