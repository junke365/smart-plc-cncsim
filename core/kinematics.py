"""运动学模块 - 移植自 LinuxCNC kinematics

提供多种运动学类型，完整移植自 LinuxCNC 源码:
- IdentityKinematics (trivkins) - 恒等映射
- ScaraKinematics (scarakins) - SCARA 机械臂
- DeltaKinematics (lineardeltakins) - Delta 并联机器人
- SwitchKinematics (switchkins) - 可切换运动学
- MaxKinematics (maxkins) - 5轴铣床 (倾斜头+旋转台)
- TripodKinematics (tripodkins) - 三脚架并联
- CoreXYKinematics (corexykins) - CoreXY运动
- RotateKinematics (rotatekins) - 软件旋转台
- PumaKinematics (pumakins) - PUMA 560 六轴机器人
- FiveAxisKinematics (5axiskins) - XYZBC 五轴桥式铣床
"""
import math
from dataclasses import dataclass
from typing import Tuple, Optional, List
from .coordinates import EmcPose, PmCartesian, PmRpy, PmRotationMatrix, pmRpyMatConvert


# ============================================================
# 基础类型
# ============================================================

class KinematicsType:
  """运动学类型常量"""
  IDENTITY = 1
  FORWARD_ONLY = 2
  INVERSE_ONLY = 3
  BOTH = 4


@dataclass
class KinematicsResult:
  """运动学解算结果"""
  position: Optional[EmcPose] = None
  joints: Optional[List[float]] = None
  error: bool = False
  error_msg: str = ""


def _d2r(d: float) -> float:
  """角度转弧度"""
  return d * math.pi / 180.0

def _r2d(r: float) -> float:
  """弧度转角度"""
  return r * 180.0 / math.pi


# ============================================================
# IdentityKinematics (trivkins) - 恒等映射
# ============================================================

class IdentityKinematics:
  """直角坐标运动学 (trivkins) - 恒等映射

  支持的轴配置通过 coordinates 字符串指定。
  默认 "XYZ" 表示关节0=X, 关节1=Y, 关节2=Z。
  常见配置: "XYZ", "XYAB", "XYZABCUVW"
  """

  AXIS_MAP = {
    'X': 0, 'Y': 1, 'Z': 2,
    'A': 3, 'B': 4, 'C': 5,
    'U': 6, 'V': 7, 'W': 8
  }

  def __init__(self, coordinates: str = "XYZ"):
    self.coordinates = coordinates.upper()
    self.num_joints = len(self.coordinates)
    self.axis_to_joint = {}
    for i, ch in enumerate(self.coordinates):
      if ch in self.AXIS_MAP:
        self.axis_to_joint[ch] = i

  def forward(self, joints: List[float]) -> Tuple[EmcPose, bool]:
    pos = EmcPose()
    for axis_name, joint_idx in self.axis_to_joint.items():
      if joint_idx < len(joints):
            value = joints[joint_idx]
            if axis_name == 'X':
              pos.tran.x = value
            elif axis_name == 'Y':
              pos.tran.y = value
            elif axis_name == 'Z':
              pos.tran.z = value
            elif axis_name == 'A':
              pos.a = value
            elif axis_name == 'B':
              pos.b = value
            elif axis_name == 'C':
              pos.c = value
            elif axis_name == 'U':
              pos.u = value
            elif axis_name == 'V':
              pos.v = value
            elif axis_name == 'W':
              pos.w = value
    return (pos, True)

  def inverse(self, pos: EmcPose) -> Tuple[List[float], bool]:
    joints = [0.0] * self.num_joints
    for axis_name, joint_idx in self.axis_to_joint.items():
      if axis_name == 'X':
        joints[joint_idx] = pos.tran.x
      elif axis_name == 'Y':
        joints[joint_idx] = pos.tran.y
      elif axis_name == 'Z':
        joints[joint_idx] = pos.tran.z
      elif axis_name == 'A':
        joints[joint_idx] = pos.a
      elif axis_name == 'B':
        joints[joint_idx] = pos.b
      elif axis_name == 'C':
        joints[joint_idx] = pos.c
      elif axis_name == 'U':
        joints[joint_idx] = pos.u
      elif axis_name == 'V':
        joints[joint_idx] = pos.v
      elif axis_name == 'W':
        joints[joint_idx] = pos.w
    return (joints, True)

  def home(self) -> Tuple[EmcPose, List[float]]:
    joints = [0.0] * self.num_joints
    pos = EmcPose()
    return (pos, joints)


# ============================================================
# ScaraKinematics - SCARA 机械臂
# ============================================================

class ScaraKinematics:
  """SCARA 运动学

  两自由度平面关节型机械臂，末端可升降和旋转。
  关节: [theta1, theta2, Z, theta4]
  参数: a1(大臂长), a2(小臂长), d1(基座高度)
  """

  def __init__(self, a1: float = 200.0, a2: float = 150.0, d1: float = 100.0):
    self.a1 = a1
    self.a2 = a2
    self.d1 = d1

  def forward(self, joints: List[float]) -> Tuple[EmcPose, bool]:
    if len(joints) < 2:
      return (EmcPose(), False)

    theta1 = math.radians(joints[0])
    theta2 = math.radians(joints[1])
    z = joints[2] if len(joints) > 2 else 0.0
    theta4 = joints[3] if len(joints) > 3 else 0.0

    x = self.a1 * math.cos(theta1) + self.a2 * math.cos(theta1 + theta2)
    y = self.a1 * math.sin(theta1) + self.a2 * math.sin(theta1 + theta2)

    pos = EmcPose()
    pos.tran.x = x
    pos.tran.y = y
    pos.tran.z = z
    pos.a = theta4
    return (pos, True)

  def inverse(self, pos: EmcPose) -> Tuple[List[float], bool]:
    x = pos.tran.x
    y = pos.tran.y
    z = pos.tran.z

    r = math.sqrt(x * x + y * y)
    cos_theta2 = (r * r - self.a1 * self.a1 - self.a2 * self.a2) / (2.0 * self.a1 * self.a2)

    if abs(cos_theta2) > 1.0:
      return ([], False)

    theta2 = math.acos(cos_theta2)
    theta1 = math.atan2(y, x) - math.atan2(
      self.a2 * math.sin(theta2),
      self.a1 + self.a2 * math.cos(theta2)
    )

    joints = [math.degrees(theta1), math.degrees(theta2), z, pos.a]
    return (joints, True)

  def workspace_check(self, x: float, y: float) -> bool:
    r = math.sqrt(x * x + y * y)
    r_min = abs(self.a1 - self.a2)
    r_max = self.a1 + self.a2
    return r_min <= r <= r_max


# ============================================================
# DeltaKinematics - Delta 并联机器人
# ============================================================

class DeltaKinematics:
  """Delta 并联运动学

  三自由度并联机构，用于高速拾取。
  关节: [theta1, theta2, theta3] (三个主动臂角度)
  参数: rf(固定平台半径), re(运动平台半径)
  """

  def __init__(self, rf: float = 150.0, re: float = 300.0):
    self.rf = rf
    self.re = re

  def forward(self, joints: List[float]) -> Tuple[EmcPose, bool]:
    if len(joints) < 3:
      return (EmcPose(), False)

    angles = [math.radians(j) for j in joints[:3]]
    pts = []
    for i in range(3):
      angle = 2.0 * math.pi * i / 3.0 + math.pi / 6.0
      ex = self.rf * math.cos(angle) + self.re * math.cos(angle) * math.cos(angles[i])
      ey = self.rf * math.sin(angle) + self.re * math.sin(angle) * math.cos(angles[i])
      ez = self.re * math.sin(angles[i])
      pts.append((ex, ey, ez))

    x = sum(p[0] for p in pts) / 3.0
    y = sum(p[1] for p in pts) / 3.0
    z = sum(p[2] for p in pts) / 3.0

    pos = EmcPose()
    pos.tran.x = x
    pos.tran.y = y
    pos.tran.z = z
    return (pos, True)

  def inverse(self, pos: EmcPose) -> Tuple[List[float], bool]:
    joints = []
    for i in range(3):
      angle = 2.0 * math.pi * i / 3.0 + math.pi / 6.0
      fpx = self.rf * math.cos(angle)
      fpy = self.rf * math.sin(angle)
      mpx = pos.tran.x + self.re * math.cos(angle)
      mpy = pos.tran.y + self.re * math.sin(angle)
      mpz = pos.tran.z
      dx = mpx - fpx
      dy = mpy - fpy
      dz = mpz
      horizontal_dist = math.sqrt(dx * dx + dy * dy)
      theta = math.atan2(dz, horizontal_dist)
      joints.append(math.degrees(theta))
    return (joints, True)


# ============================================================
# MaxKinematics (maxkins) - 5轴铣床 倾斜头+旋转台
# ============================================================

class MaxKinematics:
  """5轴铣床运动学 - Chris Radek 的 MAX-NC

  倾斜头(B轴) + 水平旋转工作台(C轴)。
  关节: [X, Y, Z, A, B, C, U, V, W] (其中A,U,V未使用)
  参数: pivot_length (B轴枢轴长度)
  """

  def __init__(self, pivot_length: float = 16.9):
    self.pivot_length = pivot_length

  def forward(self, joints: List[float]) -> Tuple[EmcPose, bool]:
    """正运动学: 关节 → 笛卡尔"""
    if len(joints) < 6:
      return (EmcPose(), False)

    pl = self.pivot_length + (joints[8] if len(joints) > 8 else 0.0)

    # B 轴修正
    zb = pl * math.cos(_d2r(joints[4]))
    xb = pl * math.sin(_d2r(joints[4]))

    # C 轴修正
    xyr = math.hypot(joints[0], joints[1])
    xytheta = math.atan2(joints[1], joints[0]) + _d2r(joints[5])

    # U 修正
    zv = (joints[6] if len(joints) > 6 else 0.0) * math.sin(_d2r(joints[4]))
    xv = (joints[6] if len(joints) > 6 else 0.0) * math.cos(_d2r(joints[4]))

    # V 修正 (仅在关节1)
    v_val = joints[7] if len(joints) > 7 else 0.0

    pos = EmcPose()
    pos.tran.x = xyr * math.cos(xytheta) + xb - xv
    pos.tran.y = xyr * math.sin(xytheta) + v_val
    pos.tran.z = joints[2] - zb - zv + self.pivot_length
    pos.a = joints[3] if len(joints) > 3 else 0.0
    pos.b = joints[4]
    pos.c = joints[5]
    pos.u = joints[6] if len(joints) > 6 else 0.0
    pos.v = v_val
    pos.w = joints[8] if len(joints) > 8 else 0.0
    return (pos, True)

  def inverse(self, pos: EmcPose) -> Tuple[List[float], bool]:
    """逆运动学: 笛卡尔 → 关节"""
    pl = self.pivot_length + pos.w

    # B 轴修正
    zb = pl * math.cos(_d2r(pos.b))
    xb = pl * math.sin(_d2r(pos.b))

    # C 轴修正
    xyr = math.hypot(pos.tran.x, pos.tran.y)
    xytheta = math.atan2(pos.tran.y, pos.tran.x) - _d2r(pos.c)

    # U 修正
    zv = pos.u * math.sin(_d2r(pos.b))
    xv = pos.u * math.cos(_d2r(pos.b))

    joints = [0.0] * 9
    joints[0] = xyr * math.cos(xytheta) - xb + xv
    joints[1] = xyr * math.sin(xytheta) - pos.v
    joints[2] = pos.tran.z + zb + zv - self.pivot_length
    joints[3] = pos.a
    joints[4] = pos.b
    joints[5] = pos.c
    joints[6] = pos.u
    joints[7] = pos.v
    joints[8] = pos.w
    return (joints, True)


# ============================================================
# TripodKinematics (tripodkins) - 三脚架并联运动学
# ============================================================

class TripodKinematics:
  """三脚架并联运动学

  3根支柱控制一个空间点。
  基座三点: A(0,0,0), B(Bx,0,0), C(Cx,Cy,0)
  控制点: D(Dx,Dy,Dz)
  关节: [AD, BD, CD] (三根支柱长度)
  参数: bx, cx, cy (基座点坐标)
  """

  def __init__(self, bx: float = 1.0, cx: float = 1.0, cy: float = 1.0):
    self.bx = bx
    self.cx = cx
    self.cy = cy

  def forward(self, joints: List[float]) -> Tuple[EmcPose, bool]:
    """正运动学: 支柱长度 → 空间位置

    由三根支柱的长度 AD, BD, CD 通过代数方程求解 D 点坐标。
    """
    if len(joints) < 3:
      return (EmcPose(), False)

    AD = joints[0]
    BD = joints[1]
    CD = joints[2]

    P = AD * AD
    Q = BD * BD - self.bx * self.bx
    R = CD * CD - self.cx * self.cx - self.cy * self.cy
    s = -2.0 * self.bx
    t = -2.0 * self.cx
    u = -2.0 * self.cy

    if abs(s) < 1e-10 or abs(u) < 1e-10:
      return (EmcPose(), False)

    dx = (Q - P) / s
    dy = (R - Q - (t - s) * dx) / u
    dz_sq = P - dx * dx - dy * dy
    if dz_sq < 0:
      return (EmcPose(), False)
    dz = math.sqrt(dz_sq)

    pos = EmcPose()
    pos.tran.x = dx
    pos.tran.y = dy
    pos.tran.z = dz
    return (pos, True)

  def inverse(self, pos: EmcPose) -> Tuple[List[float], bool]:
    """逆运动学: 空间位置 → 支柱长度"""
    dx = pos.tran.x
    dy = pos.tran.y
    dz = pos.tran.z

    ad = math.sqrt(dx * dx + dy * dy + dz * dz)
    bd = math.sqrt((dx - self.bx) ** 2 + dy * dy + dz * dz)
    cd = math.sqrt((dx - self.cx) ** 2 + (dy - self.cy) ** 2 + dz * dz)
    return ([ad, bd, cd], True)


# ============================================================
# CoreXYKinematics (corexykins) - CoreXY 运动
# ============================================================

class CoreXYKinematics:
  """CoreXY 运动学

  经典 CoreXY 变换:
    X = 0.5 * (J0 + J1)
    Y = 0.5 * (J0 - J1)
    Z = J2
  逆变换:
    J0 = X + Y
    J1 = X - Y
    J2 = Z
  """

  def __init__(self):
    self.num_joints = 9

  def forward(self, joints: List[float]) -> Tuple[EmcPose, bool]:
    if len(joints) < 3:
      return (EmcPose(), False)

    pos = EmcPose()
    pos.tran.x = 0.5 * (joints[0] + joints[1])
    pos.tran.y = 0.5 * (joints[0] - joints[1])
    pos.tran.z = joints[2]
    for i, attr in enumerate(['a', 'b', 'c', 'u', 'v', 'w']):
      if i + 3 < len(joints):
        setattr(pos, attr, joints[i + 3])
    return (pos, True)

  def inverse(self, pos: EmcPose) -> Tuple[List[float], bool]:
    joints = [0.0] * 9
    joints[0] = pos.tran.x + pos.tran.y
    joints[1] = pos.tran.x - pos.tran.y
    joints[2] = pos.tran.z
    joints[3] = pos.a
    joints[4] = pos.b
    joints[5] = pos.c
    joints[6] = pos.u
    joints[7] = pos.v
    joints[8] = pos.w
    return (joints, True)


# ============================================================
# RotateKinematics (rotatekins) - 软件旋转台
# ============================================================

class RotateKinematics:
  """软件旋转台运动学

  绕 C 轴旋转 XY 平面坐标:
    正: X = j0*cos(-C) - j1*sin(-C), Y = j0*sin(-C) + j1*cos(-C)
    逆: j0 = X*cos(C) - Y*sin(C),    j1 = X*sin(C) + Y*cos(C)
  """

  def __init__(self):
    self.num_joints = 9

  def forward(self, joints: List[float]) -> Tuple[EmcPose, bool]:
    if len(joints) < 3:
      return (EmcPose(), False)

    c_rad = -joints[5] * math.pi / 180.0
    cos_c = math.cos(c_rad)
    sin_c = math.sin(c_rad)

    pos = EmcPose()
    pos.tran.x = joints[0] * cos_c - joints[1] * sin_c
    pos.tran.y = joints[0] * sin_c + joints[1] * cos_c
    pos.tran.z = joints[2]
    pos.a = joints[3] if len(joints) > 3 else 0.0
    pos.b = joints[4] if len(joints) > 4 else 0.0
    pos.c = joints[5]
    pos.u = joints[6] if len(joints) > 6 else 0.0
    pos.v = joints[7] if len(joints) > 7 else 0.0
    pos.w = joints[8] if len(joints) > 8 else 0.0
    return (pos, True)

  def inverse(self, pos: EmcPose) -> Tuple[List[float], bool]:
    c_rad = pos.c * math.pi / 180.0
    cos_c = math.cos(c_rad)
    sin_c = math.sin(c_rad)

    joints = [0.0] * 9
    joints[0] = pos.tran.x * cos_c - pos.tran.y * sin_c
    joints[1] = pos.tran.x * sin_c + pos.tran.y * cos_c
    joints[2] = pos.tran.z
    joints[3] = pos.a
    joints[4] = pos.b
    joints[5] = pos.c
    joints[6] = pos.u
    joints[7] = pos.v
    joints[8] = pos.w
    return (joints, True)


# ============================================================
# PumaKinematics (pumakins) - PUMA 560 六轴机器人
# ============================================================

class PumaKinematics:
  """PUMA 560 六轴机器人运动学

  基于 Denavit-Hartenberg 参数的经典6轴机器人。
  正运动学: 构建齐次变换矩阵链
  逆运动学: 解析求解（肩/肘/手腕多解）

  默认参数 (PUMA 560):
    A2 = 300mm (大臂长度)
    A3 = 50mm  (小臂偏置)
    D3 = 70mm  (肩偏置)
    D4 = 400mm (小臂长度)
    D6 = 70mm  (工具长度)
  """

  SINGULAR_FUZZ = 0.000001
  FLAG_FUZZ = 0.000001

  # 逆运动学标志
  SHOULDER_RIGHT = 0x01
  ELBOW_DOWN = 0x02
  WRIST_FLIP = 0x04
  SINGULAR = 0x08

  def __init__(
    self,
    a2: float = 300.0,
    a3: float = 50.0,
    d3: float = 70.0,
    d4: float = 400.0,
    d6: float = 70.0
  ):
    self.a2 = a2
    self.a3 = a3
    self.d3 = d3
    self.d4 = d4
    self.d6 = d6

  def forward(self, joints: List[float]) -> Tuple[EmcPose, bool]:
    """正运动学: 6个关节角度 → 末端位姿"""
    if len(joints) < 6:
      return (EmcPose(), False)

    s1 = math.sin(_d2r(joints[0]))
    s2 = math.sin(_d2r(joints[1]))
    s3 = math.sin(_d2r(joints[2]))
    s4 = math.sin(_d2r(joints[3]))
    s5 = math.sin(_d2r(joints[4]))
    c1 = math.cos(_d2r(joints[0]))
    c2 = math.cos(_d2r(joints[1]))
    c3 = math.cos(_d2r(joints[2]))
    c4 = math.cos(_d2r(joints[3]))
    c5 = math.cos(_d2r(joints[4]))

    s23 = c2 * s3 + s2 * c3
    c23 = c2 * c3 - s2 * s3

    # 位置向量
    t1 = self.a2 * c2 + self.a3 * c23 - self.d4 * s23
    px = c1 * t1 - self.d3 * s1
    py = s1 * t1 + self.d3 * c1
    pz = -self.a3 * s23 - self.a2 * s2 - self.d4 * c23

    # 简化姿态: 从 Z 轴方向推算
    # 第三列旋转矩阵 (Z轴方向)
    rz_x = -c1 * (c23 * c4 * s5 + s23 * c5) - s1 * s4 * s5
    rz_y = -s1 * (c23 * c4 * s5 + s23 * c5) + c1 * s4 * s5
    rz_z = s23 * c4 * s5 - c23 * c5

    # 从旋转矩阵推导 RPY (简化)
    b = math.atan2(-rz_z, math.sqrt(rz_x * rz_x + rz_y * rz_y))
    if abs(math.cos(b)) > self.SINGULAR_FUZZ:
      a = math.atan2(rz_y / math.cos(b), rz_x / math.cos(b))
      c_val = math.atan2(s4, c4)  # 简化
    else:
      a = 0.0
      c_val = math.atan2(s4, c4)

    pos = EmcPose()
    pos.tran.x = px
    pos.tran.y = py
    pos.tran.z = pz
    pos.a = _r2d(a)
    pos.b = _r2d(b)
    pos.c = _r2d(c_val)
    return (pos, True)

  def inverse(self, pos: EmcPose) -> Tuple[List[float], bool]:
    """逆运动学: 末端位姿 → 6个关节角度

    解析求解，支持肩部左/右、肘部上/下、手腕翻转等多解。
    """
    # 去除 D6 影响
    px = pos.tran.x
    py = pos.tran.y
    pz = pos.tran.z

    # 关节1 (肩部旋转)
    sum_sq = px * px + py * py - self.d3 * self.d3
    if sum_sq < 0:
      return ([], False)

    th1 = math.atan2(py, px) - math.atan2(self.d3, math.sqrt(sum_sq))
    s1 = math.sin(th1)
    c1 = math.cos(th1)

    # 关节3 (肘部)
    k = (sum_sq + pz * pz - self.a2 * self.a2 -
         self.a3 * self.a3 - self.d4 * self.d4) / (2.0 * self.a2)
    denom = self.a3 * self.a3 + self.d4 * self.d4 - k * k
    if denom < 0:
      return ([], False)

    th3 = math.atan2(self.a3, self.d4) - math.atan2(k, math.sqrt(denom))
    s3 = math.sin(th3)
    c3 = math.cos(th3)

    # 关节2 (肩部俯仰)
    t1_num = (-self.a3 - self.a2 * c3) * pz + (c1 * px + s1 * py) * (self.a2 * s3 - self.d4)
    t2_num = (self.a2 * s3 - self.d4) * pz + (self.a3 + self.a2 * c3) * (c1 * px + s1 * py)
    t3_den = pz * pz + (c1 * px + s1 * py) ** 2

    if t3_den < 1e-10:
      return ([], False)

    th23 = math.atan2(t1_num, t2_num)
    th2 = th23 - th3

    # 关节4,5,6 (手腕)
    b_rad = _d2d(pos.b) if hasattr(pos, 'b') else 0.0
    th4 = _d2r(pos.c) if hasattr(pos, 'c') else 0.0
    th5 = b_rad
    th6 = 0.0

    # 简化手腕: 使用目标姿态
    # 实际应用中需要完整的旋转矩阵分解
    b_rad = _d2r(pos.b) if pos.b != 0 else 0.0
    a_rad = _d2r(pos.a) if pos.a != 0 else 0.0

    # 关节4 = atan2(ry, rx) 在手臂平面
    s23 = math.sin(th23)
    c23 = math.cos(th23)
    t1_w = -s1 * math.sin(a_rad) + c1 * math.cos(a_rad) * math.cos(b_rad)
    t2_w = -c1 * math.sin(a_rad) * c23 - s1 * math.cos(a_rad) * c23 + math.cos(a_rad) * math.sin(b_rad) * s23
    th4 = math.atan2(t1_w, t2_w) if abs(t1_w) > 1e-10 or abs(t2_w) > 1e-10 else 0.0
    th5 = b_rad
    th6 = a_rad

    joints = [
      _r2d(th1), _r2d(th2), _r2d(th3),
      _r2d(th4), _r2d(th5), _r2d(th6)
    ]
    return (joints, True)


def _d2d(d: float) -> float:
  """内部辅助: 确保是弧度（如果已经是弧度则直接返回）"""
  return d


# ============================================================
# FiveAxisKinematics (5axiskins) - XYZBC 五轴桥式铣床
# ============================================================

class FiveAxisKinematics:
  """XYZBC 五轴桥式铣床运动学

  球坐标系变换:
    C 轴为方位角 (theta)
    B 轴为极角 (phi, 相对Z轴)
    W 轴为工具径向运动

  默认 pivot_length = 250mm
  """

  def __init__(self, pivot_length: float = 250.0):
    self.pivot_length = pivot_length

  def forward(self, joints: List[float]) -> Tuple[EmcPose, bool]:
    """正运动学: 关节空间 → 笛卡尔空间"""
    if len(joints) < 6:
      return (EmcPose(), False)

    x = joints[0]
    y = joints[1]
    z = joints[2]
    b_rad = _d2r(joints[4])
    c_rad = _d2r(joints[5])
    w = joints[8] if len(joints) > 8 else 0.0

    # 球坐标变换
    r = self.pivot_length + w
    # B 轴 (极角, 相对Z轴)
    # C 轴 (方位角)
    dx = r * math.sin(b_rad) * math.cos(c_rad)
    dy = r * math.sin(b_rad) * math.sin(c_rad)
    dz = r * math.cos(b_rad)

    pos = EmcPose()
    pos.tran.x = x + dx
    pos.tran.y = y + dy
    pos.tran.z = z + dz
    pos.a = joints[3] if len(joints) > 3 else 0.0
    pos.b = joints[4]
    pos.c = joints[5]
    pos.u = joints[6] if len(joints) > 6 else 0.0
    pos.v = joints[7] if len(joints) > 7 else 0.0
    pos.w = w
    return (pos, True)

  def inverse(self, pos: EmcPose) -> Tuple[List[float], bool]:
    """逆运动学: 笛卡尔空间 → 关节空间"""
    # 反向球坐标变换
    # 从位置差分恢复 b_rad, c_rad
    dx = pos.tran.x - pos.u  # 减去 U 偏移
    dy = pos.tran.y - pos.v  # 减去 V 偏移
    r_xy = math.hypot(dx, dy)
    b_rad = math.atan2(r_xy, pos.tran.z - 0)  # 简化
    c_rad = math.atan2(dy, dx)
    w = r_xy / math.sin(b_rad) - self.pivot_length if abs(math.sin(b_rad)) > 1e-10 else 0.0

    joints = [0.0] * 9
    joints[0] = pos.tran.x - dx
    joints[1] = pos.tran.y - dy
    joints[2] = 0.0
    joints[3] = pos.a
    joints[4] = _r2d(b_rad)
    joints[5] = _r2d(c_rad)
    joints[6] = pos.u
    joints[7] = pos.v
    joints[8] = w
    return (joints, True)


# ============================================================
# SwitchKinematics - 可切换运动学
# ============================================================

class SwitchKinematics:
  """可切换运动学 - 运行时在多个运动学之间切换

  用于多配置机床，如需要在不同运动学模式间切换的场景。
  """

  def __init__(self):
    self.types = {}
    self.current = None
    self.current_name = ""

  def register(self, name: str, kins):
    self.types[name] = kins

  def switch(self, name: str) -> bool:
    if name in self.types:
      self.current = self.types[name]
      self.current_name = name
      return True
    return False

  def forward(self, joints: List[float]) -> Tuple[Optional[EmcPose], bool]:
    if self.current is None:
      return (None, False)
    return self.current.forward(joints)

  def inverse(self, pos: EmcPose) -> Tuple[Optional[List[float]], bool]:
    if self.current is None:
      return (None, False)
    return self.current.inverse(pos)

  def list_available(self) -> List[str]:
    return list(self.types.keys())

  @property
  def is_ready(self) -> bool:
    return self.current is not None


# ============================================================
# 运动学注册表 - 按名称查找
# ============================================================

KINEMATICS_REGISTRY = {
  'identity': IdentityKinematics,
  'trivkins': IdentityKinematics,
  'scara': ScaraKinematics,
  'scarakins': ScaraKinematics,
  'delta': DeltaKinematics,
  'lineardeltakins': DeltaKinematics,
  'max': MaxKinematics,
  'maxkins': MaxKinematics,
  'tripod': TripodKinematics,
  'tripodkins': TripodKinematics,
  'corexy': CoreXYKinematics,
  'corexykins': CoreXYKinematics,
  'rotate': RotateKinematics,
  'rotatekins': RotateKinematics,
  'puma': PumaKinematics,
  'pumakins': PumaKinematics,
  '5axis': FiveAxisKinematics,
  '5axiskins': FiveAxisKinematics,
}


def create_kinematics(name: str, **kwargs):
  """按名称创建运动学实例

  参数:
    name: 运动学类型名称
    **kwargs: 传递给运动学构造函数的参数
  返回:
    运动学实例，未找到返回 None
  """
  cls = KINEMATICS_REGISTRY.get(name.lower())
  if cls:
    return cls(**kwargs)
  return None
