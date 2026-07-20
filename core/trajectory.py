"""轨迹规划器 - 移植自 LinuxCNC tp 模块

简化掉弧混合(blend)，保留梯形速度规划。
独立可运行的 Python 模块。
"""
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from enum import IntEnum
from .coordinates import EmcPose, PmCartesian, PmCircle, PmCartLine, LINEAR_INTERPOLATION


# ============================================================
# 核心枚举
# ============================================================

class MotionType(IntEnum):
  """运动类型"""
  LINEAR = 1
  CIRCULAR = 2


class TermCond(IntEnum):
  """终止条件"""
  STOP = 0      # 停止（末端速度为0）
  EXACT = 1     # 精确到位
  PARABOLIC = 2  # 抛物线混合
  TANGENT = 3   # 切线混合


# ============================================================
# TC (轨迹段) 数据结构
# ============================================================

@dataclass
class TC:
  """轨迹段 - 运动的最小单位

  移植自 LinuxCNC 的 TC_STRUCT，每段代表一条 G 代码运动。
  """
  # 几何描述
  motion_type: MotionType = MotionType.LINEAR
  start_x: float = 0.0
  start_y: float = 0.0
  start_z: float = 0.0
  end_x: float = 0.0
  end_y: float = 0.0
  end_z: float = 0.0

  # 圆弧参数
  center_x: float = 0.0
  center_y: float = 0.0
  center_z: float = 0.0
  normal_z: float = 1.0
  turn: int = 1
  _radius: float = 0.0
  _start_angle: float = 0.0
  _end_angle: float = 0.0

  # 速度/加速度
  reqvel: float = 0.0       # 请求速度 (F值)
  target_vel: float = 0.0   # 目标速度
  maxvel: float = 0.0       # 最大允许速度
  currentvel: float = 0.0   # 当前速度
  finalvel: float = 0.0     # 末端速度
  maxaccel: float = 0.0     # 最大加速度

  # 距离
  target: float = 0.0       # 总行程
  progress: float = 0.0     # 当前进度 (0..target)

  # 终止条件
  term_cond: TermCond = TermCond.STOP

  # 状态标志
  active: bool = True
  remove: bool = False
  id: int = 0

  # 同步DIO
  sync_dio_out: List[int] = field(default_factory=list)

  @property
  def done(self) -> bool:
    """是否已完成"""
    return self.progress >= self.target

  @property
  def pos_x(self) -> float:
    """当前位置 X"""
    if self.target <= 0:
      return self.end_x
    t = self.progress / self.target
    if self.motion_type == MotionType.LINEAR:
      return self.start_x + (self.end_x - self.start_x) * t
    else:
      return self._arc_pos_x(t)

  @property
  def pos_y(self) -> float:
    """当前位置 Y"""
    if self.target <= 0:
      return self.end_y
    t = self.progress / self.target
    if self.motion_type == MotionType.LINEAR:
      return self.start_y + (self.end_y - self.start_y) * t
    else:
      return self._arc_pos_y(t)

  @property
  def pos_z(self) -> float:
    """当前位置 Z"""
    if self.target <= 0:
      return self.end_z
    t = self.progress / self.target
    if self.motion_type == MotionType.LINEAR:
      return self.start_z + (self.end_z - self.start_z) * t
    else:
      return self.start_z + (self.end_z - self.start_z) * t

  def _arc_pos_x(self, t: float) -> float:
    """圆弧插补 X 位置"""
    angle = self._start_angle + (self._end_angle - self._start_angle) * t
    return self.center_x + self._radius * math.cos(angle)

  def _arc_pos_y(self, t: float) -> float:
    """圆弧插补 Y 位置"""
    angle = self._start_angle + (self._end_angle - self._start_angle) * t
    return self.center_y + self._radius * math.sin(angle)


# ============================================================
# TP (轨迹规划器) 类
# ============================================================

class TrajectoryPlanner:
  """轨迹规划器 - 移植自 LinuxCNC TP_STRUCT

  核心职责:
  1. 维护轨迹段队列
  2. 每个伺服周期执行梯形速度规划
  3. 计算插补位置输出给运动学/驱动层
  """

  def __init__(self):
    self.queue: List[TC] = []
    self.cycle_time: float = 0.001  # 1ms 伺服周期
    self.maxvel: float = 5000.0     # mm/min
    self.maxaccel: float = 50000.0  # mm/s²
    self.current_x: float = 0.0
    self.current_y: float = 0.0
    self.current_z: float = 0.0
    self.done: bool = True
    self.paused: bool = False
    self.aborting: bool = False
    self.tolerance: float = 0.0
    self.next_id: int = 1
    self._queue_max: int = 64

  # ----------------------------------------------------------
  # 队列管理
  # ----------------------------------------------------------

  def add_line(
    self, end_x: float, end_y: float, end_z: float,
    vel: float, acc: float, motion_type: MotionType = MotionType.LINEAR
  ) -> Optional[TC]:
    """添加直线运动段

    参数:
      end_x, end_y, end_z: 目标位置 (mm)
      vel: 请求速度 (mm/min)
      acc: 加速度 (mm/s²)
    返回:
      创建的 TC 段，或队列满时返回 None
    """
    if len(self.queue) >= self._queue_max:
      return None

    tc = TC()
    tc.id = self.next_id
    self.next_id += 1
    tc.motion_type = motion_type

    # 设置起点（取上一段终点或当前位置）
    if self.queue:
      last = self.queue[-1]
      tc.start_x = last.end_x
      tc.start_y = last.end_y
      tc.start_z = last.end_z
    else:
      tc.start_x = self.current_x
      tc.start_y = self.current_y
      tc.start_z = self.current_z

    tc.end_x = end_x
    tc.end_y = end_y
    tc.end_z = end_z

    # 计算行程长度
    dx = tc.end_x - tc.start_x
    dy = tc.end_y - tc.start_y
    dz = tc.end_z - tc.start_z
    tc.target = math.sqrt(dx * dx + dy * dy + dz * dz)

    # 速度参数
    tc.reqvel = vel
    tc.target_vel = min(vel, self.maxvel)
    tc.maxvel = self.maxvel
    tc.maxaccel = acc if acc > 0 else self.maxaccel

    # 默认终止条件: 停止
    tc.term_cond = TermCond.STOP
    tc.active = True

    self.queue.append(tc)

    # 执行前瞻优化
    self._optimize()

    return tc

  def add_arc(
    self, end_x: float, end_y: float, end_z: float,
    center_x: float, center_y: float, center_z: float,
    turn: int, vel: float, acc: float
  ) -> Optional[TC]:
    """添加圆弧运动段

    参数:
      end_x, end_y, end_z: 目标位置 (mm)
      center_x, center_y, center_z: 圆心 (mm)
      turn: 旋转方向 (1=逆时针, -1=顺时针)
      vel: 请求速度 (mm/min)
      acc: 加速度 (mm/s²)
    返回:
      创建的 TC 段
    """
    if len(self.queue) >= self._queue_max:
      return None

    tc = TC()
    tc.id = self.next_id
    self.next_id += 1
    tc.motion_type = MotionType.CIRCULAR

    # 起点
    if self.queue:
      last = self.queue[-1]
      tc.start_x = last.end_x
      tc.start_y = last.end_y
      tc.start_z = last.end_z
    else:
      tc.start_x = self.current_x
      tc.start_y = self.current_y
      tc.start_z = self.current_z

    tc.end_x = end_x
    tc.end_y = end_y
    tc.end_z = end_z
    tc.center_x = center_x
    tc.center_y = center_y
    tc.center_z = center_z
    tc.turn = turn

    # 计算半径
    dx_s = tc.start_x - center_x
    dy_s = tc.start_y - center_y
    tc._radius = math.sqrt(dx_s * dx_s + dy_s * dy_s)

    # 计算起始和终止角度
    tc._start_angle = math.atan2(tc.start_y - center_y, tc.start_x - center_x)
    tc._end_angle = math.atan2(end_y - center_y, end_x - center_x)

    # 修正角度差以匹配 turn 方向
    angle_diff = tc._end_angle - tc._start_angle
    if turn > 0:
      # 逆时针: 确保角度差为正
      while angle_diff <= 0:
        angle_diff += 2 * math.pi
      while angle_diff > 2 * math.pi:
        angle_diff -= 2 * math.pi
    else:
      # 顺时针: 确保角度差为负
      while angle_diff >= 0:
        angle_diff -= 2 * math.pi
      while angle_diff < -2 * math.pi:
        angle_diff += 2 * math.pi
    tc._end_angle = tc._start_angle + angle_diff

    # 行程 = 弧长
    tc.target = abs(tc._radius * angle_diff)

    # 速度参数
    tc.reqvel = vel
    tc.target_vel = min(vel, self.maxvel)
    tc.maxvel = self.maxvel
    tc.maxaccel = acc if acc > 0 else self.maxaccel

    tc.term_cond = TermCond.STOP
    tc.active = True

    self.queue.append(tc)

    self._optimize()

    return tc

  # ----------------------------------------------------------
  # 核心周期
  # ----------------------------------------------------------

  def run_cycle(self, servo_period: float) -> Optional[dict]:
    """运行一个周期 - 返回当前运动命令或 None

    这是轨迹规划器的核心，每个伺服周期(1ms)调用一次。
    返回: {'x': float, 'y': float, 'z': float, 'vel': float, 'done': bool}
    """
    if self.paused or self.aborting:
      return None

    # 无队列则返回当前位置
    if not self.queue:
      self.done = True
      return {
        'x': self.current_x,
        'y': self.current_y,
        'z': self.current_z,
        'vel': 0.0,
        'done': True,
        'distance_to_go': 0.0
      }

    # 取出队首 TC
    tc = self.queue[0]

    # 梯形速度规划
    self._trapezoidal_accel(tc, servo_period)

    # 计算插补位置
    pos_x = tc.pos_x
    pos_y = tc.pos_y
    pos_z = tc.pos_z

    # 更新当前全局位置
    self.current_x = pos_x
    self.current_y = pos_y
    self.current_z = pos_z

    result = {
      'x': pos_x,
      'y': pos_y,
      'z': pos_z,
      'vel': tc.currentvel,
      'done': tc.done,
      'distance_to_go': tc.target - tc.progress
    }

    # TC 完成，弹出队列
    if tc.done:
      self.queue.pop(0)
      # 如果队列空了，本次运动完成
      if not self.queue:
        self.done = True
      result['done'] = True

    return result

  # ----------------------------------------------------------
  # 速度规划
  # ----------------------------------------------------------

  def _trapezoidal_accel(self, tc: TC, dt: float):
    """梯形速度规划

    移植自 tp.c 的 tpCalculateTrapezoidalAccel。
    每个伺服周期计算一次，更新 tc 的速度和进度。

    算法:
    1. 计算加速极限: v += a * dt
    2. 计算减速极限: v = sqrt(vf² + 2*a*d)
    3. 取三者最小值 (加速、减速、目标速度)
    4. 计算位移并更新进度
    """
    dist_to_go = tc.target - tc.progress
    if dist_to_go <= 0:
      tc.currentvel = 0
      tc.progress = tc.target
      return

    # 末端速度 (最终速度)
    vf = tc.finalvel

    # 加速极限: 当前速度 + 加速度 * 时间
    vel_accel_limit = tc.currentvel + tc.maxaccel * dt

    # 减速极限: 运动学公式 v² = vf² + 2*a*d
    if dist_to_go > 0:
      vel_decel_limit = math.sqrt(vf * vf + 2.0 * tc.maxaccel * dist_to_go)
    else:
      vel_decel_limit = vf

    # 取最小值: 加速上限、减速上限、目标速度
    newvel = min(vel_accel_limit, vel_decel_limit, tc.target_vel)
    newvel = max(0.0, newvel)

    # 位移 = 平均速度 * 时间 (梯形面积)
    displacement = (tc.currentvel + newvel) * 0.5 * dt

    # 不能超过剩余距离
    if displacement > dist_to_go:
      displacement = dist_to_go

    # 更新状态
    tc.progress += displacement
    tc.currentvel = newvel

  # ----------------------------------------------------------
  # 前瞻优化
  # ----------------------------------------------------------

  def _optimize(self):
    """逆向速度优化（涨潮算法简化版）

    从队列尾部向前遍历，确保相邻段的速度过渡平滑。
    如果后一段要求停车（STOP），前一段的末端速度也需要减到0。

    移植自 tpRunOptimization 的简化实现。
    """
    if len(self.queue) < 2:
      return

    # 从队列尾部向前遍历
    for i in range(len(self.queue) - 1, 0, -1):
      tc = self.queue[i]
      prev_tc = self.queue[i - 1]

      # 如果当前段以0速度终止，前一段末端也需要减速到0
      if tc.term_cond == TermCond.STOP:
        tc.finalvel = 0.0

      # 计算前一段的最大安全末端速度
      # 用公式: v² = 2 * a * d (从末端速度减速到0所需距离)
      # prev_tc 的末端速度必须满足:
      # 从 prev_tc.finalvel 减速到 tc.finalvel 不会超过交界处
      dist = prev_tc.target - prev_tc.progress
      if dist <= 0:
        dist = prev_tc.target

      # 前一段允许的最大末端速度
      # 考虑到前一段末端需要能过渡到后一段的初始速度
      if prev_tc.target > 0:
        max_v = math.sqrt(2.0 * prev_tc.maxaccel * dist * 0.5)
        prev_tc.finalvel = min(prev_tc.finalvel, max_v, prev_tc.target_vel)

  # ----------------------------------------------------------
  # 控制接口
  # ----------------------------------------------------------

  def pause(self):
    """暂停运动"""
    self.paused = True

  def resume(self):
    """恢复运动"""
    self.paused = False

  def abort(self):
    """中止运动 - 清空队列"""
    self.aborting = True
    self.queue.clear()

  def clear(self):
    """清空队列并重置状态"""
    self.queue.clear()
    self.done = True
    self.aborting = False
    self.paused = False

  def reset(self):
    """完全重置规划器"""
    self.clear()
    self.current_x = 0.0
    self.current_y = 0.0
    self.current_z = 0.0
    self.next_id = 1

  # ----------------------------------------------------------
  # 查询接口
  # ----------------------------------------------------------

  @property
  def queue_length(self) -> int:
    """当前队列长度"""
    return len(self.queue)

  @property
  def current_vel(self) -> float:
    """当前实际速度"""
    if self.queue:
      return self.queue[0].currentvel
    return 0.0

  @property
  def current_tc(self) -> Optional[TC]:
    """当前正在执行的轨迹段"""
    if self.queue:
      return self.queue[0]
    return None

  def get_position(self) -> Tuple[float, float, float]:
    """获取当前位置"""
    return (self.current_x, self.current_y, self.current_z)

  def get_pose(self) -> EmcPose:
    """获取当前位姿"""
    return EmcPose(tran=PmCartesian(self.current_x, self.current_y, self.current_z))

  def is_idle(self) -> bool:
    """是否空闲（无运动）"""
    return self.done and not self.paused and not self.aborting
