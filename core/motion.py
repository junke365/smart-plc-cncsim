"""运动控制器 - 移植自 LinuxCNC motion 模块

连接解释器（通过CanonCommand输出）和轨迹规划器（TrajectoryPlanner）。
工作流程:
1. 解释器执行G代码，输出规范命令到CanonCommand
2. 本控制器读取CanonCommand，转换为轨迹段加入TP
3. 按周期运行TP，输出位置指令
4. 通过运动学解算关节位置
5. 模拟伺服更新
"""
import time
import threading
from dataclasses import dataclass, field
from typing import List, Optional, Callable, Dict, Any, Tuple
from enum import Enum, IntEnum
from .canon import CanonCommand
from .trajectory import TrajectoryPlanner, MotionType as TrajMotionType
from .kinematics import IdentityKinematics, create_kinematics
from .coordinates import EmcPose, PmCartesian


# ============================================================
# 枚举类型
# ============================================================

class MotionState(Enum):
  """运动控制器状态 - 对应 LinuxCNC emcmot_motion_state_t"""
  DISABLED = 0
  FREE = 1
  COORD = 2
  TELEOP = 3


class MachineState(Enum):
  """机床电源状态"""
  ESTOP = 0
  OFF = 1
  ON = 2


class MotionCmdType(IntEnum):
  """运动命令类型 - 控制器层面的分类

  用于 EmcmotStatus.motion_type 字段，区分当前正在执行的运动类型。
  TRAVERSE=0 对应 G0，LINEAR=1 对应 G1，CIRCULAR=2 对应 G2/G3。
  """
  TRAVERSE = 0
  LINEAR = 1
  CIRCULAR = 2
  RIGID_TAP = 3


# ============================================================
# 数据类
# ============================================================

@dataclass
class JointStatus:
  """关节状态 - 对应 LinuxCNC emcmot_joint_status_t"""
  index: int = 0
  position: float = 0.0
  velocity: float = 0.0
  acceleration: float = 0.0
  pos_cmd: float = 0.0
  pos_fb: float = 0.0
  vel_cmd: float = 0.0
  ferror: float = 0.0
  homed: bool = False
  homing: bool = False
  min_limit: float = -1000.0
  max_limit: float = 1000.0
  min_ferror: float = 0.1
  max_ferror: float = 1.0
  backlash: float = 0.0


@dataclass
class SpindleStatus:
  """主轴状态 - 对应 LinuxCNC emcmot_spindle_status_t"""
  speed: float = 0.0
  state: int = 0
  scale: float = 1.0
  at_speed: bool = True
  direction: int = 0
  brake: bool = False


@dataclass
class MotionConfig:
  """运动配置 - 对应 LinuxCNC emcmot_config_t"""
  num_joints: int = 3
  num_spindles: int = 1
  traj_cycle_time: float = 0.001
  servo_cycle_time: float = 0.001
  max_velocity: float = 5000.0
  max_acceleration: float = 50000.0
  max_jog_velocity: float = 1000.0
  default_linear_velocity: float = 1000.0
  default_rapid_velocity: float = 3000.0
  coordinates: str = "XYZ"
  axis_names: list = field(default_factory=lambda: ["X", "Y", "Z"])
  kinematics_name: str = "identity"
  queue_max: int = 64


@dataclass
class EmcmotStatus:
  """运动控制器完整状态 - 对应 LinuxCNC emcmot_status_t"""
  head: int = 0
  tail: int = 0
  motion_state: MotionState = MotionState.DISABLED
  carte_pos_cmd: EmcPose = field(default_factory=EmcPose)
  carte_pos_fb: EmcPose = field(default_factory=EmcPose)
  joints: List[JointStatus] = field(default_factory=list)
  spindles: List[SpindleStatus] = field(default_factory=list)
  vel: float = 0.0
  acc: float = 0.0
  motion_type: int = 0
  distance_to_go: float = 0.0
  current_vel: float = 0.0
  requested_vel: float = 0.0
  paused: bool = False
  queue_full: bool = False
  tcq_len: int = 0
  done: bool = True
  feed_scale: float = 1.0
  rapid_scale: float = 1.0
  spindle_scale: float = 1.0
  net_feed_scale: float = 1.0
  feed_override_enabled: bool = True
  spindle_override_enabled: bool = True
  adaptive_feed_enabled: bool = False
  feed_hold: bool = False
  digital_out: List[bool] = field(default_factory=list)
  analog_out: List[float] = field(default_factory=list)
  digital_in: List[bool] = field(default_factory=list)
  analog_in: List[float] = field(default_factory=list)
  tool_offset: EmcPose = field(default_factory=EmcPose)
  probe_val: int = 0
  probing: bool = False
  probed_pos: EmcPose = field(default_factory=EmcPose)
  error: bool = False
  error_message: str = ""


# ============================================================
# 运动控制器
# ============================================================

class MotionController:
  """运动控制器 - 连接解释器和轨迹规划器

  对应 LinuxCNC 的 emcmotController，核心职责:
  1. 读取规范命令，转换为轨迹段加入 TP
  2. 按伺服周期运行 TP，输出位置指令
  3. 通过运动学解算关节位置
  4. 管理机床状态（急停、上电、回零等）
  5. 处理手动进给（JOG）
  """

  def __init__(self, config: Optional[MotionConfig] = None):
    self.config = config or MotionConfig()
    self.tp = TrajectoryPlanner()
    # 按名称创建运动学实例，不再硬编码 IdentityKinematics
    kinName = self.config.kinematics_name.lower()
    self.kinematics = create_kinematics(kinName)
    if self.kinematics is None:
      # 运动学名称未注册时回退到恒等映射
      self.kinematics = IdentityKinematics(self.config.coordinates)
      print(f"[运动] 未知运动学类型 '{kinName}'，回退到 identity")
    print(f"[运动] 创建运动学: {kinName} ({type(self.kinematics).__name__})")
    self.status = EmcmotStatus()
    self.machineState = MachineState.ESTOP
    self.motionState = MotionState.DISABLED

    # 后台运行线程
    self._running = False
    self._thread: Optional[threading.Thread] = None
    self._lock = threading.Lock()

    # 当前笛卡尔位置 (mm)
    self.currentX: float = 0.0
    self.currentY: float = 0.0
    self.currentZ: float = 0.0
    self.currentA: float = 0.0
    self.currentB: float = 0.0
    self.currentC: float = 0.0

    # 轴名称列表（按配置，如 ["X","Y","Z"] 或 ["J1","J2","J3","J4","J5","J6"]）
    self.axisNames: list = list(self.config.axis_names) if self.config.axis_names else ["X", "Y", "Z"]

    # 轴位置映射
    self._jointPositions: dict = {name: 0.0 for name in self.axisNames}

    # 速度设置
    self._traverseRate: float = self.config.default_rapid_velocity
    self._feedRate: float = 0.0

    # 进给保持
    self._feedHold: bool = False

    # 已处理的命令索引
    self._lastProcessedCmd: int = 0

    # 回调函数
    self._onPositionUpdate: Optional[Callable] = None
    self._onStateChange: Optional[Callable] = None
    self._onMessage: Optional[Callable] = None
    self._onProbe: Optional[Callable] = None

    # 初始化状态
    self._initJoints()

  # ----------------------------------------------------------
  # 初始化
  # ----------------------------------------------------------

  def _initJoints(self):
    """初始化关节和主轴状态"""
    self.status.joints = []
    for i in range(self.config.num_joints):
      joint = JointStatus(index=i)
      joint.min_limit = -1000.0
      joint.max_limit = 1000.0
      self.status.joints.append(joint)

    self.status.spindles = []
    for _ in range(self.config.num_spindles):
      self.status.spindles.append(SpindleStatus())

    self.status.digital_out = [False] * 32
    self.status.analog_out = [0.0] * 16
    self.status.digital_in = [False] * 32
    self.status.analog_in = [0.0] * 16

  def _extractXyz(self, pos: Any) -> Tuple[float, float, float]:
    """从位置对象提取XYZ坐标

    支持 EmcPose、PmCartesian 和元组输入。
    """
    if pos is None:
      return (self.currentX, self.currentY, self.currentZ)
    if isinstance(pos, EmcPose):
      return (pos.tran.x, pos.tran.y, pos.tran.z)
    if isinstance(pos, PmCartesian):
      return (pos.x, pos.y, pos.z)
    if isinstance(pos, (tuple, list)) and len(pos) >= 3:
      return (float(pos[0]), float(pos[1]), float(pos[2]))
    return (self.currentX, self.currentY, self.currentZ)

  def getPositionDict(self) -> dict:
    """获取所有轴位置字典 {轴名: 位置值}"""
    pos_map = {
      'X': self.currentX, 'Y': self.currentY, 'Z': self.currentZ,
      'A': self.currentA, 'B': self.currentB, 'C': self.currentC,
    }
    return {name: pos_map.get(name, self._jointPositions.get(name, 0.0))
            for name in self.axisNames}

  def _syncJointPositions(self):
    """同步笛卡尔坐标到关节位置字典"""
    pos_map = {
      'X': self.currentX, 'Y': self.currentY, 'Z': self.currentZ,
      'A': self.currentA, 'B': self.currentB, 'C': self.currentC,
    }
    for name in self.axisNames:
      if name in pos_map:
        self._jointPositions[name] = pos_map[name]

  # ----------------------------------------------------------
  # 回调设置
  # ----------------------------------------------------------

  def setCallbacks(
    self,
    onPosition: Optional[Callable] = None,
    onState: Optional[Callable] = None,
    onMessage: Optional[Callable] = None,
    onProbe: Optional[Callable] = None
  ):
    """设置回调函数

    参数:
      onPosition: 位置更新回调 fn(x, y, z)
      onState: 状态变化回调 fn(state_name)
      onMessage: 消息回调 fn(message_text)
      onProbe: 探测回调 fn(probe_val, x, y, z)
    """
    self._onPositionUpdate = onPosition
    self._onStateChange = onState
    self._onMessage = onMessage
    self._onProbe = onProbe

  # ----------------------------------------------------------
  # 电源与状态控制
  # ----------------------------------------------------------

  def estopOn(self):
    """急停 - 立即中止所有运动"""
    self.machineState = MachineState.ESTOP
    self.motionState = MotionState.DISABLED
    self._feedHold = False
    self.tp.abort()
    if self._onStateChange:
      self._onStateChange("ESTOP")

  def estopOff(self):
    """解除急停"""
    if self.machineState == MachineState.ESTOP:
      self.machineState = MachineState.OFF
      self.motionState = MotionState.DISABLED
      if self._onStateChange:
        self._onStateChange("OFF")

  def powerOn(self):
    """上电 - 从OFF切换到ON"""
    if self.machineState == MachineState.OFF:
      self.machineState = MachineState.ON
      self.motionState = MotionState.FREE
      if self._onStateChange:
        self._onStateChange("ON")

  def powerOff(self):
    """断电"""
    self.machineState = MachineState.OFF
    self.motionState = MotionState.DISABLED
    self.tp.abort()
    if self._onStateChange:
      self._onStateChange("OFF")

  def setMode(self, mode: MotionState):
    """设置运动模式"""
    if self.motionState == mode:
      return
    self.motionState = mode
    if mode == MotionState.FREE:
      self.tp.clear()
    if self._onStateChange:
      self._onStateChange(mode.name)

  # ----------------------------------------------------------
  # 规范命令处理 (核心接口)
  # ----------------------------------------------------------

  def processCanonCommands(self, canon: CanonCommand):
    """处理规范命令列表，将运动命令加入轨迹规划器

    这是连接解释器和运动控制器的关键接口。
    解释器执行完G代码后，调用此方法将规范命令转化为轨迹段。

    参数:
      canon: 规范命令记录器，包含解释器输出的所有命令
    """
    commands = canon.getCommands()

    for cmdEntry in commands:
      cmdName = cmdEntry.command_name
      params = cmdEntry.params

      if cmdName == "INIT_CANON":
        self.tp.clear()
        self._feedRate = 0.0
        self._traverseRate = self.config.default_rapid_velocity

      elif cmdName == "STRAIGHT_TRAVERSE":
        self._handleStraightTraverse(params)

      elif cmdName == "STRAIGHT_FEED":
        self._handleStraightFeed(params)

      elif cmdName == "ARC_FEED":
        self._handleArcFeed(params)

      elif cmdName == "RIGID_TAP":
        self._handleRigidTap(params)

      elif cmdName == "SET_FEED_RATE":
        self._feedRate = params.get('rate', 0.0)

      elif cmdName == "SET_TRAVERSE_RATE":
        self._traverseRate = params.get('rate', self.config.default_rapid_velocity)

      elif cmdName == "SET_FEED_MODE":
        pass  # 仿真器简化: 进给模式保持 mm/min

      elif cmdName == "START_SPINDLE_CLOCKWISE":
        sp = params.get('sp', 0)
        speed = params.get('speed', 0.0)
        direction = params.get('direction', 1)
        self._startSpindle(sp, speed, direction)

      elif cmdName == "START_SPINDLE_COUNTERCLOCKWISE":
        sp = params.get('sp', 0)
        speed = params.get('speed', 0.0)
        direction = params.get('direction', -1)
        self._startSpindle(sp, speed, direction)

      elif cmdName == "STOP_SPINDLE_TURNING":
        sp = params.get('sp', 0)
        self._stopSpindle(sp)

      elif cmdName == "SET_SPINDLE_SPEED":
        sp = params.get('sp', 0)
        speed = params.get('speed', 0.0)
        self._setSpindleSpeed(sp, speed)

      elif cmdName == "SPINDLE_ORIENT":
        sp = params.get('sp', 0)
        orientation = params.get('orientation', 0.0)
        direction = params.get('direction', 1)
        self._spindleOrient(sp, orientation, direction)

      elif cmdName == "WAIT_SPINDLE_ORIENT_COMPLETE":
        pass  # 仿真器简化: 主轴定向立即完成

      elif cmdName == "CHANGE_TOOL":
        pocket = params.get('pocket', 0)
        toolNumber = params.get('tool_number', 0)
        self._changeTool(pocket, toolNumber)

      elif cmdName == "SELECT_TOOL":
        pocket = params.get('pocket', 0)
        toolNumber = params.get('tool_number', 0)
        self._selectTool(pocket, toolNumber)

      elif cmdName == "SET_TOOL_TABLE_ENTRY":
        self._setToolTableEntry(params)

      elif cmdName == "SET_TOOL_TABLE":
        pocket = params.get('pocket', 0)
        tool = params.get('tool', None)
        self._setToolTable(pocket, tool)

      elif cmdName == "USE_TOOL_LENGTH_OFFSET":
        offset = params.get('offset')
        self._useToolLengthOffset(offset)

      elif cmdName == "CANCEL_TOOL_LENGTH_OFFSET":
        self.status.tool_offset = EmcPose()

      elif cmdName == "SET_G5X_OFFSET":
        pass  # 仿真器简化: 坐标偏移由解释器处理

      elif cmdName == "SET_G92_OFFSET":
        pass  # 仿真器简化: 坐标偏移由解释器处理

      elif cmdName == "SET_XY_ROTATION":
        pass  # 仿真器简化: 旋转由解释器处理

      elif cmdName == "DWELL":
        pass  # 仿真器简化: 延时由解释器控制

      elif cmdName == "PROGRAM_STOP":
        self.tp.pause()
        self._feedHold = True

      elif cmdName == "PROGRAM_END":
        self._programEnd()

      elif cmdName == "STOP":
        self.tp.abort()
        self._feedHold = True

      elif cmdName == "FLOOD_ON":
        self.status.digital_out[0] = True

      elif cmdName == "FLOOD_OFF":
        self.status.digital_out[0] = False

      elif cmdName == "MIST_ON":
        self.status.digital_out[1] = True

      elif cmdName == "MIST_OFF":
        self.status.digital_out[1] = False

      elif cmdName == "MESSAGE":
        msg = params.get('message_text', '')
        if self._onMessage and msg:
          self._onMessage(msg)

      elif cmdName == "COMMENT":
        pass  # 注释不执行任何操作

      elif cmdName == "SELECT_PLANE":
        pass  # 仿真器简化: 加工平面由解释器处理

      elif cmdName == "USE_LENGTH_UNITS":
        pass  # 仿真器简化: 单位由解释器处理

      elif cmdName == "SET_MOTION_CONTROL_MODE":
        pass  # 仿真器简化: 运动模式保持连续

      elif cmdName == "CANCEL_MOTION_CONTROL_MODE":
        pass

      elif cmdName == "SET_CUTTER_RADIUS_COMPENSATION":
        pass  # 仿真器简化: 刀补由解释器处理

      elif cmdName == "START_CUTTER_RADIUS_COMPENSATION":
        pass  # 仿真器简化

      elif cmdName == "STOP_CUTTER_RADIUS_COMPENSATION":
        pass  # 仿真器简化

      elif cmdName == "CANCEL_CUTTER_RADIUS_COMPENSATION":
        pass  # 仿真器简化

      elif cmdName == "SET_BLOCK_DELETE":
        pass  # 仿真器简化

      elif cmdName == "DISABLE_FEED_OVERRIDE":
        self.status.feed_override_enabled = False

      elif cmdName == "ENABLE_FEED_OVERRIDE":
        self.status.feed_override_enabled = True

      elif cmdName == "DISABLE_SPEED_OVERRIDE":
        self.status.spindle_override_enabled = False

      elif cmdName == "ENABLE_SPEED_OVERRIDE":
        self.status.spindle_override_enabled = True

      elif cmdName == "DISABLE_ADAPTIVE_FEED":
        self.status.adaptive_feed_enabled = False

      elif cmdName == "ENABLE_ADAPTIVE_FEED":
        self.status.adaptive_feed_enabled = True

      elif cmdName == "DISABLE_FEED_HOLD":
        self._feedHold = False

      elif cmdName == "ENABLE_FEED_HOLD":
        self._feedHold = True

      elif cmdName == "UNLOCK_ROTARY":
        pass  # 仿真器简化

      elif cmdName == "LOCK_ROTARY":
        pass  # 仿真器简化

      elif cmdName == "PROBE_OPEN":
        self.status.probing = False
        self.status.probe_val = 0

      elif cmdName == "PROBE_CLOSE":
        self.status.probing = True

      elif cmdName == "SET_AUX_OUTPUT_VALUE":
        index = params.get('index', 0)
        value = params.get('value', 0.0)
        if 0 <= index < len(self.status.analog_out):
          self.status.analog_out[index] = value

      elif cmdName == "SET_AUX_OUTPUT_BIT":
        index = params.get('index', 0)
        if 0 <= index < len(self.status.digital_out):
          self.status.digital_out[index] = True

      elif cmdName == "CLEAR_AUX_OUTPUT_BIT":
        index = params.get('index', 0)
        if 0 <= index < len(self.status.digital_out):
          self.status.digital_out[index] = False

      elif cmdName == "SET_LOADER":
        pass  # 仿真器简化

      elif cmdName == "SET_OPTIONAL_PROGRAM_STOP":
        pass  # 仿真器简化

      elif cmdName == "SET_OPTIONAL_BLOCK_DELETE":
        pass  # 仿真器简化

  # ----------------------------------------------------------
  # 运动命令处理 (私有)
  # ----------------------------------------------------------

  def _handleStraightTraverse(self, params: Dict[str, Any]):
    """处理直线快移命令 (G0)"""
    endX, endY, endZ = self._extractXyz(params.get('end'))
    a = params.get('a', 0.0)
    b = params.get('b', 0.0)
    c = params.get('c', 0.0)

    # 快移速度乘以快速倍率
    vel = self._traverseRate * self.status.rapid_scale
    vel = max(0.0, vel)

    self.tp.add_line(
      endX, endY, endZ,
      vel,
      self.config.max_acceleration,
      TrajMotionType.LINEAR
    )

    # 更新当前位置
    self.currentX = endX
    self.currentY = endY
    self.currentZ = endZ
    self.currentA = a
    self.currentB = b
    self.currentC = c

    self.status.motion_type = MotionCmdType.TRAVERSE
    self.status.vel = vel

  def _handleStraightFeed(self, params: Dict[str, Any]):
    """处理直线切削进给命令 (G1)"""
    endX, endY, endZ = self._extractXyz(params.get('end'))
    a = params.get('a', 0.0)
    b = params.get('b', 0.0)
    c = params.get('c', 0.0)

    # 切削进给速度
    vel = self._feedRate * self.status.net_feed_scale
    if vel <= 0:
      vel = self.config.default_linear_velocity
    vel = max(0.0, vel)

    self.tp.add_line(
      endX, endY, endZ,
      vel,
      self.config.max_acceleration,
      TrajMotionType.LINEAR
    )

    self.currentX = endX
    self.currentY = endY
    self.currentZ = endZ
    self.currentA = a
    self.currentB = b
    self.currentC = c

    self.status.motion_type = MotionCmdType.LINEAR
    self.status.vel = vel

  def _handleArcFeed(self, params: Dict[str, Any]):
    """处理圆弧进给命令 (G2/G3)"""
    endX, endY, endZ = self._extractXyz(params.get('end'))
    cx, cy, cz = self._extractXyz(params.get('center'))
    rotation = params.get('rotation', 1)
    feedRate = params.get('feed_rate', self._feedRate)
    a = params.get('a', 0.0)
    b = params.get('b', 0.0)
    c = params.get('c', 0.0)

    # 圆弧进给速度
    vel = feedRate * self.status.net_feed_scale
    if vel <= 0:
      vel = self.config.default_linear_velocity
    vel = max(0.0, vel)

    self.tp.add_arc(
      endX, endY, endZ,
      cx, cy, cz,
      int(rotation),
      vel,
      self.config.max_acceleration
    )

    self.currentX = endX
    self.currentY = endY
    self.currentZ = endZ
    self.currentA = a
    self.currentB = b
    self.currentC = c

    self.status.motion_type = MotionCmdType.CIRCULAR
    self.status.vel = vel

  def _handleRigidTap(self, params: Dict[str, Any]):
    """处理刚性攻丝命令 (G84)"""
    endX, endY, endZ = self._extractXyz(params.get('end'))
    feed = params.get('feed', self._feedRate)
    speed = params.get('speed', 0.0)

    # 刚性攻丝简化为直线运动
    vel = feed if feed > 0 else self.config.default_linear_velocity
    self.tp.add_line(
      endX, endY, endZ,
      vel,
      self.config.max_acceleration,
      TrajMotionType.LINEAR
    )

    self.currentX = endX
    self.currentY = endY
    self.currentZ = endZ

    self.status.motion_type = MotionCmdType.RIGID_TAP
    self.status.vel = vel

    # 同步主轴转速
    if self.status.spindles:
      self.status.spindles[0].speed = speed

  def _startSpindle(self, sp: int, speed: float, direction: int):
    """启动主轴

    参数:
      sp: 主轴编号
      speed: 转速 (RPM)
      direction: 方向 (1=CW, -1=CCW)
    """
    if 0 <= sp < len(self.status.spindles):
      spindle = self.status.spindles[sp]
      spindle.speed = speed
      spindle.direction = direction
      spindle.state = 1 if direction >= 1 else 2
      spindle.at_speed = True

  def _stopSpindle(self, sp: int):
    """停止主轴"""
    if 0 <= sp < len(self.status.spindles):
      spindle = self.status.spindles[sp]
      spindle.speed = 0.0
      spindle.state = 0
      spindle.direction = 0
      spindle.at_speed = True

  def _setSpindleSpeed(self, sp: int, speed: float):
    """设置主轴转速"""
    if 0 <= sp < len(self.status.spindles):
      self.status.spindles[sp].speed = speed

  def _spindleOrient(self, sp: int, orientation: float, direction: int):
    """主轴定向 - 仿真器简化为停止并标记"""
    if 0 <= sp < len(self.status.spindles):
      spindle = self.status.spindles[sp]
      spindle.speed = 0.0
      spindle.direction = direction
      spindle.at_speed = True

  def _changeTool(self, pocket: int, toolNumber: int):
    """换刀 - 仿真器简化处理"""
    pass

  def _selectTool(self, pocket: int, toolNumber: int):
    """选择刀具 - 仿真器简化处理"""
    pass

  def _setToolTableEntry(self, params: Dict[str, Any]):
    """设置刀具表条目"""
    pass  # 仿真器简化: 刀具表由解释器维护

  def _setToolTable(self, pocket: int, tool: Any):
    """设置刀具表"""
    pass  # 仿真器简化

  def _useToolLengthOffset(self, offset: Any):
    """应用刀具长度补偿"""
    if offset is None:
      return
    if isinstance(offset, EmcPose):
      self.status.tool_offset = offset.copy()
    elif isinstance(offset, PmCartesian):
      self.status.tool_offset = EmcPose(tran=offset.copy())

  def _programEnd(self):
    """程序结束处理"""
    # 停止主轴
    for i in range(len(self.status.spindles)):
      self._stopSpindle(i)
    # 关闭冷却
    self.status.digital_out[0] = False
    self.status.digital_out[1] = False

  # ----------------------------------------------------------
  # 伺服周期 (核心运行循环)
  # ----------------------------------------------------------

  def runCycle(self):
    """运行一个伺服周期

    在伺服周期中:
    1. 从轨迹规划器获取当前位置指令
    2. 通过运动学解算关节位置
    3. 更新跟随误差
    4. 更新状态结构
    """
    if self.motionState == MotionState.COORD and not self._feedHold:
      dt = self.config.servo_cycle_time

      # 运行轨迹规划器
      result = self.tp.run_cycle(dt)

      if result is not None:
        # 更新笛卡尔位置
        self.currentX = result['x']
        self.currentY = result['y']
        self.currentZ = result['z']

        # 通过运动学解算关节位置
        pose = EmcPose(tran=PmCartesian(self.currentX, self.currentY, self.currentZ))
        joints, ok = self.kinematics.inverse(pose)

        if ok:
          for i, pos in enumerate(joints):
            if i < len(self.status.joints):
              joint = self.status.joints[i]
              # 更新位置指令和反馈 (仿真器中无延迟)
              joint.pos_cmd = pos
              joint.pos_fb = pos
              # 速度和加速度从 TP 获取
              joint.vel_cmd = result.get('vel', 0.0)
              joint.velocity = result.get('vel', 0.0)
              joint.acceleration = 0.0
              # 跟随误差 (仿真器中为 0)
              joint.ferror = 0.0

        # 更新笛卡尔状态
        self.status.carte_pos_cmd = EmcPose(
          tran=PmCartesian(self.currentX, self.currentY, self.currentZ),
          a=self.currentA, b=self.currentB, c=self.currentC
        )
        self.status.carte_pos_fb = self.status.carte_pos_cmd.copy()

        # 更新运动信息
        self.status.current_vel = result.get('vel', 0.0)
        self.status.requested_vel = self.status.vel
        self.status.distance_to_go = result.get('distance_to_go', 0.0)
        self.status.done = result.get('done', False)
      else:
        # 无轨迹输出，保持当前位置
        self.status.current_vel = 0.0
        self.status.done = self.tp.done

    # 更新队列状态
    self.status.tcq_len = len(self.tp.queue)
    self.status.queue_full = self.status.tcq_len >= self.config.queue_max
    self.status.paused = self.tp.paused

    # 回调通知位置更新
    if self._onPositionUpdate:
      self._onPositionUpdate(self.getPositionDict())

  # ----------------------------------------------------------
  # 后台线程运行
  # ----------------------------------------------------------

  def start(self):
    """启动后台伺服循环线程"""
    if self._running:
      return
    self._running = True
    self._thread = threading.Thread(target=self._runLoop, daemon=True)
    self._thread.start()

  def stop(self):
    """停止后台伺服循环线程"""
    self._running = False
    if self._thread is not None:
      self._thread.join(timeout=2.0)
      self._thread = None

  def _runLoop(self):
    """后台伺服循环"""
    cycleTime = self.config.servo_cycle_time
    while self._running:
      startTime = time.monotonic()
      with self._lock:
        self.runCycle()
      # 精确等待到下一个周期
      elapsed = time.monotonic() - startTime
      sleepTime = cycleTime - elapsed
      if sleepTime > 0:
        time.sleep(sleepTime)

  # ----------------------------------------------------------
  # 循环控制
  # ----------------------------------------------------------

  def cycleStart(self):
    """循环启动 - 开始执行轨迹队列"""
    if self.machineState != MachineState.ON:
      return
    self.motionState = MotionState.COORD
    self._feedHold = False
    self.tp.done = False

  def cycleStop(self):
    """循环停止 (进给保持)"""
    self._feedHold = True
    self.tp.pause()

  def cycleResume(self):
    """恢复运行"""
    self._feedHold = False
    self.tp.resume()

  # ----------------------------------------------------------
  # 手动进给 (JOG)
  # ----------------------------------------------------------

  def jog(self, axis: str, velocity: float, increment: float = 0):
    """手动JOG进给

    参数:
      axis: 轴名称 ('X', 'Y', 'Z', 'A', 'B', 'C')
      velocity: 进给速度 (mm/min)，0 表示增量模式
      increment: 增量距离 (mm)，0 表示连续模式
    """
    if self.motionState == MotionState.DISABLED:
      return
    if self.machineState != MachineState.ON:
      return

    axis = axis.upper()
    dt = self.config.servo_cycle_time

    # 计算位移
    if increment != 0:
      displacement = increment
    elif velocity != 0:
      displacement = velocity * dt / 60.0  # mm/min -> mm/cycle
    else:
      return

    # 更新对应轴位置
    # 1. 先尝试标准轴名 (X/Y/Z/A/B/C) 直接更新笛卡尔坐标
    if axis == 'X':
      self.currentX += displacement
    elif axis == 'Y':
      self.currentY += displacement
    elif axis == 'Z':
      self.currentZ += displacement
    elif axis == 'A':
      self.currentA += displacement
    elif axis == 'B':
      self.currentB += displacement
    elif axis == 'C':
      self.currentC += displacement
    # 2. 任意轴名（J1/J2/J3等）更新关节位置字典
    elif axis in self.axisNames:
      self._jointPositions[axis] = self._jointPositions.get(axis, 0.0) + displacement

    # 通过运动学更新关节
    pose = EmcPose(
      tran=PmCartesian(self.currentX, self.currentY, self.currentZ),
      a=self.currentA, b=self.currentB, c=self.currentC
    )
    joints, ok = self.kinematics.inverse(pose)
    if ok:
      for i, pos in enumerate(joints):
        if i < len(self.status.joints):
          self.status.joints[i].pos_cmd = pos
          self.status.joints[i].pos_fb = pos
          self.status.joints[i].ferror = 0.0

    # 更新笛卡尔状态
    self.status.carte_pos_cmd = pose.copy()
    self.status.carte_pos_fb = pose.copy()

    if self._onPositionUpdate:
      self._onPositionUpdate(self.getPositionDict())

  def jogStop(self, axis: str):
    """停止指定轴的JOG"""
    pass  # 仿真器简化: 增量JOG到达目标后自动停止

  # ----------------------------------------------------------
  # 回零
  # ----------------------------------------------------------

  def homeJoint(self, jointNum: int):
    """关节回零

    参数:
      jointNum: 关节编号 (从0开始)
    """
    if not (0 <= jointNum < len(self.status.joints)):
      return

    joint = self.status.joints[jointNum]
    joint.pos_cmd = 0.0
    joint.pos_fb = 0.0
    joint.velocity = 0.0
    joint.homed = True
    joint.homing = False

    # 更新对应轴的笛卡尔位置
    # 使用 axisNames 而不是 coordinates 字符串索引（支持多字符轴名如 J1/J2）
    if jointNum < len(self.axisNames):
      axis = self.axisNames[jointNum]
      if axis == 'X':
        self.currentX = 0.0
      elif axis == 'Y':
        self.currentY = 0.0
      elif axis == 'Z':
        self.currentZ = 0.0
      elif axis == 'A':
        self.currentA = 0.0
      elif axis == 'B':
        self.currentB = 0.0
      elif axis == 'C':
        self.currentC = 0.0
      else:
        # 任意轴名（J1/J2/J3等）归零关节位置字典
        self._jointPositions[axis] = 0.0

    # 更新笛卡尔状态
    pose = EmcPose(
      tran=PmCartesian(self.currentX, self.currentY, self.currentZ),
      a=self.currentA, b=self.currentB, c=self.currentC
    )
    self.status.carte_pos_cmd = pose.copy()
    self.status.carte_pos_fb = pose.copy()

    if self._onPositionUpdate:
      self._onPositionUpdate(self.getPositionDict())

  def homeAll(self):
    """所有关节回零"""
    for i in range(len(self.status.joints)):
      self.homeJoint(i)

  def unhomeJoint(self, jointNum: int):
    """取消关节回零状态"""
    if 0 <= jointNum < len(self.status.joints):
      self.status.joints[jointNum].homed = False

  # ----------------------------------------------------------
  # 倍率设置
  # ----------------------------------------------------------

  def setFeedOverride(self, scale: float):
    """设置进给倍率 (0.0 ~ 2.0)"""
    self.status.feed_scale = max(0.0, min(2.0, scale))
    self._updateNetFeedScale()

  def setRapidOverride(self, scale: float):
    """设置快速倍率 (0.0 ~ 2.0)"""
    self.status.rapid_scale = max(0.0, min(2.0, scale))

  def setSpindleOverride(self, scale: float):
    """设置主轴倍率 (0.0 ~ 2.0)"""
    self.status.spindle_scale = max(0.0, min(2.0, scale))
    for spindle in self.status.spindles:
      spindle.scale = self.status.spindle_scale

  def setAdaptiveFeed(self, enabled: bool):
    """设置自适应进给开关"""
    self.status.adaptive_feed_enabled = enabled
    self._updateNetFeedScale()

  def _updateNetFeedScale(self):
    """更新净进给倍率"""
    net = self.status.feed_scale
    if self.status.adaptive_feed_enabled:
      net *= 0.5  # 仿真器简化: 自适应进给减半
    self.status.net_feed_scale = net

  # ----------------------------------------------------------
  # I/O 控制
  # ----------------------------------------------------------

  def setDigitalOut(self, index: int, value: bool):
    """设置数字输出"""
    if 0 <= index < len(self.status.digital_out):
      self.status.digital_out[index] = value

  def setAnalogOut(self, index: int, value: float):
    """设置模拟输出"""
    if 0 <= index < len(self.status.analog_out):
      self.status.analog_out[index] = value

  def getDigitalIn(self, index: int) -> bool:
    """读取数字输入"""
    if 0 <= index < len(self.status.digital_in):
      return self.status.digital_in[index]
    return False

  def getAnalogIn(self, index: int) -> float:
    """读取模拟输入"""
    if 0 <= index < len(self.status.analog_in):
      return self.status.analog_in[index]
    return 0.0

  # ----------------------------------------------------------
  # 探测
  # ----------------------------------------------------------

  def setProbeValue(self, value: int):
    """设置探测器值 (由外部I/O回调触发)"""
    self.status.probe_val = value
    if self.status.probing and value != 0:
      # 探测触发: 记录当前位置
      self.status.probing = False
      self.status.probed_pos = self.getPosition()
      if self._onProbe:
        self._onProbe(
          value,
          self.currentX, self.currentY, self.currentZ
        )

  # ----------------------------------------------------------
  # 位置查询
  # ----------------------------------------------------------

  def getPosition(self) -> EmcPose:
    """获取当前笛卡尔位置"""
    return EmcPose(
      tran=PmCartesian(self.currentX, self.currentY, self.currentZ),
      a=self.currentA, b=self.currentB, c=self.currentC
    )

  def getJointPosition(self, jointNum: int) -> float:
    """获取指定关节位置"""
    if 0 <= jointNum < len(self.status.joints):
      return self.status.joints[jointNum].pos_fb
    return 0.0

  def isIdle(self) -> bool:
    """控制器是否空闲"""
    return self.tp.is_idle()

  def isAllHomed(self) -> bool:
    """是否所有关节都已回零"""
    return all(j.homed for j in self.status.joints)

  def isMotionComplete(self) -> bool:
    """运动是否完成"""
    return self.status.done and self.tp.is_idle()

  # ----------------------------------------------------------
  # 速度限制
  # ----------------------------------------------------------

  def setMaxVelocity(self, vel: float):
    """设置最大速度"""
    self.config.max_velocity = max(0.0, vel)
    self.tp.maxvel = self.config.max_velocity

  def setMaxAcceleration(self, acc: float):
    """设置最大加速度"""
    self.config.max_acceleration = max(0.0, acc)
    self.tp.maxaccel = self.config.max_acceleration

  # ----------------------------------------------------------
  # 复位
  # ----------------------------------------------------------

  def reset(self):
    """完全复位控制器"""
    self.stop()
    self.tp.reset()

    self.currentX = 0.0
    self.currentY = 0.0
    self.currentZ = 0.0
    self.currentA = 0.0
    self.currentB = 0.0
    self.currentC = 0.0

    self._feedRate = 0.0
    self._traverseRate = self.config.default_rapid_velocity
    self._feedHold = False

    self.status = EmcmotStatus()
    self._initJoints()

    self.status.carte_pos_cmd = EmcPose()
    self.status.carte_pos_fb = EmcPose()
    self.motionState = MotionState.DISABLED

    if self._onPositionUpdate:
      self._onPositionUpdate(self.getPositionDict())

  # ----------------------------------------------------------
  # 状态查询
  # ----------------------------------------------------------

  def getMotionStateName(self) -> str:
    """获取运动状态名称"""
    return self.motionState.name

  def getMachineStateName(self) -> str:
    """获取机床状态名称"""
    return self.machineState.name

  def getStatusSummary(self) -> Dict[str, Any]:
    """获取状态摘要 - 用于调试和显示"""
    return {
      'machineState': self.machineState.name,
      'motionState': self.motionState.name,
      'position': {
        'x': round(self.currentX, 4),
        'y': round(self.currentY, 4),
        'z': round(self.currentZ, 4),
        'a': round(self.currentA, 4),
        'b': round(self.currentB, 4),
        'c': round(self.currentC, 4),
      },
      'velocity': round(self.status.current_vel, 2),
      'distanceToGo': round(self.status.distance_to_go, 4),
      'done': self.status.done,
      'paused': self.status.paused,
      'feedHold': self._feedHold,
      'queueLen': self.status.tcq_len,
      'feedScale': self.status.feed_scale,
      'rapidScale': self.status.rapid_scale,
      'spindleScale': self.status.spindle_scale,
      'spindleSpeed': self.status.spindles[0].speed if self.status.spindles else 0.0,
      'spindleState': self.status.spindles[0].state if self.status.spindles else 0,
      'allHomed': self.isAllHomed(),
    }

  # ----------------------------------------------------------
  # snake_case 别名 - 兼容 GUI 调用
  # ----------------------------------------------------------

  @property
  def machine_state(self) -> MachineState:
    """机床状态 (snake_case 别名)"""
    return self.machineState

  @property
  def motion_state(self) -> MotionState:
    """运动状态 (snake_case 别名)"""
    return self.motionState

  def set_callbacks(self, on_position=None, on_state=None, on_message=None, on_probe=None):
    """设置回调函数 (snake_case 别名)"""
    self.setCallbacks(on_position, on_state, on_message, on_probe)

  def estop_on(self):
    """急停 (snake_case 别名)"""
    self.estopOn()

  def estop_off(self):
    """解除急停 (snake_case 别名)"""
    self.estopOff()

  def power_on(self):
    """上电 (snake_case 别名)"""
    self.powerOn()

  def power_off(self):
    """断电 (snake_case 别名)"""
    self.powerOff()

  def set_mode(self, mode: MotionState):
    """设置运动模式 (snake_case 别名)"""
    self.setMode(mode)

  def process_canon_commands(self, canon: CanonCommand):
    """处理规范命令 (snake_case 别名)"""
    self.processCanonCommands(canon)

  def cycle_start(self):
    """循环启动 (snake_case 别名)"""
    self.cycleStart()

  def cycle_stop(self):
    """循环停止 (snake_case 别名)"""
    self.cycleStop()

  def cycle_resume(self):
    """恢复运行 (snake_case 别名)"""
    self.cycleResume()

  def home_joint(self, joint_num: int):
    """关节回零 (snake_case 别名)"""
    self.homeJoint(joint_num)

  def home_all(self):
    """所有关节回零 (snake_case 别名)"""
    self.homeAll()

  def set_feed_override(self, scale: float):
    """设置进给倍率 (snake_case 别名)"""
    self.setFeedOverride(scale)

  def set_rapid_override(self, scale: float):
    """设置快速倍率 (snake_case 别名)"""
    self.setRapidOverride(scale)

  def set_spindle_override(self, scale: float):
    """设置主轴倍率 (snake_case 别名)"""
    self.setSpindleOverride(scale)

  def run_cycle(self):
    """运行一个伺服周期 (snake_case 别名)"""
    self.runCycle()

  def get_position(self) -> EmcPose:
    """获取当前位置 (snake_case 别名)"""
    return self.getPosition()

  def get_motion_state_name(self) -> str:
    """获取运动状态名称 (snake_case 别名)"""
    return self.getMotionStateName()

  def get_machine_state_name(self) -> str:
    """获取机床状态名称 (snake_case 别名)"""
    return self.getMachineStateName()

  def get_status_summary(self) -> Dict[str, Any]:
    """获取状态摘要 (snake_case 别名)"""
    return self.getStatusSummary()
