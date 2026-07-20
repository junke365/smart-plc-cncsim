from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
from enum import Enum, IntEnum


# ============================================================
# 枚举类型
# ============================================================

class CanonMotionType(Enum):
  """运动类型"""
  TRAVERSE = 0
  LINEAR = 1
  CIRCULAR = 2


class CanonPlane(Enum):
  """加工平面"""
  XY = 17
  XZ = 18
  YZ = 19


class CanonUnits(Enum):
  """单位"""
  INCH = 20
  MM = 21


class CanonDistanceMode(Enum):
  """距离模式"""
  ABSOLUTE = 90
  INCREMENTAL = 91


class CanonFeedMode(Enum):
  """进给模式"""
  INVERSE_TIME = 93
  UNITS_PER_MINUTE = 94
  UNITS_PER_REVOLUTION = 95


class CanonCoordSystem(Enum):
  """工件坐标系"""
  G54 = 54
  G55 = 55
  G56 = 56
  G57 = 57
  G58 = 58
  G59 = 59
  G59_1 = 591
  G59_2 = 592
  G59_3 = 593


class CanonCutterComp(Enum):
  """刀具补偿"""
  OFF = 40
  LEFT = 41
  RIGHT = 42


class CanonToolChange(Enum):
  """换刀模式"""
  NONE = 0
  CHANGE = 1


class CanonMotionMode(Enum):
  """运动控制模式"""
  EXACT_STOP = 0
  EXACT_PATH = 1
  CONTINUOUS = 2


# ============================================================
# 数据类
# ============================================================

@dataclass
class ToolTableEntry:
  """刀具表条目"""
  pocket: int = 0
  tool_number: int = 0
  x_offset: float = 0.0
  y_offset: float = 0.0
  z_offset: float = 0.0
  a_offset: float = 0.0
  b_offset: float = 0.0
  c_offset: float = 0.0
  diameter: float = 0.0
  front_angle: float = 0.0
  back_angle: float = 0.0
  orientation: float = 0.0

  def copy(self) -> 'ToolTableEntry':
    """返回副本"""
    return ToolTableEntry(
      pocket=self.pocket,
      tool_number=self.tool_number,
      x_offset=self.x_offset,
      y_offset=self.y_offset,
      z_offset=self.z_offset,
      a_offset=self.a_offset,
      b_offset=self.b_offset,
      c_offset=self.c_offset,
      diameter=self.diameter,
      front_angle=self.front_angle,
      back_angle=self.back_angle,
      orientation=self.orientation
    )


@dataclass
class CanonCommandEntry:
  """单条规范命令记录"""
  command_name: str = ""
  params: Dict[str, Any] = field(default_factory=dict)

  def __repr__(self) -> str:
    """可读的命令表示"""
    if self.params:
      params_str = ", ".join(f"{k}={v}" for k, v in self.params.items())
      return f"{self.command_name}({params_str})"
    return f"{self.command_name}()"


# ============================================================
# 规范命令记录器
# ============================================================

class CanonCommand:
  """规范命令记录器 - 记录所有解释器规范命令"""

  def __init__(self) -> None:
    self.commands: List[CanonCommandEntry] = []

  def clear(self) -> None:
    """清空命令列表"""
    self.commands.clear()

  def getCommands(self) -> List[CanonCommandEntry]:
    """获取所有命令"""
    return list(self.commands)

  def _record(self, name: str, **kwargs: Any) -> None:
    """记录一条命令"""
    entry = CanonCommandEntry(command_name=name, params=kwargs)
    self.commands.append(entry)

  # --------------------------------------------------------
  # 运动命令
  # --------------------------------------------------------

  def STRAIGHT_TRAVERSE(
    self, end: Any,
    a: float = 0.0, b: float = 0.0, c: float = 0.0,
    u: float = 0.0, v: float = 0.0, w: float = 0.0
  ) -> None:
    """直线快移"""
    self._record("STRAIGHT_TRAVERSE", end=end, a=a, b=b, c=c, u=u, v=v, w=w)

  def STRAIGHT_FEED(
    self, end: Any,
    a: float = 0.0, b: float = 0.0, c: float = 0.0,
    u: float = 0.0, v: float = 0.0, w: float = 0.0
  ) -> None:
    """直线切削进给"""
    self._record("STRAIGHT_FEED", end=end, a=a, b=b, c=c, u=u, v=v, w=w)

  def ARC_FEED(
    self, end: Any, center: Any, axis: float,
    rotation: float, feed_rate: float, a: float = 0.0,
    b: float = 0.0, c: float = 0.0,
    u: float = 0.0, v: float = 0.0, w: float = 0.0
  ) -> None:
    """圆弧进给"""
    self._record(
      "ARC_FEED", end=end, center=center, axis=axis,
      rotation=rotation, feed_rate=feed_rate,
      a=a, b=b, c=c, u=u, v=v, w=w
    )

  def RIGID_TAP(self, end: Any, center: Any, axis: float, feed: float, speed: float) -> None:
    """刚性攻丝"""
    self._record("RIGID_TAP", end=end, center=center, axis=axis, feed=feed, speed=speed)

  # --------------------------------------------------------
  # 速度与进给
  # --------------------------------------------------------

  def SET_FEED_RATE(self, rate: float) -> None:
    """设置进给速率"""
    self._record("SET_FEED_RATE", rate=rate)

  def SET_TRAVERSE_RATE(self, rate: float) -> None:
    """设置快移速率"""
    self._record("SET_TRAVERSE_RATE", rate=rate)

  def SET_FEED_MODE(self, mode: int) -> None:
    """设置进给模式"""
    self._record("SET_FEED_MODE", mode=mode)

  # --------------------------------------------------------
  # 主轴控制
  # --------------------------------------------------------

  def START_SPINDLE_CLOCKWISE(self, speed: float, sp: int = 0, direction: int = 1) -> None:
    """主轴正转"""
    self._record("START_SPINDLE_CLOCKWISE", speed=speed, sp=sp, direction=direction)

  def START_SPINDLE_COUNTERCLOCKWISE(self, speed: float, sp: int = 0, direction: int = -1) -> None:
    """主轴反转"""
    self._record("START_SPINDLE_COUNTERCLOCKWISE", speed=speed, sp=sp, direction=direction)

  def STOP_SPINDLE_TURNING(self, sp: int = 0) -> None:
    """主轴停止"""
    self._record("STOP_SPINDLE_TURNING", sp=sp)

  def SET_SPINDLE_SPEED(self, sp: int, speed: float) -> None:
    """设置主轴转速"""
    self._record("SET_SPINDLE_SPEED", sp=sp, speed=speed)

  def SPINDLE_ORIENT(self, sp: int, orientation: float, direction: int) -> None:
    """主轴定向"""
    self._record("SPINDLE_ORIENT", sp=sp, orientation=orientation, direction=direction)

  def WAIT_SPINDLE_ORIENT_COMPLETE(self, sp: int, timeout: float) -> None:
    """等待主轴定向完成"""
    self._record("WAIT_SPINDLE_ORIENT_COMPLETE", sp=sp, timeout=timeout)

  # --------------------------------------------------------
  # 刀具控制
  # --------------------------------------------------------

  def CHANGE_TOOL(self, pocket: int, tool_number: int) -> None:
    """换刀"""
    self._record("CHANGE_TOOL", pocket=pocket, tool_number=tool_number)

  def SELECT_TOOL(self, pocket: int, tool_number: int) -> None:
    """选择刀具"""
    self._record("SELECT_TOOL", pocket=pocket, tool_number=tool_number)

  def SET_TOOL_TABLE_ENTRY(
    self, pocket: int, tool_number: int,
    x_offset: float = 0.0, y_offset: float = 0.0,
    z_offset: float = 0.0, a_offset: float = 0.0,
    b_offset: float = 0.0, c_offset: float = 0.0,
    diameter: float = 0.0, front_angle: float = 0.0,
    back_angle: float = 0.0, orientation: float = 0.0
  ) -> None:
    """设置刀具表条目"""
    self._record(
      "SET_TOOL_TABLE_ENTRY",
      pocket=pocket, tool_number=tool_number,
      x_offset=x_offset, y_offset=y_offset, z_offset=z_offset,
      a_offset=a_offset, b_offset=b_offset, c_offset=c_offset,
      diameter=diameter, front_angle=front_angle,
      back_angle=back_angle, orientation=orientation
    )

  def USE_TOOL_LENGTH_OFFSET(self, offset: Any) -> None:
    """使用刀具长度补偿"""
    self._record("USE_TOOL_LENGTH_OFFSET", offset=offset)

  def SET_TOOL_TABLE(self, pocket: int, tool: Optional[ToolTableEntry]) -> None:
    """设置刀具表"""
    self._record("SET_TOOL_TABLE", pocket=pocket, tool=tool)

  # --------------------------------------------------------
  # 暂停与停止
  # --------------------------------------------------------

  def DWELL(self, seconds: float) -> None:
    """暂停"""
    self._record("DWELL", seconds=seconds)

  def PROGRAM_STOP(self) -> None:
    """程序停止"""
    self._record("PROGRAM_STOP")

  def PROGRAM_END(self) -> None:
    """程序结束"""
    self._record("PROGRAM_END")

  def STOP(self) -> None:
    """立即停止"""
    self._record("STOP")

  # --------------------------------------------------------
  # 冷却控制
  # --------------------------------------------------------

  def FLOOD_ON(self) -> None:
    """切削液开"""
    self._record("FLOOD_ON")

  def FLOOD_OFF(self) -> None:
    """切削液关"""
    self._record("FLOOD_OFF")

  def MIST_ON(self) -> None:
    """雾化冷却开"""
    self._record("MIST_ON")

  def MIST_OFF(self) -> None:
    """雾化冷却关"""
    self._record("MIST_OFF")

  # --------------------------------------------------------
  # 平面与单位
  # --------------------------------------------------------

  def SELECT_PLANE(self, plane: CanonPlane) -> None:
    """选择加工平面"""
    self._record("SELECT_PLANE", plane=plane)

  def USE_LENGTH_UNITS(self, units: CanonUnits) -> None:
    """设置单位"""
    self._record("USE_LENGTH_UNITS", units=units)

  # --------------------------------------------------------
  # 坐标偏移
  # --------------------------------------------------------

  def SET_G5X_OFFSET(self, offset: Any) -> None:
    """设置G54-G59坐标系偏移"""
    self._record("SET_G5X_OFFSET", offset=offset)

  def SET_G92_OFFSET(self, offset: Any) -> None:
    """设置G92坐标偏移"""
    self._record("SET_G92_OFFSET", offset=offset)

  def SET_XY_ROTATION(self, rotation: Any) -> None:
    """设置XY旋转"""
    self._record("SET_XY_ROTATION", rotation=rotation)

  # --------------------------------------------------------
  # 取消与复位
  # --------------------------------------------------------

  def CANCEL_CUTTER_RADIUS_COMPENSATION(self) -> None:
    """取消刀具半径补偿"""
    self._record("CANCEL_CUTTER_RADIUS_COMPENSATION")

  def CANCEL_MOTION_CONTROL_MODE(self) -> None:
    """取消运动控制模式"""
    self._record("CANCEL_MOTION_CONTROL_MODE")

  def CANCEL_TOOL_LENGTH_OFFSET(self) -> None:
    """取消刀具长度补偿"""
    self._record("CANCEL_TOOL_LENGTH_OFFSET")

  # --------------------------------------------------------
  # 通知命令
  # --------------------------------------------------------

  def COMMENT(self, comment_text: str) -> None:
    """注释"""
    self._record("COMMENT", comment_text=comment_text)

  def MESSAGE(self, message_text: str) -> None:
    """消息"""
    self._record("MESSAGE", message_text=message_text)

  # --------------------------------------------------------
  # 初始化
  # --------------------------------------------------------

  def INIT_CANON(self) -> None:
    """初始化规范模块"""
    self._record("INIT_CANON")

  # --------------------------------------------------------
  # 运动控制模式
  # --------------------------------------------------------

  def SET_MOTION_CONTROL_MODE(self, mode: CanonMotionMode, tolerance: float = 0.0) -> None:
    """设置运动控制模式"""
    self._record("SET_MOTION_CONTROL_MODE", mode=mode, tolerance=tolerance)

  # --------------------------------------------------------
  # 刀具半径补偿
  # --------------------------------------------------------

  def SET_CUTTER_RADIUS_COMPENSATION(self, side: CanonCutterComp, diameter: float = 0.0) -> None:
    """设置刀具半径补偿"""
    self._record(
      "SET_CUTTER_RADIUS_COMPENSATION",
      side=side, diameter=diameter
    )

  def START_CUTTER_RADIUS_COMPENSATION(self, side: CanonCutterComp) -> None:
    """启动刀具半径补偿"""
    self._record("START_CUTTER_RADIUS_COMPENSATION", side=side)

  def STOP_CUTTER_RADIUS_COMPENSATION(self) -> None:
    """停止刀具半径补偿"""
    self._record("STOP_CUTTER_RADIUS_COMPENSATION")

  # --------------------------------------------------------
  # 块删除
  # --------------------------------------------------------

  def SET_BLOCK_DELETE(self, state: bool) -> None:
    """设置块删除状态"""
    self._record("SET_BLOCK_DELETE", state=state)

  # --------------------------------------------------------
  # 超驰控制
  # --------------------------------------------------------

  def DISABLE_FEED_OVERRIDE(self) -> None:
    """禁用进给超驰"""
    self._record("DISABLE_FEED_OVERRIDE")

  def ENABLE_FEED_OVERRIDE(self) -> None:
    """启用进给超驰"""
    self._record("ENABLE_FEED_OVERRIDE")

  def DISABLE_SPEED_OVERRIDE(self) -> None:
    """禁用速度超驰"""
    self._record("DISABLE_SPEED_OVERRIDE")

  def ENABLE_SPEED_OVERRIDE(self) -> None:
    """启用速度超驰"""
    self._record("ENABLE_SPEED_OVERRIDE")

  def DISABLE_ADAPTIVE_FEED(self) -> None:
    """禁用自适应进给"""
    self._record("DISABLE_ADAPTIVE_FEED")

  def ENABLE_ADAPTIVE_FEED(self) -> None:
    """启用自适应进给"""
    self._record("ENABLE_ADAPTIVE_FEED")

  def DISABLE_FEED_HOLD(self) -> None:
    """禁用进给保持"""
    self._record("DISABLE_FEED_HOLD")

  def ENABLE_FEED_HOLD(self) -> None:
    """启用进给保持"""
    self._record("ENABLE_FEED_HOLD")

  # --------------------------------------------------------
  # 旋转轴锁定
  # --------------------------------------------------------

  def UNLOCK_ROTARY(self) -> None:
    """解锁旋转轴"""
    self._record("UNLOCK_ROTARY")

  def LOCK_ROTARY(self) -> None:
    """锁定旋转轴"""
    self._record("LOCK_ROTARY")

  # --------------------------------------------------------
  # 探测
  # --------------------------------------------------------

  def PROBE_OPEN(self, channel: int = 0) -> None:
    """探测器打开"""
    self._record("PROBE_OPEN", channel=channel)

  def PROBE_CLOSE(self, channel: int = 0) -> None:
    """探测器关闭"""
    self._record("PROBE_CLOSE", channel=channel)

  # --------------------------------------------------------
  # 输出控制
  # --------------------------------------------------------

  def SET_AUX_OUTPUT_VALUE(self, index: int, value: float) -> None:
    """设置辅助输出值"""
    self._record("SET_AUX_OUTPUT_VALUE", index=index, value=value)

  def SET_AUX_OUTPUT_BIT(self, index: int) -> None:
    """设置辅助输出位"""
    self._record("SET_AUX_OUTPUT_BIT", index=index)

  def CLEAR_AUX_OUTPUT_BIT(self, index: int) -> None:
    """清除辅助输出位"""
    self._record("CLEAR_AUX_OUTPUT_BIT", index=index)

  # --------------------------------------------------------
  # 载荷检测
  # --------------------------------------------------------

  def SET_LOADER(self, on: bool) -> None:
    """设置装载器"""
    self._record("SET_LOADER", on=on)

  # --------------------------------------------------------
  # 程序检查
  # --------------------------------------------------------

  def SET_OPTIONAL_PROGRAM_STOP(self, state: bool) -> None:
    """设置选择性程序停止"""
    self._record("SET_OPTIONAL_PROGRAM_STOP", state=state)

  def SET_OPTIONAL_BLOCK_DELETE(self, state: bool) -> None:
    """设置选择性块删除"""
    self._record("SET_OPTIONAL_BLOCK_DELETE", state=state)

  # --------------------------------------------------------
  # 速度/进给查询
  # --------------------------------------------------------

  def GET_EXTERNAL_FEED_RATE(self) -> float:
    """获取外部进给速率"""
    self._record("GET_EXTERNAL_FEED_RATE")
    return 0.0

  def GET_EXTERNAL_TRAVERSE_RATE(self) -> float:
    """获取外部快移速率"""
    self._record("GET_EXTERNAL_TRAVERSE_RATE")
    return 0.0

  def GET_EXTERNAL_SPINDLE_SPEED(self, sp: int = 0) -> float:
    """获取外部主轴转速"""
    self._record("GET_EXTERNAL_SPINDLE_SPEED", sp=sp)
    return 0.0

  def GET_EXTERNAL_SPINDLE_OVERRIDE(self, sp: int = 0) -> float:
    """获取外部主轴超驰"""
    self._record("GET_EXTERNAL_SPINDLE_OVERRIDE", sp=sp)
    return 1.0

  def GET_EXTERNAL_FEED_OVERRIDE(self) -> float:
    """获取外部进给超驰"""
    self._record("GET_EXTERNAL_FEED_OVERRIDE")
    return 1.0

  def GET_EXTERNAL_ADAPTIVE_FEED(self) -> float:
    """获取外部自适应进给"""
    self._record("GET_EXTERNAL_ADAPTIVE_FEED")
    return 0.0

  def GET_EXTERNAL_FEED_HOLD(self) -> float:
    """获取外部进给保持状态"""
    self._record("GET_EXTERNAL_FEED_HOLD")
    return 0.0
