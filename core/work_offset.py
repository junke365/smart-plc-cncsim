"""工件坐标系管理 - 移植自 LinuxCNC coord_offsets

管理 G54-G59.3 工件坐标系偏移，对应 LinuxCNC 的:
- parameters[5221-5390]: 9个坐标系原点偏移
- G10 L2/Pn 设置坐标系
- G54-G59.3 切换
"""
from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Optional
from enum import IntEnum


class CoordSystemId(IntEnum):
  """坐标系编号"""
  G54 = 54
  G55 = 55
  G56 = 56
  G57 = 57
  G58 = 58
  G59 = 59
  G591 = 591
  G592 = 592
  G593 = 593


# 坐标系编号 -> 参数索引偏移量 (每个坐标系占20个参数位)
# 参数范围: #5221-#5390
# 规律: offset = (origin_id - 54) * 20
# G54=0, G55=20, G56=40, G57=60, G58=80, G59=100, G59.1=120, G59.2=140, G59.3=160
COORD_PARAM_OFFSET = {
    54: 0, 55: 20, 56: 40, 57: 60,
    58: 80, 59: 100, 591: 120, 592: 140, 593: 160,
}

# 每个坐标系参数位中的轴偏移索引
AXIS_PARAM_INDEX = {
    'X': 0, 'Y': 1, 'Z': 2,
    'A': 3, 'B': 4, 'C': 5,
    'U': 6, 'V': 7, 'W': 8,
    'R': 9,  # 旋转角
}


@dataclass
class CoordOffset:
  """单个坐标系的偏移量

  字段:
    x/y/z: 线性轴原点偏移
    a/b/c: 旋转轴原点偏移
    u/v/w: 第二线性组偏移
    rotation: XY平面旋转角（度）
  """
  x: float = 0.0
  y: float = 0.0
  z: float = 0.0
  a: float = 0.0
  b: float = 0.0
  c: float = 0.0
  u: float = 0.0
  v: float = 0.0
  w: float = 0.0
  rotation: float = 0.0

  def getAxis(self, axis: str) -> float:
    """获取指定轴偏移"""
    return getattr(self, axis.lower(), 0.0)

  def setAxis(self, axis: str, value: float):
    """设置指定轴偏移"""
    if hasattr(self, axis.lower()):
      setattr(self, axis.lower(), value)

  def toList(self) -> List[float]:
    """转为列表 [x,y,z,a,b,c,u,v,w,rotation]"""
    return [self.x, self.y, self.z, self.a, self.b, self.c,
            self.u, self.v, self.w, self.rotation]

  @classmethod
  def fromList(cls, lst: List[float]) -> 'CoordOffset':
    """从列表创建"""
    args = {}
    names = ['x', 'y', 'z', 'a', 'b', 'c', 'u', 'v', 'w', 'rotation']
    for i, name in enumerate(names):
      if i < len(lst):
        args[name] = lst[i]
    return cls(**args)

  def toDict(self) -> dict:
    return {
      'x': self.x, 'y': self.y, 'z': self.z,
      'a': self.a, 'b': self.b, 'c': self.c,
      'u': self.u, 'v': self.v, 'w': self.w,
      'rotation': self.rotation,
    }


class WorkOffsetManager:
  """工件坐标系管理器

  管理 G54-G59.3 共9个工件坐标系的偏移量。
  与 LinuxCNC 的 parameters[5221-5390] 对应。

  功能:
  - 获取/设置各坐标系偏移
  - G10 L2 设置坐标系原点
  - G10 L20 相对当前位置设置坐标系
  - 坐标系切换时的偏移计算
  """

  def __init__(self):
    # 当前激活的坐标系
    self._currentCoord: int = 54
    # 各坐标系偏移
    self._offsets: Dict[int, CoordOffset] = {}
    # G92 偏移
    self._g92Offset: CoordOffset = CoordOffset()
    self._g92Active: bool = False

    # 初始化默认偏移（全部为0）
    for cs_id in CoordSystemId:
      self._offsets[cs_id.value] = CoordOffset()

    print("[WCS] 工件坐标系管理器初始化完成")

  @property
  def currentCoord(self) -> int:
    """当前坐标系编号"""
    return self._currentCoord

  @currentCoord.setter
  def currentCoord(self, value: int):
    if value in self._offsets:
      old = self._currentCoord
      self._currentCoord = value
      print(f"[WCS] 坐标系切换: G{old} -> G{value}")

  @property
  def g92Offset(self) -> CoordOffset:
    """G92偏移"""
    return self._g92Offset

  def getOffset(self, coord_id: int) -> CoordOffset:
    """获取指定坐标系偏移"""
    return self._offsets.get(coord_id, CoordOffset())

  def getCurrentOffset(self) -> CoordOffset:
    """获取当前激活坐标系的偏移"""
    return self._offsets.get(self._currentCoord, CoordOffset())

  def setOffset(self, coord_id: int, offset: CoordOffset):
    """设置指定坐标系偏移"""
    self._offsets[coord_id] = offset
    print(f"[WCS] 设置 G{coord_id} 偏移: "
          f"X={offset.x:.3f} Y={offset.y:.3f} Z={offset.z:.3f}")

  def setAxisOffset(self, coord_id: int, axis: str, value: float):
    """设置指定坐标系的指定轴偏移"""
    if coord_id not in self._offsets:
      self._offsets[coord_id] = CoordOffset()
    self._offsets[coord_id].setAxis(axis, value)

  def setToolTable(self, coord_id: int, p: int, axes_dict: Dict[str, float]):
    """G10 L2 设置坐标系原点

    对应 LinuxCNC convert_setup():
    G10 L2 P<coord_num> X<val> Y<val> Z<val> ...

    参数:
      coord_id: 坐标系编号 (54-593)
      p: P参数（1-9 对应 G54-G59.3）
      axes_dict: 轴名->偏移值 字典
    """
    # P值1-9 映射到坐标系编号
    p_to_cs = {1: 54, 2: 55, 3: 56, 4: 57, 5: 58, 6: 59,
               7: 591, 8: 592, 9: 593}

    if p in p_to_cs:
      target = p_to_cs[p]
    else:
      target = coord_id

    if target not in self._offsets:
      self._offsets[target] = CoordOffset()

    offset = self._offsets[target]
    for axis, val in axes_dict.items():
      offset.setAxis(axis, val)

    print(f"[WCS] G10 L2 P{p} 设置 G{target}: {axes_dict}")

  def setToolTableRelative(self, coord_id: int, p: int,
                           current_pos: Dict[str, float],
                           target_vals: Dict[str, float]):
    """G10 L20 相对当前位置设置坐标系原点

    对应 LinuxCNC convert_setup() L20 分支:
    新偏移 = 当前位置 - 目标值 + 旧偏移

    参数:
      current_pos: 当前各轴位置
      target_vals: 目标值
    """
    p_to_cs = {1: 54, 2: 55, 3: 56, 4: 57, 5: 58, 6: 59,
               7: 591, 8: 592, 9: 593}

    if p in p_to_cs:
      target = p_to_cs[p]
    else:
      target = coord_id

    if target not in self._offsets:
      self._offsets[target] = CoordOffset()

    offset = self._offsets[target]
    for axis, val in target_vals.items():
      cur = current_pos.get(axis, 0.0)
      old = offset.getAxis(axis)
      offset.setAxis(axis, cur - val + old)

    print(f"[WCS] G10 L20 P{p} 设置 G{target}")

  def setG92Offset(self, **kwargs):
    """设置G92偏移"""
    for axis, val in kwargs.items():
      self._g92Offset.setAxis(axis, val)
    self._g92Active = any(v != 0.0 for v in [
      self._g92Offset.x, self._g92Offset.y, self._g92Offset.z])
    print(f"[WCS] G92偏移: X={self._g92Offset.x:.3f} "
          f"Y={self._g92Offset.y:.3f} Z={self._g92Offset.z:.3f}")

  def clearG92(self):
    """清除G92偏移 (G92.1/G92.2)"""
    self._g92Offset = CoordOffset()
    self._g92Active = False
    print("[WCS] G92偏移已清除")

  def getTotalOffset(self, axis: str) -> float:
    """获取指定轴的总偏移 = 坐标系偏移 + G92偏移

    对应 LinuxCNC _get_total_offset 的简化版
    """
    coord_off = self.getCurrentOffset().getAxis(axis)
    g92_off = self._g92Offset.getAxis(axis)
    return coord_off + g92_off

  def listCoordSystems(self) -> List[Tuple[int, CoordOffset]]:
    """列出所有坐标系及其偏移"""
    return [(cs_id, self._offsets.get(cs_id, CoordOffset()))
            for cs_id in CoordSystemId]

  def getCoordName(self, coord_id: int) -> str:
    """获取坐标系显示名称"""
    names = {
      54: "G54", 55: "G55", 56: "G56", 57: "G57",
      58: "G58", 59: "G59", 591: "G59.1", 592: "G59.2", 593: "G59.3",
    }
    return names.get(coord_id, f"G{coord_id}")

  def toParameters(self) -> Dict[int, float]:
    """导出为LinuxCNC参数格式 (#5221-#5390)

    用于与解释器的参数系统同步
    """
    params = {}
    for cs_id, offset in self._offsets.items():
      if cs_id not in COORD_PARAM_OFFSET:
        continue
      base = 5221 + COORD_PARAM_OFFSET[cs_id]
      vals = offset.toList()
      for i, val in enumerate(vals):
        params[base + i] = val
    # 当前坐标系号 #5220
    params[5220] = list(CoordSystemId).index(
      CoordSystemId(self._currentCoord)) + 1 if self._currentCoord in [
        e.value for e in CoordSystemId] else 1
    # G92激活标志 #5210
    params[5210] = 1.0 if self._g92Active else 0.0
    # G92偏移 #5211-#5219
    g92_vals = self._g92Offset.toList()
    for i, val in enumerate(g92_vals[:9]):
      params[5211 + i] = val
    return params

  def fromParameters(self, params: Dict[int, float]):
    """从LinuxCNC参数格式导入"""
    # 导入各坐标系偏移
    for cs_id in CoordSystemId:
      if cs_id.value not in COORD_PARAM_OFFSET:
        continue
      base = 5221 + COORD_PARAM_OFFSET[cs_id.value]
      vals = [params.get(base + i, 0.0) for i in range(10)]
      self._offsets[cs_id.value] = CoordOffset.fromList(vals)
    # G92偏移
    g92_vals = [params.get(5211 + i, 0.0) for i in range(9)]
    self._g92Offset = CoordOffset.fromList(g92_vals)
    self._g92Active = params.get(5210, 0.0) != 0.0
    print("[WCS] 已从参数导入坐标系偏移")
