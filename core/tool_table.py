"""刀具表管理 - 移植自 LinuxCNC tool_table

刀具数据结构对应 LinuxCNC CANON_TOOL_TABLE：
- toolno: 刀具号
- pocket: 刀库位置
- offset: 多轴偏移（XYZABC UVW）
- diameter: 刀具直径
- front_angle/back_angle: 车刀角度
- orientation: 刀具方向（车刀Q值）
- comment: 注释
"""
import os
import json
from dataclasses import dataclass, field
from typing import Dict, List, Optional


@dataclass
class ToolData:
  """刀具数据 - 对应 LinuxCNC CANON_TOOL_TABLE

  字段说明:
    tool_no: 刀具号（0=无刀）
    pocket: 刀库位置号
    x_offset/y_offset/z_offset: XYZ线性偏移
    a_offset/b_offset/c_offset: 旋转轴偏移
    u_offset/v_offset/w_offset: 第二线性组偏移
    diameter: 刀具直径
    front_angle: 车刀前角
    back_angle: 车刀后角
    orientation: 刀具方向（车刀Q值, 0-9）
    comment: 刀具注释
  """
  tool_no: int = 0
  pocket: int = 0
  x_offset: float = 0.0
  y_offset: float = 0.0
  z_offset: float = 0.0
  a_offset: float = 0.0
  b_offset: float = 0.0
  c_offset: float = 0.0
  u_offset: float = 0.0
  v_offset: float = 0.0
  w_offset: float = 0.0
  diameter: float = 0.0
  front_angle: float = 0.0
  back_angle: float = 0.0
  orientation: int = 0
  comment: str = ""

  def to_dict(self) -> dict:
    """序列化为字典"""
    return {
      'tool_no': self.tool_no,
      'pocket': self.pocket,
      'x_offset': self.x_offset,
      'y_offset': self.y_offset,
      'z_offset': self.z_offset,
      'a_offset': self.a_offset,
      'b_offset': self.b_offset,
      'c_offset': self.c_offset,
      'u_offset': self.u_offset,
      'v_offset': self.v_offset,
      'w_offset': self.w_offset,
      'diameter': self.diameter,
      'front_angle': self.front_angle,
      'back_angle': self.back_angle,
      'orientation': self.orientation,
      'comment': self.comment,
    }

  @classmethod
  def from_dict(cls, d: dict) -> 'ToolData':
    """从字典反序列化"""
    return cls(**{k: v for k, v in d.items() if k in cls.__dataclass_fields__})

  def copy(self) -> 'ToolData':
    """深拷贝"""
    return ToolData(**self.to_dict())


class ToolTable:
  """刀具表管理器

  管理刀具库中所有刀具的数据，支持:
  - 按刀具号/刀库位置存取
  - G10 L1 设置刀具参数
  - M6 换刀查询
  - 文件保存/加载（JSON格式）
  """

  def __init__(self):
    # 刀具号 -> ToolData 映射
    self._tools: Dict[int, ToolData] = {}
    # 刀库位置 -> 刀具号 映射（固定刀位模式）
    self._pocket_map: Dict[int, int] = {}
    # 当前选中的刀号
    self._selected_tool: int = 0
    # 主轴上的刀号
    self._current_tool: int = 0
    # 刀位总数
    self._max_pockets: int = 99
    # 随机换刀模式
    self._random_toolchanger: bool = False

    # 初始化空刀具表
    self._initDefaultTools()

  def _initDefaultTools(self):
    """初始化默认刀具表（空刀位）"""
    for i in range(1, self._max_pockets + 1):
      self._pocket_map[i] = 0

  @property
  def selectedTool(self) -> int:
    """当前选中刀号"""
    return self._selected_tool

  @property
  def currentTool(self) -> int:
    """主轴上当前刀号"""
    return self._current_tool

  def getTool(self, tool_no: int) -> Optional[ToolData]:
    """按刀具号获取刀具数据"""
    return self._tools.get(tool_no)

  def getToolByPocket(self, pocket: int) -> Optional[ToolData]:
    """按刀库位置获取刀具数据"""
    if pocket in self._pocket_map:
      tool_no = self._pocket_map[pocket]
      if tool_no > 0:
        return self._tools.get(tool_no)
    return None

  def setTool(self, tool_data: ToolData):
    """设置/更新刀具数据"""
    self._tools[tool_data.tool_no] = tool_data
    # 更新刀位映射
    if tool_data.pocket > 0:
      self._pocket_map[tool_data.pocket] = tool_data.tool_no
    print(f"[刀具表] 设置刀具 T{tool_data.tool_no} "
          f"P{tool_data.pocket} "
          f"D={tool_data.diameter:.2f} "
          f"Z偏移={tool_data.z_offset:.3f}")

  def removeTool(self, tool_no: int):
    """移除刀具"""
    if tool_no in self._tools:
      td = self._tools.pop(tool_no)
      if td.pocket > 0 and td.pocket in self._pocket_map:
        self._pocket_map[td.pocket] = 0
      print(f"[刀具表] 移除刀具 T{tool_no}")

  def selectTool(self, tool_no: int):
    """选刀（T代码）- 暂存待换刀号"""
    self._selected_tool = tool_no
    print(f"[刀具表] 选刀 T{tool_no}")

  def changeTool(self, tool_no: int) -> Optional[ToolData]:
    """执行换刀（M6）- 返回新刀数据

    换刀流程:
    1. 如果目标刀号有效，从刀具表获取数据
    2. 将新刀安装到主轴
    3. 更新当前刀号
    4. 返回新刀数据（供应用刀具补偿）
    """
    tool_data = self._tools.get(tool_no)
    self._current_tool = tool_no
    self._selected_tool = 0

    if tool_data:
      print(f"[刀具表] 换刀完成 T{tool_no} "
            f"D={tool_data.diameter:.2f} "
            f"Z偏移={tool_data.z_offset:.3f} "
            f"\"{tool_data.comment}\"")
    else:
      # 刀号不在刀具表中，创建空刀具
      tool_data = ToolData(tool_no=tool_no, pocket=0)
      print(f"[刀具表] 换刀 T{tool_no} (无刀具数据)")

    return tool_data

  def setToolParameter(self, tool_no: int, pocket: int = 0, **kwargs):
    """G10 L1 设置刀具参数

    参数:
      tool_no: 刀具号
      pocket: 刀库位置（可选）
      **kwargs: 可设置 x_offset, y_offset, z_offset, diameter, comment 等
    """
    if tool_no not in self._tools:
      self._tools[tool_no] = ToolData(tool_no=tool_no, pocket=pocket)

    td = self._tools[tool_no]
    if pocket > 0:
      td.pocket = pocket
      self._pocket_map[pocket] = tool_no

    for key, val in kwargs.items():
      if hasattr(td, key):
        setattr(td, key, val)

    print(f"[刀具表] G10 L1 设置 T{tool_no} P{td.pocket}: {kwargs}")

  def listTools(self) -> List[ToolData]:
    """列出所有已定义的刀具"""
    return sorted(self._tools.values(), key=lambda t: t.tool_no)

  def clear(self):
    """清空刀具表"""
    self._tools.clear()
    self._pocket_map.clear()
    self._selected_tool = 0
    self._current_tool = 0
    self._initDefaultTools()
    print("[刀具表] 已清空")

  # ==================== 文件I/O ====================

  def saveToFile(self, filepath: str) -> bool:
    """保存刀具表到文件（JSON格式）"""
    try:
      data = {
        'tools': {str(k): v.to_dict() for k, v in self._tools.items()},
        'pocket_map': {str(k): v for k, v in self._pocket_map.items()},
        'current_tool': self._current_tool,
        'selected_tool': self._selected_tool,
      }
      with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
      print(f"[刀具表] 已保存到 {filepath}")
      return True
    except Exception as e:
      print(f"[刀具表] 保存失败: {e}")
      return False

  def loadFromFile(self, filepath: str) -> bool:
    """从文件加载刀具表"""
    if not os.path.exists(filepath):
      print(f"[刀具表] 文件不存在: {filepath}")
      return False
    try:
      with open(filepath, 'r', encoding='utf-8') as f:
        data = json.load(f)
      self._tools = {int(k): ToolData.from_dict(v)
                     for k, v in data.get('tools', {}).items()}
      self._pocket_map = {int(k): v
                          for k, v in data.get('pocket_map', {}).items()}
      self._current_tool = data.get('current_tool', 0)
      self._selected_tool = data.get('selected_tool', 0)
      print(f"[刀具表] 已从 {filepath} 加载 {len(self._tools)} 把刀具")
      return True
    except Exception as e:
      print(f"[刀具表] 加载失败: {e}")
      return False


# ==================== 预设刀具库 ====================

def createDefaultToolTable() -> ToolTable:
  """创建带常用刀具的默认刀具表

  包含:
  - T1: 10mm 立铣刀
  - T2: 6mm 球刀
  - T3: 3mm 钻头
  - T4: 20mm 面铣刀
  - T5: 丝锥 M6
  """
  tt = ToolTable()

  tools = [
    ToolData(tool_no=1, pocket=1, z_offset=50.0, diameter=10.0,
             comment="D10立铣刀"),
    ToolData(tool_no=2, pocket=2, z_offset=45.0, diameter=6.0,
             comment="D6球刀"),
    ToolData(tool_no=3, pocket=3, z_offset=60.0, diameter=3.0,
             comment="D3钻头"),
    ToolData(tool_no=4, pocket=4, z_offset=55.0, diameter=20.0,
             comment="D20面铣刀"),
    ToolData(tool_no=5, pocket=5, z_offset=48.0, diameter=6.0,
             comment="M6丝锥"),
  ]

  for td in tools:
    tt.setTool(td)

  return tt
