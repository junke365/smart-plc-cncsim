"""ABB RAPID 风格机器人解释器

支持的指令:
- 运动指令: MoveJ, MoveL, MoveC, MoveABSJ
- 流程控制: IF/ELSEIF/ELSE/ENDIF, FOR/ENDFOR, WHILE/ENDWHILE, TEST/CASE/DEFAULT/ENDTEST, GOTO
- 程序调用: PROCCALL, CALLBYVAR
- 程序控制: STOP, EXIT, BREAK, EXITCYCLE, COMPACTIF
- 赋值: := 操作符
- 注释: COMMENT
- TP功能: TPERASE, TWRITE, TPREADFK, TPREADNUM
- Socket通信: SOCKETCREATE, SOCKETCONNECT, SOCKETSEND, SOCKETRECEIVE, SOCKETCLOSE
- 字符串转换: STRTOVAL
"""
import re
import math
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Any
from enum import Enum

from .canon import CanonCommand
from .coordinates import EmcPose, PmCartesian


# ============================================================
# 枚举类型
# ============================================================

class VarType(Enum):
  """变量类型"""
  NUM = "num"
  STRING = "string"
  ROBTARGET = "robtarget"
  JOINTTARGET = "jointtarget"
  SPEEDDATA = "speeddata"
  ZONEDATA = "zonedata"
  TOOLDATA = "tooldata"
  LOADDATA = "loaddata"


class InstructionType(Enum):
  """指令类型"""
  MOVEJ = "MoveJ"
  MOVEL = "MoveL"
  MOVEC = "MoveC"
  MOVEABSJ = "MoveABSJ"
  ASSIGN = "ASSIGN"
  IF = "IF"
  ELSEIF = "ELSEIF"
  ELSE = "ELSE"
  ENDIF = "ENDIF"
  FOR = "FOR"
  ENDFOR = "ENDFOR"
  WHILE = "WHILE"
  ENDWHILE = "ENDWHILE"
  TEST = "TEST"
  CASE = "CASE"
  DEFAULT = "DEFAULT"
  ENDTEST = "ENDTEST"
  GOTO = "GOTO"
  LABEL = "LABEL"
  PROCCALL = "PROC"
  CALLBYVAR = "CALLBYVAR"
  STOP = "STOP"
  EXIT = "EXIT"
  BREAK = "BREAK"
  EXITCYCLE = "EXITCYCLE"
  COMMENT = "COMMENT"
  TPWRITE = "TPWRITE"
  TPREADFK = "TPREADFK"
  TPREADNUM = "TPREADNUM"
  TPERASE = "TPERASE"
  SOCKETCREATE = "SOCKETCREATE"
  SOCKETCONNECT = "SOCKETCONNECT"
  SOCKETSEND = "SOCKETSEND"
  SOCKETRECEIVE = "SOCKETRECEIVE"
  SOCKETCLOSE = "SOCKETCLOSE"
  STRTOVAL = "STRTOVAL"
  CONNECT = "CONNECT"
  COMPACTIF = "COMPACTIF"
  WAIT = "WAIT"
  RESET = "RESET"
  SET = "SET"
  PULSE = "PULSE"
  CLKRESET = "CLKRESET"
  CLKSTART = "CLKSTART"
  CLKSTOP = "CLKSTOP"


# ============================================================
# 数据类
# ============================================================

@dataclass
class Variable:
  """变量定义"""
  name: str = ""
  var_type: VarType = VarType.NUM
  value: Any = 0.0
  is_pers: bool = False  # PERS持久变量
  is_const: bool = False  # CONST常量


@dataclass
class RobotTarget:
  """机器人目标点数据"""
  x: float = 0.0
  y: float = 0.0
  z: float = 0.0
  q1: float = 1.0
  q2: float = 0.0
  q3: float = 0.0
  q4: float = 0.0
  cf1: float = 0.0
  cf4: float = 0.0
  cf6: float = 0.0
  cfx: float = 0.0


@dataclass
class JointTarget:
  """关节目标点数据"""
  joints: List[float] = field(default_factory=lambda: [0.0] * 6)
  extax: List[float] = field(default_factory=lambda: [0.0] * 6)


@dataclass
class SpeedData:
  """速度数据"""
  v: float = 100.0
  v_tcp: float = 100.0
  v_ori: float = 500.0
  v_leax: float = 5000.0
  v_reax: float = 5000.0


@dataclass
class ZoneData:
  """区域数据"""
  zone_name: str = "fine"
  pzone_tcp: float = 0.0
  pzone_ori: float = 0.0
  pzone_eax: float = 0.0
  zone_ori: float = 0.0
  zone_leax: float = 0.0
  zone_reax: float = 0.0


@dataclass
class ToolData:
  """工具数据"""
  tool_name: str = "tool0"
  robhold: bool = False
  tframe: Optional[RobotTarget] = None
  tload: Optional[Any] = None


@dataclass
class LoadData:
  """负载数据"""
  mass: float = 0.0
  cog: Optional[RobotTarget] = None
  aom: Optional[RobotTarget] = None
  ixx: float = 0.0
  iyy: float = 0.0
  izz: float = 0.0


@dataclass
class SubContext:
  """程序调用上下文"""
  filename: str = ""
  line_number: int = 0
  proc_name: str = ""
  saved_vars: Dict[str, Any] = field(default_factory=dict)


@dataclass
class LoopContext:
  """循环上下文"""
  loop_type: str = ""
  var_name: str = ""
  start_val: float = 0.0
  end_val: float = 0.0
  step: float = 1.0
  start_line: int = 0
  end_line: int = 0


# ============================================================
# 预定义常量
# ============================================================

# 预定义速度数据
SPEED_DATA = {
  "v10": SpeedData(v=10, v_tcp=10),
  "v20": SpeedData(v=20, v_tcp=20),
  "v50": SpeedData(v=50, v_tcp=50),
  "v100": SpeedData(v=100, v_tcp=100),
  "v200": SpeedData(v=200, v_tcp=200),
  "v300": SpeedData(v=300, v_tcp=300),
  "v500": SpeedData(v=500, v_tcp=500),
  "v1000": SpeedData(v=1000, v_tcp=1000),
  "v1500": SpeedData(v=1500, v_tcp=1500),
  "v2000": SpeedData(v=2000, v_tcp=2000),
  "v3000": SpeedData(v=3000, v_tcp=3000),
  "v5000": SpeedData(v=5000, v_tcp=5000),
  "v7000": SpeedData(v=7000, v_tcp=7000),
  "vmax": SpeedData(v=5000, v_tcp=5000),
}

# 预定义区域数据
ZONE_DATA = {
  "fine": ZoneData(zone_name="fine", pzone_tcp=0, pzone_ori=0),
  "z0": ZoneData(zone_name="z0", pzone_tcp=0, pzone_ori=0, zone_ori=0),
  "z1": ZoneData(zone_name="z1", pzone_tcp=0.3, pzone_ori=0.3, zone_ori=1),
  "z5": ZoneData(zone_name="z5", pzone_tcp=1, pzone_ori=1, zone_ori=5),
  "z10": ZoneData(zone_name="z10", pzone_tcp=2, pzone_ori=2, zone_ori=10),
  "z15": ZoneData(zone_name="z15", pzone_tcp=3, pzone_ori=3, zone_ori=15),
  "z20": ZoneData(zone_name="z20", pzone_tcp=4, pzone_ori=4, zone_ori=20),
  "z30": ZoneData(zone_name="z30", pzone_tcp=6, pzone_ori=6, zone_ori=30),
  "z40": ZoneData(zone_name="z40", pzone_tcp=8, pzone_ori=8, zone_ori=40),
  "z50": ZoneData(zone_name="z50", pzone_tcp=10, pzone_ori=10, zone_ori=50),
  "z60": ZoneData(zone_name="z60", pzone_tcp=12, pzone_ori=12, zone_ori=60),
  "z80": ZoneData(zone_name="z80", pzone_tcp=16, pzone_ori=16, zone_ori=80),
  "z100": ZoneData(zone_name="z100", pzone_tcp=20, pzone_ori=20, zone_ori=100),
}

# 预定义工具
TOOL_DATA = {
  "tool0": ToolData(tool_name="tool0", robhold=False),
}


# ============================================================
# 机器人解释器
# ============================================================

class RobotInterpreter:
  """ABB RAPID 风格机器人解释器"""

  def __init__(self):
    self._canon = CanonCommand()
    self._error_msg: str = ""

    # 程序状态
    self._file_lines: List[str] = []
    self._current_line: int = 0
    self._filename: str = ""
    self._line_num: int = 0

    # 变量存储
    self._variables: Dict[str, Variable] = {}
    self._local_vars: List[Dict[str, Variable]] = []

    # 程序控制
    self._call_stack: List[SubContext] = []
    self._if_stack: List[bool] = []
    self._skip_stack: List[bool] = []
    self._loop_stack: List[LoopContext] = []
    self._break_flag: bool = False
    self._exit_flag: bool = False
    self._exitcycle_flag: bool = False
    self._compact_if: bool = False

    # 程序跳转表（标签 -> 行号）
    self._labels: Dict[str, int] = {}

    # TP读写缓冲
    self._tp_buffer: List[str] = []
    self._tp_read_result: Any = None

    # Socket (简化)
    self._sockets: Dict[str, Any] = {}

    # 初始化预定义变量
    self._init_predefined()

    print("[机器人解释器] 已初始化")

  @property
  def canon(self) -> CanonCommand:
    return self._canon

  @property
  def error(self) -> str:
    return self._error_msg

  @property
  def variables(self) -> Dict[str, Variable]:
    return self._variables

  def _init_predefined(self):
    """初始化预定义速度/区域/工具数据"""
    for name, data in SPEED_DATA.items():
      self._variables[name] = Variable(name=name, var_type=VarType.SPEEDDATA, value=data)
    for name, data in ZONE_DATA.items():
      self._variables[name] = Variable(name=name, var_type=VarType.ZONEDATA, value=data)
    for name, data in TOOL_DATA.items():
      self._variables[name] = Variable(name=name, var_type=VarType.TOOLDATA, value=data)
    # 默认工具和负载
    self._variables["tool0"] = Variable(
      name="tool0", var_type=VarType.TOOLDATA,
      value=ToolData(tool_name="tool0", robhold=False))
    self._variables["load0"] = Variable(
      name="load0", var_type=VarType.LOADDATA, value=LoadData(mass=0))
    # 预定义robtarget
    self._variables["home"] = Variable(
      name="home", var_type=VarType.ROBTARGET,
      value=RobotTarget(x=0, y=0, z=400, q1=1, q2=0, q3=0, q4=0))

  # ==================== 文件操作 ====================

  def open_file(self, filename: str) -> Tuple[bool, str]:
    """打开RAPID程序文件"""
    try:
      with open(filename, 'r', encoding='utf-8', errors='replace') as f:
        self._file_lines = f.readlines()
      self._current_line = 0
      self._filename = filename
      self._build_label_table()
      return (True, "")
    except Exception as e:
      self._error_msg = f"无法打开文件: {e}"
      return (False, self._error_msg)

  def _build_label_table(self):
    """构建标签跳转表"""
    self._labels.clear()
    for i, line in enumerate(self._file_lines):
      line_stripped = line.strip()
      # 标签格式: "标签名:"
      if ':' in line_stripped and not line_stripped.startswith('!'):
        parts = line_stripped.split(':', 1)
        label = parts[0].strip()
        if label and re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', label):
          self._labels[label] = i

  def init(self):
    """完全初始化解释器状态"""
    self._canon.clear()
    self._canon.INIT_CANON()
    self._error_msg = ""
    self._current_line = 0
    self._line_num = 0
    self._call_stack.clear()
    self._if_stack.clear()
    self._skip_stack.clear()
    self._loop_stack.clear()
    self._break_flag = False
    self._exit_flag = False
    self._exitcycle_flag = False
    self._compact_if = False
    self._tp_buffer.clear()
    self._sockets.clear()
    self._local_vars.clear()
    # 清除非预定义变量
    predefined = set(SPEED_DATA.keys()) | set(ZONE_DATA.keys()) | set(TOOL_DATA.keys())
    predefined |= {"tool0", "load0", "home"}
    to_remove = [k for k in self._variables if k not in predefined]
    for k in to_remove:
      del self._variables[k]
    self._init_predefined()

  def reset(self):
    """复位"""
    self.init()

  def execute_file(self) -> Tuple[bool, str]:
    """执行整个程序"""
    while self._current_line < len(self._file_lines):
      if self._exit_flag:
        break
      ok, err = self._execute_line()
      if not ok:
        return (False, err)
      self._current_line += 1
    return (True, "")

  def execute(self, line: str) -> Tuple[bool, str]:
    """执行单行RAPID代码 (MDI模式)"""
    self._line_num += 1

    # 检查是否需要跳过（IF/FOR/WHILE/TEST等条件不满足时）
    if self._should_skip():
      # 仍需处理 ENDIF/ELSEIF/ELSE/ENDFOR/ENDWHILE/ENDTEST 等闭合或分支指令
      inst = self._parse_instruction(line)
      if inst is not None:
        itype = inst[0]
        if itype in (InstructionType.ENDIF, InstructionType.ELSEIF,
                     InstructionType.ELSE,
                     InstructionType.ENDFOR, InstructionType.ENDWHILE,
                     InstructionType.ENDTEST):
          return self._execute_instruction(inst, line)
      return (True, "")

    # 检查BREAK/EXIT/EXITCYCLE
    if self._break_flag or self._exit_flag or self._exitcycle_flag:
      return (True, "")

    inst = self._parse_instruction(line)
    if inst is not None:
      return self._execute_instruction(inst, line)
    return (True, "")

  # ==================== 行执行 ====================

  def _execute_line(self) -> Tuple[bool, str]:
    """执行当前行"""
    if self._current_line >= len(self._file_lines):
      return (True, "")

    line = self._file_lines[self._current_line].rstrip('\n\r')
    self._line_num = self._current_line + 1

    # 检查COMPACTIF模式
    if self._compact_if:
      return self._handle_compact_if(line)

    # 检查是否需要跳过
    if self._should_skip():
      # 仍需处理 ELSEIF/ELSE/ENDIF 等分支和闭合指令
      inst = self._parse_instruction(line)
      if inst is not None:
        itype = inst[0]
        if itype in (InstructionType.ENDIF, InstructionType.ELSEIF,
                     InstructionType.ELSE,
                     InstructionType.ENDFOR, InstructionType.ENDWHILE,
                     InstructionType.ENDTEST):
          return self._execute_instruction(inst, line)
      return (True, "")

    # 检查BREAK/EXIT/EXITCYCLE
    if self._break_flag or self._exit_flag or self._exitcycle_flag:
      return (True, "")

    # 解析和执行
    inst = self._parse_instruction(line)
    if inst is not None:
      return self._execute_instruction(inst, line)

    return (True, "")

  def _should_skip(self) -> bool:
    """检查当前行是否应被跳过"""
    return (self._break_flag or self._exit_flag or
            self._exitcycle_flag or
            (self._skip_stack and self._skip_stack[-1]))

  # ==================== 指令解析 ====================

  def _parse_instruction(self, line: str) -> Optional[Tuple[InstructionType, str]]:
    """解析一行RAPID代码"""
    line = line.strip()

    # 空行
    if not line:
      return None

    # 注释 (! 开头)
    if line.startswith('!'):
      return (InstructionType.COMMENT, line[1:].strip())

    # 模块声明和过程头尾 - 跳过
    if line.startswith(('MODULE', 'ENDMODULE', 'PROC', 'ENDPROC',
                        'FUNC', 'ENDFUNC', 'TRAP', 'ENDTRAP',
                        'record', 'ERROR')):
      return None

    # 标签 (格式: "标签名:")
    label_match = re.match(r'^([A-Za-z_][A-Za-z0-9_]*):', line)
    if label_match:
      label_name = label_match.group(1)
      # 检查是否是关键字而不是标签
      if label_name.upper() not in ('PROC', 'ENDPROC', 'IF', 'FOR', 'WHILE',
                                     'TEST', 'CASE', 'DEFAULT', 'END',
                                     'CONNECT', 'ERROR', 'RECORD'):
        return (InstructionType.LABEL, label_name)

    # 预处理: 移除行尾注释 (!)
    comment_idx = line.find('!')
    if comment_idx >= 0:
      line = line[:comment_idx].strip()

    # 预处理: 合并续行 (以 , 结尾的行与下一行合并)
    # 简化处理: 在RAPID中一个语句可能跨多行

    upper_line = line.upper().lstrip()

    # COMPACTIF (紧凑IF)
    if upper_line.startswith('COMPACT'):
      self._compact_if = True
      rest = line[7:].strip()
      if rest:
        inst = self._parse_instruction(rest)
        return inst
      return None

    # IF
    if upper_line.startswith('IF ') or upper_line.startswith('IF('):
      return (InstructionType.IF, line)

    # ELSEIF
    if upper_line.startswith('ELSEIF ') or upper_line.startswith('ELSEIF('):
      return (InstructionType.ELSEIF, line)

    # ELSE
    if upper_line == 'ELSE':
      return (InstructionType.ELSE, line)

    # ENDIF
    if upper_line in ('ENDIF', 'END IF'):
      return (InstructionType.ENDIF, line)

    # FOR
    if upper_line.startswith('FOR ') or upper_line.startswith('FOR('):
      return (InstructionType.FOR, line)

    # ENDFOR
    if upper_line in ('ENDFOR', 'END FOR'):
      return (InstructionType.ENDFOR, line)

    # WHILE
    if upper_line.startswith('WHILE ') or upper_line.startswith('WHILE('):
      return (InstructionType.WHILE, line)

    # ENDWHILE
    if upper_line in ('ENDWHILE', 'END WHILE'):
      return (InstructionType.ENDWHILE, line)

    # TEST
    if upper_line.startswith('TEST ') or upper_line.startswith('TEST('):
      return (InstructionType.TEST, line)

    # CASE
    if upper_line.startswith('CASE '):
      return (InstructionType.CASE, line)

    # DEFAULT
    if upper_line == 'DEFAULT:':
      return (InstructionType.DEFAULT, line)

    # ENDTEST
    if upper_line in ('ENDTEST', 'END TEST'):
      return (InstructionType.ENDTEST, line)

    # GOTO
    if upper_line.startswith('GOTO ') or upper_line.startswith('GOTO('):
      return (InstructionType.GOTO, line)

    # 运动指令
    if upper_line.startswith('MOVEJ ') or upper_line.startswith('MOVEJ('):
      return (InstructionType.MOVEJ, line)
    if upper_line.startswith('MOVEL ') or upper_line.startswith('MOVEL('):
      return (InstructionType.MOVEL, line)
    if upper_line.startswith('MOVEC ') or upper_line.startswith('MOVEC('):
      return (InstructionType.MOVEC, line)
    if upper_line.startswith('MOVEABSJ ') or upper_line.startswith('MOVEABSJ('):
      return (InstructionType.MOVEABSJ, line)

    # 赋值
    if ':=' in line:
      return (InstructionType.ASSIGN, line)

    # PROCCALL (过程调用)
    if re.match(r'^[A-Za-z_][A-Za-z0-9_]*\s*[\(;]', line):
      # 可能是过程调用或函数调用
      parts = re.split(r'[\(;]', line, 1)
      name = parts[0].strip()
      if name.upper() not in ('IF', 'FOR', 'WHILE', 'TEST', 'REPEAT',
                               'SELECT', 'CASE', 'WHEN'):
        return (InstructionType.PROCCALL, line)

    # CALLBYVAR
    if upper_line.startswith('CALLBYVAR ') or upper_line.startswith('CALLBYVAR('):
      return (InstructionType.CALLBYVAR, line)

    # STOP (允许带或不带分号)
    if upper_line.rstrip(';') == 'STOP':
      return (InstructionType.STOP, line)

    # EXIT (允许带或不带分号)
    if upper_line.rstrip(';') == 'EXIT':
      return (InstructionType.EXIT, line)

    # BREAK (允许带或不带分号)
    if upper_line.rstrip(';') == 'BREAK':
      return (InstructionType.BREAK, line)

    # EXITCYCLE (允许带或不带分号)
    if upper_line.rstrip(';') == 'EXITCYCLE':
      return (InstructionType.EXITCYCLE, line)

    # TPWRITE
    if upper_line.startswith('TPWRITE ') or upper_line.startswith('TPWRITE('):
      return (InstructionType.TPWRITE, line)

    # TPERASE (允许带或不带分号)
    if upper_line.rstrip(';') == 'TPERASE':
      return (InstructionType.TPERASE, line)

    # TPREADFK
    if upper_line.startswith('TPREADFK ') or upper_line.startswith('TPREADFK('):
      return (InstructionType.TPREADFK, line)

    # TPREADNUM
    if upper_line.startswith('TPREADNUM ') or upper_line.startswith('TPREADNUM('):
      return (InstructionType.TPREADNUM, line)

    # Socket通信
    if upper_line.startswith('SOCKETCREATE '):
      return (InstructionType.SOCKETCREATE, line)
    if upper_line.startswith('SOCKETCONNECT '):
      return (InstructionType.SOCKETCONNECT, line)
    if upper_line.startswith('SOCKETSEND '):
      return (InstructionType.SOCKETSEND, line)
    if upper_line.startswith('SOCKETRECEIVE '):
      return (InstructionType.SOCKETRECEIVE, line)
    if upper_line.startswith('SOCKETCLOSE '):
      return (InstructionType.SOCKETCLOSE, line)

    # STRTOVAL
    if upper_line.startswith('STRTOVAL ') or upper_line.startswith('STRTOVAL('):
      return (InstructionType.STRTOVAL, line)

    # CONNECT
    if upper_line.startswith('CONNECT '):
      return (InstructionType.CONNECT, line)

    # WAIT
    if upper_line.startswith('WAIT ') or upper_line.startswith('WAIT('):
      return (InstructionType.WAIT, line)

    # Reset
    if upper_line.startswith('RESET ') or upper_line.startswith('RESET('):
      return (InstructionType.RESET, line)

    # Set
    if upper_line.startswith('SET ') or upper_line.startswith('SET('):
      return (InstructionType.SET, line)

    # PulseDO
    if upper_line.startswith('PULSE ') or upper_line.startswith('PULSE('):
      return (InstructionType.PULSE, line)

    # Clock
    if upper_line.startswith('CLKRESET'):
      return (InstructionType.CLKRESET, line)
    if upper_line.startswith('CLKSTART'):
      return (InstructionType.CLKSTART, line)
    if upper_line.startswith('CLKSTOP'):
      return (InstructionType.CLKSTOP, line)

    # 跳过模块声明等
    return None

  # ==================== 参数解析 ====================

  def _parse_args(self, text: str) -> List[str]:
    """解析括号中的参数列表"""
    start = text.find('(')
    if start == -1:
      start = text.find(' ')
    if start == -1:
      return []
    end = text.rfind(')')
    if end == -1:
      end = len(text)
    arg_str = text[start + 1:end]
    return self._split_args(arg_str)

  def _split_args(self, text: str) -> List[str]:
    """智能分割参数（处理嵌套括号和字符串）"""
    args = []
    depth = 0
    current = []
    in_string = False

    for ch in text:
      if ch == '"' and not in_string:
        in_string = True
        current.append(ch)
      elif ch == '"' and in_string:
        in_string = False
        current.append(ch)
      elif ch in '([' and not in_string:
        depth += 1
        current.append(ch)
      elif ch in ')]' and not in_string:
        depth -= 1
        current.append(ch)
      elif ch == ',' and depth == 0 and not in_string:
        args.append(''.join(current).strip())
        current = []
      else:
        current.append(ch)

    if current:
      args.append(''.join(current).strip())

    return args

  def _parse_target(self, text: str) -> RobotTarget:
    """解析目标点 [x, y, z, q1, q2, q3, q4]"""
    target = RobotTarget()
    args = self._parse_args(text)
    vals = [self._eval_expression(a) for a in args]
    nums = [v for v in vals if isinstance(v, (int, float))]

    if len(nums) >= 3:
      target.x, target.y, target.z = nums[0], nums[1], nums[2]
    if len(nums) >= 7:
      target.q1, target.q2, target.q3, target.q4 = nums[3], nums[4], nums[5], nums[6]
    if len(nums) >= 11:
      target.cf1, target.cf4, target.cf6, target.cfx = nums[7], nums[8], nums[9], nums[10]
    return target

  def _parse_joint_target(self, text: str) -> JointTarget:
    """解析关节目标 [[j1, j2, j3, j4, j5, j6], [e1, e2, e3, e4, e5, e6]]"""
    target = JointTarget()
    # 移除外层括号
    text = text.strip()
    if text.startswith('[') and text.endswith(']'):
      text = text[1:-1]

    parts = self._split_args(text)
    if len(parts) >= 1:
      joint_args = self._parse_args(parts[0])
      for i, a in enumerate(joint_args[:6]):
        val = self._eval_expression(a)
        if isinstance(val, (int, float)):
          target.joints[i] = val
    if len(parts) >= 2:
      ext_args = self._parse_args(parts[1])
      for i, a in enumerate(ext_args[:6]):
        val = self._eval_expression(a)
        if isinstance(val, (int, float)):
          target.extax[i] = val
    return target

  def _parse_speed_data(self, text: str) -> SpeedData:
    """解析速度数据 [v_tcp, v_ori, v_leax, v_reax]"""
    text = text.strip()
    if text in SPEED_DATA:
      return SPEED_DATA[text]
    if text in self._variables:
      var = self._variables[text]
      if var.var_type == VarType.SPEEDDATA:
        return var.value
    target = SpeedData()
    args = self._parse_args(text)
    vals = [self._eval_expression(a) for a in args]
    nums = [v for v in vals if isinstance(v, (int, float))]
    if len(nums) >= 1:
      target.v = nums[0]
      target.v_tcp = nums[0]
    if len(nums) >= 2:
      target.v_ori = nums[1]
    if len(nums) >= 3:
      target.v_leax = nums[2]
    if len(nums) >= 4:
      target.v_reax = nums[3]
    return target

  def _parse_zone_data(self, text: str) -> ZoneData:
    """解析区域数据 z10 或 [pzone_tcp, pzone_ori, zone_ori]"""
    text = text.strip()
    if text in ZONE_DATA:
      return ZONE_DATA[text]
    if text in self._variables:
      var = self._variables[text]
      if var.var_type == VarType.ZONEDATA:
        return var.value
    target = ZoneData()
    args = self._parse_args(text)
    vals = [self._eval_expression(a) for a in args]
    nums = [v for v in vals if isinstance(v, (int, float))]
    if len(nums) >= 1:
      target.pzone_tcp = nums[0]
    if len(nums) >= 2:
      target.pzone_ori = nums[1]
    if len(nums) >= 3:
      target.zone_ori = nums[2]
    return target

  def _parse_tool_data(self, text: str) -> ToolData:
    """解析工具数据"""
    text = text.strip()
    if text in self._variables:
      var = self._variables[text]
      if var.var_type == VarType.TOOLDATA:
        return var.value
    return ToolData(tool_name=text)

  def _parse_load_data(self, text: str) -> LoadData:
    """解析负载数据 [mass, [cogx, cogy, cogz]]"""
    text = text.strip()
    if text in self._variables:
      var = self._variables[text]
      if var.var_type == VarType.LOADDATA:
        return var.value
    load = LoadData()
    args = self._parse_args(text)
    vals = [self._eval_expression(a) for a in args]
    nums = [v for v in vals if isinstance(v, (int, float))]
    if len(nums) >= 1:
      load.mass = nums[0]
    if len(nums) >= 4:
      load.cog = RobotTarget(x=nums[1], y=nums[2], z=nums[3])
    return load

  # ==================== 表达式求值 ====================

  def _eval_expression(self, expr: str) -> Any:
    """求值表达式（支持变量、数字、字符串、函数）"""
    if expr is None:
      return None

    expr = expr.strip()
    if not expr:
      return None

    # 字符串
    if expr.startswith('"') and expr.endswith('"'):
      return expr[1:-1]

    # 布尔
    if expr.upper() == 'TRUE':
      return True
    if expr.upper() == 'FALSE':
      return False

    # 数字
    try:
      if '.' in expr or 'e' in expr.lower():
        return float(expr)
      return int(expr)
    except ValueError:
      pass

    # 去除分号
    if expr.endswith(';'):
      expr = expr[:-1].strip()

    # 函数调用
    func_match = re.match(r'^([A-Za-z_][A-Za-z0-9_]*)\s*\((.+)\)$', expr)
    if func_match:
      func_name = func_match.group(1).upper()
      args_str = func_match.group(2)
      args = self._split_args(args_str)
      return self._eval_function(func_name, args)

    # 变量
    if re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', expr):
      return self._get_var(expr)

    # 简单数学表达式 (支持 + - * / 和括号)
    try:
      # 替换变量名
      resolved = self._resolve_variables_in_expr(expr)
      result = eval(resolved, {"__builtins__": {}}, {})
      return result
    except Exception:
      pass

    return expr

  def _resolve_variables_in_expr(self, expr: str) -> str:
    """在表达式中替换变量名为其值"""
    def replace_var(match):
      name = match.group(0)
      val = self._get_var(name)
      if val is None:
        return '0'
      if isinstance(val, bool):
        return '1' if val else '0'
      if isinstance(val, (int, float)):
        return str(val)
      return '0'

    result = re.sub(r'[A-Za-z_][A-Za-z0-9_]*', replace_var, expr)
    return result

  def _eval_function(self, name: str, args: List[str]) -> Any:
    """求值内置函数"""
    if name == 'STRTOVAL':
      s = self._eval_expression(args[0]) if args else ''
      try:
        return float(str(s))
      except (ValueError, TypeError):
        return 0.0

    if name == 'VAL':
      s = self._eval_expression(args[0]) if args else ''
      try:
        return float(str(s))
      except (ValueError, TypeError):
        return 0.0

    if name == 'ABS':
      val = self._eval_expression(args[0]) if args else 0
      return abs(float(val)) if isinstance(val, (int, float)) else 0

    if name == 'SIN':
      val = self._eval_expression(args[0]) if args else 0
      return math.sin(math.radians(float(val))) if isinstance(val, (int, float)) else 0

    if name == 'COS':
      val = self._eval_expression(args[0]) if args else 0
      return math.cos(math.radians(float(val))) if isinstance(val, (int, float)) else 0

    if name == 'SQRT':
      val = self._eval_expression(args[0]) if args else 0
      return math.sqrt(abs(float(val))) if isinstance(val, (int, float)) else 0

    if name == 'ROUND':
      val = self._eval_expression(args[0]) if args else 0
      return round(float(val)) if isinstance(val, (int, float)) else 0

    return 0

  # ==================== 变量操作 ====================

  def _var_exists(self, name: str) -> bool:
    """检查变量是否存在"""
    name = name.strip().rstrip(';')
    for scope in reversed(self._local_vars):
      if name in scope:
        return True
    return name in self._variables

  def _get_var(self, name: str) -> Any:
    """获取变量值"""
    name = name.strip().rstrip(';')
    # 先查局部变量
    for scope in reversed(self._local_vars):
      if name in scope:
        return scope[name].value
    # 再查全局变量
    if name in self._variables:
      return self._variables[name].value
    # 预定义特殊值
    if name.upper() == 'TRUE':
      return True
    if name.upper() == 'FALSE':
      return False
    return 0

  def _get_var_obj(self, name: str) -> Optional[Variable]:
    """获取变量对象"""
    name = name.strip().rstrip(';')
    for scope in reversed(self._local_vars):
      if name in scope:
        return scope[name]
    return self._variables.get(name)

  def _set_var(self, name: str, value: Any, var_type: VarType = VarType.NUM,
               is_pers: bool = False, is_const: bool = False):
    """设置变量值"""
    name = name.strip().rstrip(';')
    # 检查是否为CONST
    if name in self._variables and self._variables[name].is_const:
      print(f"[机器人解释器] 警告: 尝试修改常量 {name}")
      return
    if name in self._variables:
      self._variables[name].value = value
    else:
      self._variables[name] = Variable(
        name=name, var_type=var_type, value=value,
        is_pers=is_pers, is_const=is_const)

  # ==================== 字符串操作 ====================

  def _resolve_str(self, text: str) -> str:
    """解析字符串表达式 (支持字符串连接 := )"""
    text = text.strip().rstrip(';')
    if text.startswith('"') and text.endswith('"'):
      return text[1:-1]
    # 处理字符串连接
    if ':=' in text:
      parts = text.split(':=', 1)
      left = self._resolve_str(parts[0])
      right = self._resolve_str(parts[1])
      return left + right
    # 变量引用
    if re.match(r'^[A-Za-z_][A-Za-z0-9_]*$', text):
      val = self._get_var(text)
      return str(val) if val is not None else ""
    return text

  def _tp_format(self, text: str) -> str:
    """TPWRITE格式化输出"""
    result = text.strip()
    if result.startswith('"') and result.endswith('"'):
      result = result[1:-1]
    # 替换变量
    def replace_var(m):
      name = m.group(1)
      val = self._get_var(name)
      return str(val) if val is not None else ""
    result = re.sub(r'\\([A-Za-z_][A-Za-z0-9_]*)', replace_var, result)
    return result

  # ==================== 指令执行 ====================

  def _execute_instruction(self, inst: Tuple[InstructionType, str],
                           raw_line: str) -> Tuple[bool, str]:
    """执行单条指令"""
    itype, content = inst

    # 注意: 不做全局变量替换，每个指令处理器自行通过 _eval_expression 解析变量
    # 否则会破坏变量名（如 counter := counter + 1 变成 0 := 0 + 1）

    if itype == InstructionType.MOVEJ:
      return self._exec_movej(raw_line)
    elif itype == InstructionType.MOVEL:
      return self._exec_movel(raw_line)
    elif itype == InstructionType.MOVEC:
      return self._exec_movec(raw_line)
    elif itype == InstructionType.MOVEABSJ:
      return self._exec_moveabsj(raw_line)
    elif itype == InstructionType.ASSIGN:
      return self._exec_assign(raw_line)
    elif itype == InstructionType.IF:
      return self._exec_if(raw_line)
    elif itype == InstructionType.ELSEIF:
      return self._exec_elseif(raw_line)
    elif itype == InstructionType.ELSE:
      return self._exec_else()
    elif itype == InstructionType.ENDIF:
      return self._exec_endif()
    elif itype == InstructionType.FOR:
      return self._exec_for(raw_line)
    elif itype == InstructionType.ENDFOR:
      return self._exec_endfor()
    elif itype == InstructionType.WHILE:
      return self._exec_while(raw_line)
    elif itype == InstructionType.ENDWHILE:
      return self._exec_endwhile()
    elif itype == InstructionType.TEST:
      return self._exec_test(raw_line)
    elif itype == InstructionType.CASE:
      return self._exec_case(raw_line)
    elif itype == InstructionType.DEFAULT:
      return self._exec_default()
    elif itype == InstructionType.ENDTEST:
      return self._exec_endtest()
    elif itype == InstructionType.GOTO:
      return self._exec_goto(raw_line)
    elif itype == InstructionType.LABEL:
      return (True, "")
    elif itype == InstructionType.PROCCALL:
      return self._exec_proccall(raw_line)
    elif itype == InstructionType.CALLBYVAR:
      return self._exec_callbyvar(raw_line)
    elif itype == InstructionType.STOP:
      return self._exec_stop()
    elif itype == InstructionType.EXIT:
      return self._exec_exit()
    elif itype == InstructionType.BREAK:
      return self._exec_break()
    elif itype == InstructionType.EXITCYCLE:
      return self._exec_exitcycle()
    elif itype == InstructionType.TPWRITE:
      return self._exec_tpwrite(content)
    elif itype == InstructionType.TPERASE:
      self._tp_buffer.clear()
      return (True, "")
    elif itype == InstructionType.TPREADFK:
      return self._exec_tpreadfk(content)
    elif itype == InstructionType.TPREADNUM:
      return self._exec_tpreadnum(content)
    elif itype == InstructionType.SOCKETCREATE:
      return self._exec_socket_create(raw_line)
    elif itype == InstructionType.SOCKETCONNECT:
      return self._exec_socket_connect(raw_line)
    elif itype == InstructionType.SOCKETSEND:
      return self._exec_socket_send(raw_line)
    elif itype == InstructionType.SOCKETRECEIVE:
      return self._exec_socket_receive(raw_line)
    elif itype == InstructionType.SOCKETCLOSE:
      return self._exec_socket_close(raw_line)
    elif itype == InstructionType.STRTOVAL:
      return self._exec_strtoval(content)
    elif itype == InstructionType.WAIT:
      return self._exec_wait(raw_line)
    elif itype == InstructionType.RESET:
      return self._exec_reset(raw_line)
    elif itype == InstructionType.SET:
      return self._exec_set(raw_line)
    elif itype == InstructionType.PULSE:
      return self._exec_pulse(raw_line)
    elif itype == InstructionType.CLKRESET:
      return (True, "")
    elif itype == InstructionType.CLKSTART:
      return (True, "")
    elif itype == InstructionType.CLKSTOP:
      return (True, "")
    elif itype == InstructionType.COMMENT:
      return (True, "")
    elif itype == InstructionType.CONNECT:
      return self._exec_connect(raw_line)

    return (True, "")

  # ==================== 变量解析 ====================

  def _resolve_line_variables(self, line: str) -> str:
    """解析行中的变量引用"""
    def replace_var(match):
      name = match.group(0)
      if name.upper() in ('TRUE', 'FALSE'):
        return name
      val = self._get_var(name)
      if val is None:
        return name
      if isinstance(val, bool):
        return '1' if val else '0'
      if isinstance(val, (int, float)):
        return str(val)
      if isinstance(val, str):
        return f'"{val}"'
      return name

    result = re.sub(r'[A-Za-z_][A-Za-z0-9_]*', replace_var, line)
    return result

  # ==================== 运动指令 ====================

  def _exec_movej(self, line: str) -> Tuple[bool, str]:
    """执行 MoveJ 指令"""
    args = self._parse_args(line)
    if len(args) < 1:
      return (False, "MoveJ 缺少目标点参数")

    target = self._parse_target(args[0])
    speed = "v100"
    zone = "fine"
    tool = "tool0"

    for arg in args[1:]:
      arg = arg.strip()
      if arg in SPEED_DATA or 'v' in arg.lower():
        speed = arg
      elif arg in ZONE_DATA or 'z' in arg.lower():
        zone = arg
      elif 'tool' in arg.lower():
        tool = arg

    speed_data = self._parse_speed_data(speed)
    zone_data = self._parse_zone_data(zone)

    print(f"[机器人] MoveJ -> [{target.x:.1f}, {target.y:.1f}, {target.z:.1f}] "
          f"v={speed_data.v:.0f} z={zone_data.zone_name}")

    self._canon.STRAIGHT_TRAVERSE(
      EmcPose(PmCartesian(target.x, target.y, target.z)))
    return (True, "")

  def _exec_movel(self, line: str) -> Tuple[bool, str]:
    """执行 MoveL 指令"""
    args = self._parse_args(line)
    if len(args) < 1:
      return (False, "MoveL 缺少目标点参数")

    target = self._parse_target(args[0])
    speed = "v100"
    zone = "fine"
    tool = "tool0"

    for arg in args[1:]:
      arg = arg.strip()
      if arg in SPEED_DATA or 'v' in arg.lower():
        speed = arg
      elif arg in ZONE_DATA or 'z' in arg.lower():
        zone = arg
      elif 'tool' in arg.lower():
        tool = arg

    speed_data = self._parse_speed_data(speed)
    zone_data = self._parse_zone_data(zone)

    print(f"[机器人] MoveL -> [{target.x:.1f}, {target.y:.1f}, {target.z:.1f}] "
          f"v={speed_data.v:.0f} z={zone_data.zone_name}")

    self._canon.STRAIGHT_FEED(
      EmcPose(PmCartesian(target.x, target.y, target.z)))
    return (True, "")

  def _exec_movec(self, line: str) -> Tuple[bool, str]:
    """执行 MoveC 指令 (圆弧)"""
    args = self._parse_args(line)
    if len(args) < 2:
      return (False, "MoveC 缺少中间点或终点参数")

    mid = self._parse_target(args[0])
    end = self._parse_target(args[1])
    speed = "v100"
    zone = "fine"
    tool = "tool0"

    for arg in args[2:]:
      arg = arg.strip()
      if arg in SPEED_DATA or 'v' in arg.lower():
        speed = arg
      elif arg in ZONE_DATA or 'z' in arg.lower():
        zone = arg
      elif 'tool' in arg.lower():
        tool = arg

    speed_data = self._parse_speed_data(speed)

    print(f"[机器人] MoveC -> mid[{mid.x:.1f},{mid.y:.1f},{mid.z:.1f}] "
          f"end[{end.x:.1f},{end.y:.1f},{end.z:.1f}]")

    # 简化圆弧为直线 (真实解释器需要计算圆弧)
    self._canon.STRAIGHT_FEED(
      EmcPose(PmCartesian(end.x, end.y, end.z)))
    return (True, "")

  def _exec_moveabsj(self, line: str) -> Tuple[bool, str]:
    """执行 MoveABSJ 指令 (关节绝对运动)"""
    args = self._parse_args(line)
    if len(args) < 1:
      return (False, "MoveABSJ 缺少目标点参数")

    target = self._parse_joint_target(args[0])
    speed = "v100"
    zone = "fine"

    for arg in args[1:]:
      arg = arg.strip()
      if arg in SPEED_DATA or 'v' in arg.lower():
        speed = arg
      elif arg in ZONE_DATA or 'z' in arg.lower():
        zone = arg

    print(f"[机器人] MoveABSJ -> joints={[f'{j:.1f}' for j in target.joints[:6]]}")

    # 简化: 使用第一个关节值作为Z坐标
    self._canon.STRAIGHT_TRAVERSE(
      EmcPose(PmCartesian(target.joints[0] if len(target.joints) > 0 else 0,
                          target.joints[1] if len(target.joints) > 1 else 0,
                          target.joints[2] if len(target.joints) > 2 else 0)))
    return (True, "")

  # ==================== 赋值 ====================

  def _exec_assign(self, line: str) -> Tuple[bool, str]:
    """执行赋值指令 := """
    parts = line.split(':=', 1)
    if len(parts) != 2:
      return (False, "无效的赋值语句")

    var_name = parts[0].strip().rstrip(';')
    val_expr = parts[1].strip().rstrip(';')

    # 获取或创建变量
    var_obj = self._get_var_obj(var_name)
    if var_obj and var_obj.is_const:
      return (False, f"不能修改常量 {var_name}")

    # 求值右侧表达式
    val = self._eval_expression(val_expr)

    # 检查变量类型并转换
    if var_obj:
      var_type = var_obj.var_type
    else:
      var_type = VarType.NUM
      if isinstance(val, str):
        var_type = VarType.STRING

    self._set_var(var_name, val, var_type)
    print(f"[机器人] {var_name} := {val}")
    return (True, "")

  # ==================== 流程控制 ====================

  def _exec_if(self, line: str) -> Tuple[bool, str]:
    """执行 IF 指令"""
    match = re.search(r'IF\s*\((.+)\)', line)
    if not match:
      match = re.search(r'IF\s+(.+)', line)
    if not match:
      return (False, "无效的IF语句")

    cond_str = match.group(1).strip()
    cond = self._eval_condition(cond_str)

    self._if_stack.append(cond)
    self._skip_stack.append(not cond)

    print(f"[机器人] IF {cond_str} = {cond}")
    return (True, "")

  def _exec_elseif(self, line: str) -> Tuple[bool, str]:
    """执行 ELSEIF 指令"""
    if not self._if_stack:
      return (False, "ELSEIF 没有匹配的IF")

    prev = self._if_stack[-1]

    match = re.search(r'ELSEIF\s*\((.+)\)', line)
    if not match:
      match = re.search(r'ELSEIF\s+(.+)', line)
    if not match:
      return (False, "无效的ELSEIF语句")

    cond_str = match.group(1).strip()

    if prev:
      # 前面的IF/ELSEIF已经满足，跳过
      self._skip_stack[-1] = True
    else:
      # 前面未满足，检查当前条件
      cond = self._eval_condition(cond_str)
      self._if_stack[-1] = cond
      self._skip_stack[-1] = not cond

    return (True, "")

  def _exec_else(self) -> Tuple[bool, str]:
    """执行 ELSE 指令"""
    if not self._if_stack:
      return (False, "ELSE 没有匹配的IF")

    prev = self._if_stack[-1]
    self._if_stack[-1] = not prev
    self._skip_stack[-1] = prev
    return (True, "")

  def _exec_endif(self) -> Tuple[bool, str]:
    """执行 ENDIF 指令"""
    if self._if_stack:
      self._if_stack.pop()
    if self._skip_stack:
      self._skip_stack.pop()
    return (True, "")

  def _exec_for(self, line: str) -> Tuple[bool, str]:
    """执行 FOR 指令: FOR i FROM 1 TO 10 STEP 1"""
    match = re.search(
      r'FOR\s+(\w+)\s+FROM\s+(.+?)\s+TO\s+(.+?)(?:\s+STEP\s+(.+?))?(?:\s*;|$)',
      line, re.IGNORECASE)
    if not match:
      return (False, "无效的FOR语句，格式: FOR var FROM start TO end [STEP step]")

    var_name = match.group(1)
    start_val = float(self._eval_expression(match.group(2)))
    end_val = float(self._eval_expression(match.group(3)))
    step = float(self._eval_expression(match.group(4))) if match.group(4) else 1.0

    self._set_var(var_name, start_val)
    self._loop_stack.append(LoopContext(
      loop_type='for', var_name=var_name,
      start_val=start_val, end_val=end_val, step=step,
      start_line=self._current_line, end_line=-1
    ))

    print(f"[机器人] FOR {var_name}={start_val} TO {end_val} STEP {step}")
    return (True, "")

  def _exec_endfor(self) -> Tuple[bool, str]:
    """执行 ENDFOR 指令"""
    if not self._loop_stack or self._loop_stack[-1].loop_type != 'for':
      return (False, "ENDFOR 没有匹配的FOR")

    ctx = self._loop_stack[-1]
    current_val = self._get_var(ctx.var_name)
    if isinstance(current_val, (int, float)):
      new_val = current_val + ctx.step
      self._set_var(ctx.var_name, new_val)

      # 检查是否继续循环
      if ctx.step > 0 and new_val <= ctx.end_val:
        self._current_line = ctx.start_line
      elif ctx.step < 0 and new_val >= ctx.end_val:
        self._current_line = ctx.start_line
      else:
        self._loop_stack.pop()
    else:
      self._loop_stack.pop()

    return (True, "")

  def _exec_while(self, line: str) -> Tuple[bool, str]:
    """执行 WHILE 指令"""
    match = re.search(r'WHILE\s*\((.+)\)', line)
    if not match:
      match = re.search(r'WHILE\s+(.+)', line)
    if not match:
      return (False, "无效的WHILE语句")

    cond_str = match.group(1).strip()
    cond = self._eval_condition(cond_str)

    if not cond:
      # 条件不满足，跳到ENDWHILE之后
      self._skip_stack.append(True)
      self._loop_stack.append(LoopContext(
        loop_type='while_skip', end_line=-1))
    else:
      self._loop_stack.append(LoopContext(
        loop_type='while', start_line=self._current_line))

    return (True, "")

  def _exec_endwhile(self) -> Tuple[bool, str]:
    """执行 ENDWHILE 指令"""
    if not self._loop_stack:
      return (False, "ENDWHILE 没有匹配的WHILE")

    ctx = self._loop_stack[-1]

    if ctx.loop_type == 'while_skip':
      self._loop_stack.pop()
      if self._skip_stack:
        self._skip_stack.pop()
      return (True, "")

    if ctx.loop_type == 'while':
      # 跳回WHILE行重新检查条件
      self._current_line = ctx.start_line

    return (True, "")

  def _exec_test(self, line: str) -> Tuple[bool, str]:
    """执行 TEST 指令"""
    match = re.search(r'TEST\s*\((.+)\)', line)
    if not match:
      match = re.search(r'TEST\s+(.+)', line)
    if not match:
      return (False, "无效的TEST语句")

    expr_str = match.group(1).strip()
    val = self._eval_expression(expr_str)
    self._loop_stack.append(LoopContext(
      loop_type='test', start_line=self._current_line))
    self._set_var('__test_value__', val)

    print(f"[机器人] TEST {expr_str} = {val}")
    return (True, "")

  def _exec_case(self, line: str) -> Tuple[bool, str]:
    """执行 CASE 指令"""
    match = re.search(r'CASE\s+(.+?):', line)
    if not match:
      return (False, "无效的CASE语句")

    case_str = match.group(1).strip()
    test_val = self._get_var('__test_value__')

    if isinstance(test_val, (int, float)):
      case_val = self._eval_expression(case_str)
      if isinstance(case_val, (int, float)):
        if test_val == case_val:
          # 匹配: 取消跳过
          if self._skip_stack:
            self._skip_stack[-1] = False
        else:
          # 不匹配: 跳过
          if self._skip_stack:
            self._skip_stack[-1] = True
    else:
      if self._skip_stack:
        self._skip_stack[-1] = True

    return (True, "")

  def _exec_default(self) -> Tuple[bool, str]:
    """执行 DEFAULT 指令"""
    # DEFAULT 在所有CASE都不匹配时执行
    if self._skip_stack:
      self._skip_stack[-1] = False
    return (True, "")

  def _exec_endtest(self) -> Tuple[bool, str]:
    """执行 ENDTEST 指令"""
    if self._loop_stack and self._loop_stack[-1].loop_type == 'test':
      self._loop_stack.pop()
    if self._skip_stack:
      self._skip_stack.pop()
    return (True, "")

  def _exec_goto(self, line: str) -> Tuple[bool, str]:
    """执行 GOTO 指令"""
    match = re.search(r'GOTO\s+(\w+)', line)
    if not match:
      return (False, "无效的GOTO语句")

    label = match.group(1)
    if label in self._labels:
      self._current_line = self._labels[label]
      print(f"[机器人] GOTO {label} (行{self._labels[label] + 1})")
    else:
      return (False, f"未定义的标签: {label}")
    return (True, "")

  # ==================== 程序调用 ====================

  def _exec_proccall(self, line: str) -> Tuple[bool, str]:
    """执行过程调用"""
    match = re.match(r'^([A-Za-z_][A-Za-z0-9_]*)', line)
    if not match:
      return (False, "无效的过程调用")

    proc_name = match.group(1)

    # 在文件中查找PROC定义
    proc_line = -1
    for i, file_line in enumerate(self._file_lines):
      stripped = file_line.strip()
      if stripped.startswith(f'PROC {proc_name}(') or stripped.startswith(f'PROC {proc_name} ('):
        proc_line = i
        break

    if proc_line == -1:
      # 可能是内置过程
      print(f"[机器人] 内置过程调用: {proc_name}")
      return (True, "")

    # 保存当前上下文
    ctx = SubContext(
      filename=self._filename,
      line_number=self._current_line,
      proc_name=proc_name)
    self._call_stack.append(ctx)
    self._local_vars.append({})

    # 解析参数
    args = self._parse_args(line)
    # (参数传递简化处理)

    # 跳转到过程定义
    self._current_line = proc_line
    print(f"[机器人] 调用过程 {proc_name} (行{proc_line + 1})")
    return (True, "")

  def _exec_callbyvar(self, line: str) -> Tuple[bool, str]:
    """执行 CALLBYVAR - 通过变量值调用过程"""
    match = re.search(r'CALLBYVAR\s+(\w+)', line)
    if not match:
      return (False, "无效的CALLBYVAR语句")

    var_name = match.group(1)
    proc_name = str(self._get_var(var_name))
    print(f"[机器人] CALLBYVAR {var_name}={proc_name}")
    return self._exec_proccall(f"{proc_name}()")

  # ==================== 程序控制 ====================

  def _exec_stop(self) -> Tuple[bool, str]:
    """执行 STOP 指令"""
    self._canon.PROGRAM_STOP()
    print("[机器人] STOP - 程序停止")
    return (True, "")

  def _exec_exit(self) -> Tuple[bool, str]:
    """执行 EXIT 指令"""
    self._exit_flag = True
    self._canon.PROGRAM_END()
    print("[机器人] EXIT - 退出程序")
    return (True, "")

  def _exec_break(self) -> Tuple[bool, str]:
    """执行 BREAK 指令 - 退出当前循环"""
    self._break_flag = True
    print("[机器人] BREAK - 退出循环")
    return (True, "")

  def _exec_exitcycle(self) -> Tuple[bool, str]:
    """执行 EXITCYCLE 指令 - 退出当前周期"""
    self._exitcycle_flag = True
    print("[机器人] EXITCYCLE")
    return (True, "")

  # ==================== TP功能 ====================

  def _exec_tpwrite(self, content: str) -> Tuple[bool, str]:
    """执行 TPWRITE 指令"""
    match = re.search(r'TPWRITE\s+(.+)', content)
    if match:
      text = self._tp_format(match.group(1))
      self._tp_buffer.append(text)
      print(f"[机器人] TPWRITE: {text}")
    return (True, "")

  def _exec_tpreadfk(self, content: str) -> Tuple[bool, str]:
    """执行 TPREADFK 指令 - 简化版"""
    match = re.search(r'TPREADFK\s+(\w+)', content)
    if match:
      var_name = match.group(1)
      self._set_var(var_name, 1)
    print("[机器人] TPREADFK (简化: 返回1)")
    return (True, "")

  def _exec_tpreadnum(self, content: str) -> Tuple[bool, str]:
    """执行 TPREADNUM 指令 - 简化版"""
    match = re.search(r'TPREADNUM\s+(\w+)', content)
    if match:
      var_name = match.group(1)
      self._set_var(var_name, 0)
    print("[机器人] TPREADNUM (简化: 返回0)")
    return (True, "")

  # ==================== Socket通信 ====================

  def _exec_socket_create(self, line: str) -> Tuple[bool, str]:
    """SOCKETCREATE - 创建Socket"""
    match = re.search(r'SOCKETCREATE\s+"([^"]+)"', line)
    if match:
      name = match.group(1)
      self._sockets[name] = None
      print(f"[机器人] SOCKETCREATE: {name}")
    return (True, "")

  def _exec_socket_connect(self, line: str) -> Tuple[bool, str]:
    """SOCKETCONNECT - 连接"""
    print("[机器人] SOCKETCONNECT (仿真)")
    return (True, "")

  def _exec_socket_send(self, line: str) -> Tuple[bool, str]:
    """SOCKETSEND - 发送"""
    print("[机器人] SOCKETSEND (仿真)")
    return (True, "")

  def _exec_socket_receive(self, line: str) -> Tuple[bool, str]:
    """SOCKETRECEIVE - 接收"""
    match = re.search(r'SOCKETRECEIVE\s+(\w+)', line)
    if match:
      self._set_var(match.group(1), "")
    print("[机器人] SOCKETRECEIVE (仿真)")
    return (True, "")

  def _exec_socket_close(self, line: str) -> Tuple[bool, str]:
    """SOCKETCLOSE - 关闭"""
    print("[机器人] SOCKETCLOSE (仿真)")
    return (True, "")

  # ==================== 其他指令 ====================

  def _exec_strtoval(self, content: str) -> Tuple[bool, str]:
    """STRTOVAL - 字符串转数值"""
    match = re.search(r'STRTOVAL\s*\(\s*(.+?)\s*,\s*(\w+)\s*\)', content)
    if match:
      val = self._eval_expression(match.group(1))
      var_name = match.group(2)
      try:
        self._set_var(var_name, float(str(val)))
      except (ValueError, TypeError):
        self._set_var(var_name, 0.0)
    return (True, "")

  def _exec_wait(self, line: str) -> Tuple[bool, str]:
    """WAIT 指令"""
    match = re.search(r'WAIT\s+(.+)', line)
    if match:
      val = self._eval_expression(match.group(1))
      if isinstance(val, (int, float)):
        import time
        time.sleep(min(val / 1000.0, 10.0))
        print(f"[机器人] WAIT {val}ms")
    return (True, "")

  def _exec_reset(self, line: str) -> Tuple[bool, str]:
    """RESET - 复位数字输出"""
    print("[机器人] RESET (数字输出)")
    return (True, "")

  def _exec_set(self, line: str) -> Tuple[bool, str]:
    """SET - 设置数字输出"""
    print("[机器人] SET (数字输出)")
    return (True, "")

  def _exec_pulse(self, line: str) -> Tuple[bool, str]:
    """PULSEDO - 脉冲输出"""
    print("[机器人] PULSEDO (脉冲输出)")
    return (True, "")

  def _exec_connect(self, line: str) -> Tuple[bool, str]:
    """CONNECT - 连接信号"""
    print("[机器人] CONNECT (信号连接)")
    return (True, "")

  # ==================== 条件求值 ====================

  def _eval_condition(self, cond: str) -> bool:
    """求值条件表达式 (支持 AND, OR, NOT, 比较运算)"""
    cond = cond.strip().rstrip(';')

    # 处理 OR
    if ' OR ' in cond.upper():
      parts = re.split(r'\s+OR\s+', cond, flags=re.IGNORECASE)
      return any(self._eval_condition(p) for p in parts)

    # 处理 AND
    if ' AND ' in cond.upper():
      parts = re.split(r'\s+AND\s+', cond, flags=re.IGNORECASE)
      return all(self._eval_condition(p) for p in parts)

    # 处理 NOT
    if cond.upper().startswith('NOT '):
      return not self._eval_condition(cond[4:])

    # 处理比较运算符
    for op in ('<>', '!=', '<=', '>=', '=', '<', '>'):
      if op in cond:
        parts = cond.split(op, 1)
        left = self._eval_expression(parts[0])
        right = self._eval_expression(parts[1])

        if left is None or right is None:
          return False

        try:
          left_f = float(left) if not isinstance(left, (int, float)) else left
          right_f = float(right) if not isinstance(right, (int, float)) else right
        except (ValueError, TypeError):
          left_f = left
          right_f = right

        if op == '=':
          return left_f == right_f
        elif op == '<>':
          return left_f != right_f
        elif op == '!=':
          return left_f != right_f
        elif op == '<':
          return left_f < right_f
        elif op == '>':
          return left_f > right_f
        elif op == '<=':
          return left_f <= right_f
        elif op == '>=':
          return left_f >= right_f

    # 作为布尔值求值
    val = self._eval_expression(cond)
    if isinstance(val, bool):
      return val
    if isinstance(val, (int, float)):
      return val != 0
    return False

  # ==================== COMPACTIF ====================

  def _handle_compact_if(self, line: str) -> Tuple[bool, str]:
    """处理COMPACTIF模式下的行"""
    self._compact_if = False
    # 重新解析该行（去掉COMPACT前缀）
    inst = self._parse_instruction(line)
    if inst:
      return self._execute_instruction(inst, line)
    return (True, "")

  # ==================== 状态查询 ====================

  def get_position(self) -> Tuple[float, float, float]:
    """获取当前位置"""
    if self._canon.commands:
      for cmd in reversed(self._canon.commands):
        if cmd.command_name in ('STRAIGHT_TRAVERSE', 'STRAIGHT_FEED'):
          end = cmd.params.get('end')
          if isinstance(end, EmcPose):
            return (end.tran.x, end.tran.y, end.tran.z)
    return (0.0, 0.0, 0.0)

  def get_status(self) -> Dict[str, Any]:
    """获取状态"""
    pos = self.get_position()
    return {
      'position': pos,
      'variables': {k: v.value for k, v in self._variables.items()
                    if not k.startswith('_') and k not in SPEED_DATA and
                    k not in ZONE_DATA and k not in TOOL_DATA},
      'call_stack_depth': len(self._call_stack),
      'loop_stack_depth': len(self._loop_stack),
      'tp_buffer': list(self._tp_buffer),
    }
