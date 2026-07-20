"""RS274 G代码解释器 - 移植自 LinuxCNC RS274NGC

核心功能：
- 完整的词法/语法分析
- G0/G1/G2/G3 运动指令
- G4 暂停, G10 坐标设置, G28/G30 回参考点
- G17-G19 平面选择, G20/G21 单位, G40-G42 刀具补偿
- G53-G59.3 坐标系, G90/G91 距离模式, G92 偏移
- G73/G81-G89 固定循环
- M0-M9 辅助功能
- O-word 子程序控制流 (sub/call/if/while/repeat)
- 参数系统 (#1-#5400, #<命名参数>)
"""
import math
import re
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional, Any

from .canon import (CanonCommand, CanonCoordSystem, CanonMotionMode,
                    CanonPlane, CanonUnits, CanonCutterComp)
from .coordinates import EmcPose, PmCartesian
from .tool_table import ToolTable, ToolData, createDefaultToolTable
from .work_offset import WorkOffsetManager, CoordSystemId


class MotionMode:
    NONE = -1
    G0 = 0
    G1 = 1
    G2 = 2
    G3 = 3


class DistanceMode:
    ABSOLUTE = 90
    INCREMENTAL = 91


class FeedMode:
    UNITS_PER_MINUTE = 94
    INVERSE_TIME = 93
    UNITS_PER_REVOLUTION = 95


class SpindleState:
    OFF = 0
    CW = 1
    CCW = 2


class OWordType:
    SUB = 1
    ENDSUB = 2
    CALL = 3
    DO = 4
    WHILE_CONDITION = 5
    ENDWHILE = 6
    IF = 7
    ELSEIF = 8
    ELSE = 9
    ENDIF = 10
    REPEAT = 11
    ENDREPEAT = 12
    BREAK = 13
    CONTINUE = 14
    RETURN = 15
    GOTO = 16


@dataclass
class Block:
    """一行G代码的解析结果"""
    line_text: str = ""
    g_modes: Dict[int, int] = field(default_factory=dict)
    m_modes: List[int] = field(default_factory=list)
    x: Optional[float] = None
    y: Optional[float] = None
    z: Optional[float] = None
    a: Optional[float] = None
    b: Optional[float] = None
    c: Optional[float] = None
    i: Optional[float] = None
    j: Optional[float] = None
    k: Optional[float] = None
    r: Optional[float] = None
    f: Optional[float] = None
    s: Optional[float] = None
    t: Optional[int] = None
    p: Optional[float] = None
    q: Optional[float] = None
    l: Optional[int] = None
    h: Optional[int] = None
    d: Optional[int] = None
    n: int = 0
    motion_to_be: int = MotionMode.NONE
    comment: str = ""
    o_type: Optional[int] = None
    o_num: Optional[int] = None
    o_name: Optional[str] = None
    param_assigns: List[Tuple[int, float]] = field(default_factory=list)
    x_flag: bool = False
    y_flag: bool = False
    z_flag: bool = False
    e: Optional[float] = None
    turn: int = 0


# G代码模态组映射: G代码号 -> 组号
G_CODE_GROUP = {
    0: 1, 1: 1, 2: 1, 3: 1, 33: 1, 38: 1, 73: 1, 80: 1, 81: 1, 82: 1,
    83: 1, 84: 1, 85: 1, 86: 1, 87: 1, 88: 1, 89: 1,
    4: 1, 10: 1, 28: 1, 30: 1, 53: 1, 92: 1,
    17: 2, 18: 2, 19: 2,
    90: 3, 91: 3,
    93: 5, 94: 5, 95: 5,
    20: 6, 21: 6,
    40: 7, 41: 7, 42: 7,
    43: 8, 49: 8,
    98: 9, 99: 9,
    61: 10, 64: 10,
    54: 12, 55: 12, 56: 12, 57: 12, 58: 12, 59: 12,
    591: 12, 592: 12, 593: 12,
    901: 14, 911: 14,
    96: 16, 97: 16,
}

# 组1中哪些是运动G代码
MOTION_G_CODES = {0, 1, 2, 3, 33, 38, 73, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89}

# O-word关键字映射
OWORD_KEYWORDS = {
    'sub': OWordType.SUB,
    'endsub': OWordType.ENDSUB,
    'call': OWordType.CALL,
    'do': OWordType.DO,
    'while': OWordType.WHILE_CONDITION,
    'endwhile': OWordType.ENDWHILE,
    'if': OWordType.IF,
    'elseif': OWordType.ELSEIF,
    'else': OWordType.ELSE,
    'endif': OWordType.ENDIF,
    'repeat': OWordType.REPEAT,
    'endrepeat': OWordType.ENDREPEAT,
    'break': OWordType.BREAK,
    'continue': OWordType.CONTINUE,
    'return': OWordType.RETURN,
    'goto': OWordType.GOTO,
}


@dataclass
class SubContext:
    """子程序调用上下文"""
    filename: str = ""
    line_number: int = 0
    sub_name: int = 0
    saved_params: Dict[int, float] = field(default_factory=dict)
    saved_g_codes: List[int] = field(default_factory=list)
    saved_m_codes: List[int] = field(default_factory=list)
    loop_counter: int = 0
    repeat_end_line: int = 0


@dataclass
class Setup:
    """解释器全局状态"""
    current_x: float = 0.0
    current_y: float = 0.0
    current_z: float = 0.0
    current_a: float = 0.0
    current_b: float = 0.0
    current_c: float = 0.0
    program_x: float = 0.0
    program_y: float = 0.0
    program_z: float = 0.0
    coord_system: int = 54
    origin_offset_x: float = 0.0
    origin_offset_y: float = 0.0
    origin_offset_z: float = 0.0
    g92_offset_x: float = 0.0
    g92_offset_y: float = 0.0
    g92_offset_z: float = 0.0
    g92_offset_a: float = 0.0
    g92_offset_b: float = 0.0
    g92_offset_c: float = 0.0
    rotation_xy: float = 0.0
    distance_mode: int = DistanceMode.ABSOLUTE
    ijk_distance_mode: int = 90
    feed_rate: float = 0.0
    feed_mode: int = FeedMode.UNITS_PER_MINUTE
    feed_override: bool = True
    spindle_speed: float = 0.0
    spindle_turning: int = SpindleState.OFF
    spindle_override: bool = True
    motion_mode: int = MotionMode.NONE
    motion_control_mode: int = 64
    retract_mode: int = 99
    plane: int = 17
    length_units: int = 21
    current_tool: int = 0
    tool_offset_z: float = 0.0
    cutter_comp_side: int = 0
    cutter_comp_radius: float = 0.0
    mist: bool = False
    flood: bool = False
    block_delete: bool = False
    optional_stop: bool = False
    program_running: bool = False
    parameters: Dict[int, float] = field(default_factory=dict)
    subroutines: Dict[int, List[str]] = field(default_factory=dict)
    sub_call_stack: List[SubContext] = field(default_factory=list)
    local_params: List[Dict[int, float]] = field(default_factory=list)
    active_g_codes: List[int] = field(default_factory=lambda: [0] * 20)
    active_m_codes: List[int] = field(default_factory=lambda: [0] * 16)
    active_settings: List[float] = field(default_factory=lambda: [0.0] * 4)
    coord_offsets: Dict[int, List[float]] = field(default_factory=lambda: {
        54: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        55: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        56: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        57: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        58: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        59: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        591: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        592: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        593: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    })
    percent_flag: bool = False
    cycle_active: bool = False
    in_sub_definition: bool = False
    current_sub_name: Optional[int] = None
    sub_lines: List[str] = field(default_factory=list)
    skipping: bool = False
    skipping_to: Optional[str] = None
    if_condition: Optional[bool] = None
    if_stack: List[Optional[bool]] = field(default_factory=list)
    while_stack: List[Tuple[int, bool]] = field(default_factory=list)
    repeat_stack: List[Tuple[int, int, int]] = field(default_factory=list)
    break_flag: bool = False
    continue_flag: bool = False


class Interpreter:
    """RS274 G代码解释器"""

    def __init__(self):
        self._setup = Setup()
        self._canon = CanonCommand()
        self._error_msg = ""
        self._file_lines: List[str] = []
        self._current_line: int = 0
        self._filename: str = ""
        self._line_num: int = 0
        # 刀具表和工件坐标系管理器
        self._toolTable = createDefaultToolTable()
        self._workOffset = WorkOffsetManager()

    @property
    def toolTable(self) -> ToolTable:
        return self._toolTable

    @property
    def workOffset(self) -> WorkOffsetManager:
        return self._workOffset

    @property
    def canon(self) -> CanonCommand:
        return self._canon

    @property
    def setup(self) -> Setup:
        return self._setup

    @property
    def error(self) -> str:
        return self._error_msg

    def init(self):
        self._setup = Setup()
        self._canon.clear()
        self._canon.INIT_CANON()
        for i in range(20):
            self._setup.active_g_codes[i] = 0
        for i in range(16):
            self._setup.active_m_codes[i] = 0

    def open_file(self, filename: str) -> Tuple[bool, str]:
        try:
            with open(filename, 'r', encoding='utf-8', errors='replace') as f:
                self._file_lines = f.readlines()
            self._current_line = 0
            self._filename = filename
            return (True, "")
        except Exception as e:
            self._error_msg = f"无法打开文件: {e}"
            return (False, self._error_msg)

    def read_line(self) -> Tuple[Optional[str], int]:
        if self._current_line >= len(self._file_lines):
            return (None, -1)
        line = self._file_lines[self._current_line].rstrip('\n\r')
        num = self._current_line
        self._current_line += 1
        return (line, num)

    def close(self):
        self._file_lines = []
        self._current_line = 0

    def execute(self, line: str) -> Tuple[bool, str]:
        self._error_msg = ""
        self._line_num += 1
        block = self._parse_line(line)
        if block is None:
            return (False, self._error_msg)

        if block.o_type is not None:
            return self._execute_oword(block)

        # 存储子程序定义中的非O-word行（不执行）
        if self._setup.in_sub_definition and self._setup.current_sub_name is not None:
            self._setup.sub_lines.append(line)
            return (True, "")

        # 跳过被 if/while 等跳过的行
        if self._setup.skipping:
            return (True, "")

        for param_num, param_val in block.param_assigns:
            self._set_parameter(param_num, param_val)

        # T代码选刀（在M6之前执行）
        if block.t is not None and block.t > 0:
            self._toolTable.selectTool(block.t)

        self._convert_m(block)
        self._convert_g(block)

        if block.motion_to_be != MotionMode.NONE:
            self._convert_motion(block)
        elif self._setup.motion_mode != MotionMode.NONE:
            # 模态运动回退: 没有指定运动G代码时使用上一次的模态
            block.motion_to_be = self._setup.motion_mode
            self._convert_motion(block)

        self._write_active_codes(block)
        return (True, "")

    def execute_file(self) -> Tuple[bool, str]:
        while True:
            result = self.read_line()
            if result[0] is None:
                break
            line, num = result
            line = line.strip()
            if not line or line.startswith('(') or line.startswith(';'):
                continue
            if line.startswith('%'):
                self._setup.percent_flag = not self._setup.percent_flag
                continue
            ok, err = self.execute(line)
            if not ok:
                return (False, f"行{num + 1}: {err}")
        return (True, "")

    def reset(self):
        self._canon.clear()
        self._canon.INIT_CANON()
        self._setup.current_x = 0.0
        self._setup.current_y = 0.0
        self._setup.current_z = 0.0
        self._setup.program_x = 0.0
        self._setup.program_y = 0.0
        self._setup.program_z = 0.0
        self._setup.motion_mode = MotionMode.NONE
        self._setup.feed_rate = 0.0
        self._setup.distance_mode = DistanceMode.ABSOLUTE
        self._setup.coord_system = 54
        self._setup.g92_offset_x = 0.0
        self._setup.g92_offset_y = 0.0
        self._setup.g92_offset_z = 0.0
        self._setup.spindle_turning = SpindleState.OFF
        self._setup.mist = False
        self._setup.flood = False
        self._setup.program_running = False
        self._setup.sub_call_stack.clear()
        self._setup.local_params.clear()
        self._setup.if_stack.clear()
        self._setup.while_stack.clear()
        self._setup.repeat_stack.clear()
        self._setup.skipping = False
        self._line_num = 0

    # ==================== 词法分析 ====================

    def _parse_line(self, line: str) -> Optional[Block]:
        block = Block(line_text=line)
        work = self._remove_comments(line)
        if not work or work.isspace():
            return block

        pos = 0
        while pos < len(work):
            pos = self._skip_space(work, pos)
            if pos >= len(work):
                break
            ch = work[pos]
            if ch == '/':
                if self._setup.block_delete:
                    return block
                pos += 1
                continue
            if ch.isalpha():
                word_result = self._read_word(work, pos)
                if word_result is None:
                    break
                letter, value, pos = word_result
                if not self._assign_word(block, letter, value, work, pos):
                    break
            elif ch == '#':
                result = self._read_param_assign(work, pos)
                if result is None:
                    break
                assigns, pos = result
                block.param_assigns.extend(assigns)
            else:
                pos += 1
        return block

    def _remove_comments(self, line: str) -> str:
        result = []
        i = 0
        depth = 0
        while i < len(line):
            ch = line[i]
            if ch == '(':
                depth += 1
            elif ch == ')' and depth > 0:
                depth -= 1
            elif depth == 0:
                if ch == ';':
                    break
                result.append(ch)
            i += 1
        return ''.join(result)

    def _skip_space(self, line: str, pos: int) -> int:
        while pos < len(line) and line[pos] in ' \t':
            pos += 1
        return pos

    def _read_word(self, line: str, pos: int) -> Optional[Tuple[str, float, int]]:
        if pos >= len(line):
            return None
        letter = line[pos].upper()
        pos += 1
        value, pos = self._read_number(line, pos)
        if value is None:
            # 尝试读取表达式 [#expr] 或参数 #n
            if pos < len(line) and line[pos] == '[':
                value, pos = self._evaluate_expression(line, pos)
            elif pos < len(line) and line[pos] == '#':
                value, pos = self._read_param_value(line, pos)
            if value is None:
                value = 0.0
        return (letter, value, pos)

    def _read_number(self, line: str, pos: int) -> Tuple[Optional[float], int]:
        start = pos
        if pos < len(line) and line[pos] in '+-':
            pos += 1
        while pos < len(line) and (line[pos].isdigit() or line[pos] == '.'):
            pos += 1
        if pos < len(line) and line[pos] in 'eE':
            pos += 1
            if pos < len(line) and line[pos] in '+-':
                pos += 1
            while pos < len(line) and line[pos].isdigit():
                pos += 1
        if pos == start or (pos == start + 1 and line[start] in '+-'):
            return (None, start)
        try:
            return (float(line[start:pos]), pos)
        except ValueError:
            return (None, start)

    def _read_param_value(self, line: str, pos: int) -> Tuple[Optional[float], int]:
        """读取 #n 参数值"""
        if pos >= len(line) or line[pos] != '#':
            return (None, pos)
        pos += 1
        if pos < len(line) and line[pos] == '<':
            pos += 1
            end = line.find('>', pos)
            if end == -1:
                return (None, pos)
            name = line[pos:end].strip()
            pos = end + 1
            val = self._get_named_parameter(name)
            return (val, pos)
        num_val, pos = self._read_number(line, pos)
        if num_val is None:
            return (None, pos)
        param_num = int(num_val)
        val = self._get_parameter(param_num)
        return (val, pos)

    def _evaluate_expression(self, line: str, pos: int) -> Tuple[Optional[float], int]:
        """计算 [...] 表达式"""
        if pos >= len(line) or line[pos] != '[':
            return (None, pos)
        pos += 1
        pos = self._skip_space(line, pos)
        result, pos = self._eval_expr(line, pos)
        if pos < len(line) and line[pos] == ']':
            pos += 1
        return (result, pos)

    def _eval_expr(self, line: str, pos: int) -> Tuple[Optional[float], int]:
        val, pos = self._eval_term(line, pos)
        if val is None:
            return (None, pos)
        while pos < len(line):
            pos = self._skip_space(line, pos)
            if pos < len(line) and line[pos] in '+-':
                op = line[pos]
                pos += 1
                right, pos = self._eval_term(line, pos)
                if right is None:
                    return (val, pos)
                val = val + right if op == '+' else val - right
            else:
                break
        return (val, pos)

    def _eval_term(self, line: str, pos: int) -> Tuple[Optional[float], int]:
        val, pos = self._eval_factor(line, pos)
        if val is None:
            return (None, pos)
        while pos < len(line):
            pos = self._skip_space(line, pos)
            if pos < len(line) and line[pos] in '*/':
                op = line[pos]
                pos += 1
                right, pos = self._eval_factor(line, pos)
                if right is None:
                    return (val, pos)
                val = val * right if op == '*' else val / right if right != 0 else float('inf')
            else:
                break
        return (val, pos)

    def _eval_factor(self, line: str, pos: int) -> Tuple[Optional[float], int]:
        pos = self._skip_space(line, pos)
        if pos >= len(line):
            return (None, pos)
        ch = line[pos]
        if ch == '-':
            pos += 1
            val, pos = self._eval_factor(line, pos)
            return (-val if val is not None else None, pos)
        if ch == '+':
            pos += 1
            return self._eval_factor(line, pos)
        if ch == '[':
            return self._evaluate_expression(line, pos)
        if ch == '#':
            return self._read_param_value(line, pos)
        if ch == '(':
            pos += 1
            val, pos = self._eval_expr(line, pos)
            pos = self._skip_space(line, pos)
            if pos < len(line) and line[pos] == ')':
                pos += 1
            return (val, pos)
        if ch.isalpha():
            return self._eval_unary_func(line, pos)
        num, pos = self._read_number(line, pos)
        return (num, pos)

    def _eval_unary_func(self, line: str, pos: int) -> Tuple[Optional[float], int]:
        start = pos
        while pos < len(line) and line[pos].isalpha():
            pos += 1
        name = line[start:pos].lower()
        pos = self._skip_space(line, pos)
        if pos < len(line) and line[pos] == '[':
            arg, pos = self._evaluate_expression(line, pos)
        elif pos < len(line) and line[pos] == '(':
            pos += 1
            arg, pos = self._eval_expr(line, pos)
            if pos < len(line) and line[pos] == ')':
                pos += 1
        else:
            arg, pos = self._eval_factor(line, pos)
        if arg is None:
            return (None, pos)
        funcs = {
            'sin': math.sin(math.radians(arg)),
            'cos': math.cos(math.radians(arg)),
            'tan': math.tan(math.radians(arg)),
            'asin': math.degrees(math.asin(arg)) if abs(arg) <= 1 else 0,
            'acos': math.degrees(math.acos(arg)) if abs(arg) <= 1 else 0,
            'atan': math.degrees(math.atan(arg)),
            'sqrt': math.sqrt(abs(arg)),
            'abs': abs(arg),
            'exp': math.exp(arg),
            'ln': math.log(arg) if arg > 0 else 0,
            'round': round(arg),
            'fix': math.floor(arg),
            'fup': math.ceil(arg),
        }
        if name in funcs:
            return (funcs[name], pos)
        return (arg, pos)

    def _read_param_assign(self, line: str, pos: int) -> Optional[Tuple[List, int]]:
        assigns = []
        while pos < len(line) and line[pos] == '#':
            pos += 1
            if pos < len(line) and line[pos] == '<':
                pos += 1
                end = line.find('>', pos)
                if end == -1:
                    return None
                name = line[pos:end].strip()
                pos = end + 1
                pos = self._skip_space(line, pos)
                if pos < len(line) and line[pos] == '=':
                    pos += 1
                    val, pos = self._eval_expr(line, pos)
                    param_num = self._hash_named_param(name)
                    self._set_parameter(param_num, val or 0.0)
                    assigns.append((param_num, val or 0.0))
            else:
                num_val, new_pos = self._read_number(line, pos)
                if num_val is None:
                    return None
                pos = new_pos
                param_num = int(num_val)
                pos = self._skip_space(line, pos)
                if pos < len(line) and line[pos] == '=':
                    pos += 1
                    val, pos = self._eval_expr(line, pos)
                    self._set_parameter(param_num, val or 0.0)
                    assigns.append((param_num, val or 0.0))
        return (assigns, pos)

    def _assign_word(self, block: Block, letter: str, value: float, line: str, pos: int) -> bool:
        letter = letter.upper()
        if letter == 'N':
            block.n = int(value)
        elif letter == 'G':
            gcode = int(value)
            if gcode == 591:
                gcode = 591
            elif gcode == 592:
                gcode = 592
            elif gcode == 593:
                gcode = 593
            elif gcode == 901:
                gcode = 901
            elif gcode == 911:
                gcode = 911
            elif gcode == 611:
                gcode = 611
            group = G_CODE_GROUP.get(gcode, 0)
            if group > 0:
                block.g_modes[group] = gcode
            if gcode in MOTION_G_CODES:
                block.motion_to_be = gcode
        elif letter == 'M':
            mcode = int(value)
            block.m_modes.append(mcode)
        elif letter == 'X':
            block.x = value; block.x_flag = True
        elif letter == 'Y':
            block.y = value; block.y_flag = True
        elif letter == 'Z':
            block.z = value; block.z_flag = True
        elif letter == 'A':
            block.a = value
        elif letter == 'B':
            block.b = value
        elif letter == 'C':
            block.c = value
        elif letter == 'I':
            block.i = value
        elif letter == 'J':
            block.j = value
        elif letter == 'K':
            block.k = value
        elif letter == 'R':
            block.r = value
        elif letter == 'F':
            block.f = value
        elif letter == 'S':
            block.s = value
        elif letter == 'T':
            block.t = int(value)
        elif letter == 'P':
            block.p = value
        elif letter == 'Q':
            block.q = value
        elif letter == 'L':
            block.l = int(value) if value == int(value) else int(value)
        elif letter == 'H':
            block.h = int(value)
        elif letter == 'D':
            block.d = int(value)
        elif letter == 'E':
            block.e = value
        elif letter == 'O':
            block.o_num = int(value)
            pos_after = self._skip_space(line, pos)
            if pos_after < len(line):
                rest = line[pos_after:].strip().lower()
                for kw, ot in OWORD_KEYWORDS.items():
                    if rest.startswith(kw):
                        block.o_type = ot
                        if ot in (OWordType.WHILE_CONDITION, OWordType.IF, OWordType.ELSEIF):
                            # 条件表达式在关键字之后
                            cond_pos = pos_after + len(kw)
                            cond_pos = self._skip_space(line, cond_pos)
                            # 查找 [...] 或直接读取表达式
                            if cond_pos < len(line) and line[cond_pos] == '[':
                                cond_val, _ = self._evaluate_expression(line, cond_pos)
                            elif cond_pos < len(line) and line[cond_pos] == '(':
                                cond_val, _ = self._eval_expr(line, cond_pos + 1)
                            else:
                                cond_val, _ = self._eval_expr(line, cond_pos)
                            block.p = cond_val if cond_val is not None else 0.0
                        break
        return True

    # ==================== 参数系统 ====================

    def _get_parameter(self, num: int) -> float:
        if num in self._setup.parameters:
            return self._setup.parameters[num]
        return 0.0

    def _set_parameter(self, num: int, val: float):
        self._setup.parameters[num] = val

    def _get_named_parameter(self, name: str) -> float:
        if name.startswith('_'):
            key = self._hash_named_param(name)
            return self._get_parameter(key)
        named_map = {
            'current_x': self._setup.current_x,
            'current_y': self._setup.current_y,
            'current_z': self._setup.current_z,
            'feed_rate': self._setup.feed_rate,
            'spindle_speed': self._setup.spindle_speed,
        }
        return named_map.get(name, 0.0)

    def _hash_named_param(self, name: str) -> int:
        h = 5381
        for ch in name:
            h = (h * 33 + ord(ch)) & 0x7FFFFFFF
        return 10000 + (h % 4000)

    # ==================== G代码转换 ====================

    def _convert_g(self, block: Block):
        for group, gcode in block.g_modes.items():
            if group == 1:
                if gcode in (28, 30, 53, 92, 10):
                    if gcode == 28:
                        self._convert_home()
                    elif gcode == 30:
                        self._convert_home2()
                    elif gcode == 53:
                        pass
                    elif gcode == 92:
                        self._convert_axis_offsets(block)
                    elif gcode == 10:
                        self._convert_set_tool_table(block)
                elif gcode in MOTION_G_CODES and gcode not in (28, 30, 53, 92, 10):
                    self._setup.motion_mode = gcode
            elif group == 2:
                self._convert_set_plane(gcode)
            elif group == 3:
                self._convert_distance_mode(gcode)
            elif group == 5:
                self._convert_feed_mode(gcode)
            elif group == 6:
                self._convert_length_units(gcode)
            elif group == 7:
                self._convert_cutter_comp(gcode)
            elif group == 8:
                self._convert_tool_length_offset(gcode, block)
            elif group == 9:
                self._setup.retract_mode = gcode
            elif group == 10:
                self._convert_control_mode(gcode)
            elif group == 12:
                self._convert_coordinate_system(gcode)
            elif group == 14:
                self._setup.ijk_distance_mode = gcode
            elif group == 16:
                self._convert_spindle_mode(gcode)

    def _convert_set_plane(self, code: int):
        self._setup.plane = code
        plane_map = {17: CanonPlane.XY, 18: CanonPlane.XZ, 19: CanonPlane.YZ}
        self._canon.SELECT_PLANE(plane_map.get(code, CanonPlane.XY))

    def _convert_length_units(self, code: int):
        self._setup.length_units = code
        units_map = {20: CanonUnits.INCH, 21: CanonUnits.MM}
        self._canon.USE_LENGTH_UNITS(units_map.get(code, CanonUnits.MM))

    def _convert_distance_mode(self, code: int):
        self._setup.distance_mode = code

    def _convert_feed_mode(self, code: int):
        self._setup.feed_mode = code
        self._canon.SET_FEED_MODE(code)

    def _convert_control_mode(self, code: int):
        self._setup.motion_control_mode = code
        mode_map = {61: CanonMotionMode.EXACT_STOP, 64: CanonMotionMode.CONTINUOUS}
        self._canon.SET_MOTION_CONTROL_MODE(mode_map.get(code, CanonMotionMode.CONTINUOUS))

    def _convert_cutter_comp(self, code: int):
        self._setup.cutter_comp_side = code
        if code == 40:
            self._canon.STOP_CUTTER_RADIUS_COMPENSATION()
        elif code in (41, 42):
            side = CanonCutterComp.LEFT if code == 41 else CanonCutterComp.RIGHT
            self._canon.START_CUTTER_RADIUS_COMPENSATION(side)

    def _convert_tool_length_offset(self, code: int, block: Block = None):
        if code == 49:
            # G49 取消刀具长度补偿
            self._setup.tool_offset_z = 0.0
            self._canon.USE_TOOL_LENGTH_OFFSET(None)
        elif code == 43:
            # G43 H<num> 应用刀具长度补偿
            h_num = block.h if block and block.h else self._setup.current_tool
            tool_data = self._toolTable.getTool(h_num)
            if tool_data:
                self._setup.tool_offset_z = tool_data.z_offset
                offset = EmcPose(tran=PmCartesian(
                    tool_data.x_offset, tool_data.y_offset, tool_data.z_offset),
                    a=tool_data.a_offset, b=tool_data.b_offset, c=tool_data.c_offset)
                self._canon.USE_TOOL_LENGTH_OFFSET(offset)
            else:
                self._canon.USE_TOOL_LENGTH_OFFSET(None)

    def _convert_coordinate_system(self, code: int):
        if code == 53:
            # G53 机器坐标系 - 暂不完整实现
            pass
        elif code in (54, 55, 56, 57, 58, 59, 591, 592, 593):
            # G54-G59.3 工件坐标系切换
            self._setup.coord_system = code
            self._workOffset.currentCoord = code
            # 加载该坐标系的原点偏移
            offset = self._workOffset.getOffset(code)
            self._setup.origin_offset_x = offset.x
            self._setup.origin_offset_y = offset.y
            self._setup.origin_offset_z = offset.z
            self._setup.rotation_xy = offset.rotation
            # 通知Canon设置G5X偏移
            self._canon.SET_G5X_OFFSET(EmcPose(
                tran=PmCartesian(offset.x, offset.y, offset.z),
                a=offset.a, b=offset.b, c=offset.c))
            print(f"[解释器] 坐标系切换到 G{code}: "
                  f"X={offset.x:.3f} Y={offset.y:.3f} Z={offset.z:.3f}")
        elif code == 921:
            # G92.1 清除G92偏移
            self._workOffset.clearG92()
            self._setup.g92_offset_x = 0.0
            self._setup.g92_offset_y = 0.0
            self._setup.g92_offset_z = 0.0
            self._setup.g92_offset_a = 0.0
            self._setup.g92_offset_b = 0.0
            self._setup.g92_offset_c = 0.0
        elif code == 922:
            # G92.2 清除G92偏移（保留参数）
            self._setup.g92_offset_x = 0.0
            self._setup.g92_offset_y = 0.0
            self._setup.g92_offset_z = 0.0
            self._setup.g92_offset_a = 0.0
            self._setup.g92_offset_b = 0.0
            self._setup.g92_offset_c = 0.0
        elif code == 923:
            # G92.3 恢复G92偏移（从参数）
            g92 = self._workOffset.g92Offset
            self._setup.g92_offset_x = g92.x
            self._setup.g92_offset_y = g92.y
            self._setup.g92_offset_z = g92.z

    def _convert_spindle_mode(self, code: int):
        if code == 96:
            self._canon.MESSAGE("G96 恒线速模式")
        elif code == 97:
            self._canon.MESSAGE("G97 取消恒线速")

    def _convert_home(self):
        self._setup.current_x = 0.0
        self._setup.current_y = 0.0
        self._setup.current_z = 0.0
        self._canon.STRAIGHT_TRAVERSE(
            EmcPose(PmCartesian(0, 0, self._setup.current_z)))

    def _convert_home2(self):
        self._setup.current_x = 0.0
        self._setup.current_y = 0.0
        self._setup.current_z = 0.0
        self._canon.STRAIGHT_TRAVERSE(
            EmcPose(PmCartesian(0, 0, self._setup.current_z)))

    def _convert_axis_offsets(self, block: Block):
        if block.x is not None:
            self._setup.g92_offset_x = self._setup.current_x - block.x - self._get_coord_offset(0)
        if block.y is not None:
            self._setup.g92_offset_y = self._setup.current_y - block.y - self._get_coord_offset(1)
        if block.z is not None:
            self._setup.g92_offset_z = self._setup.current_z - block.z - self._get_coord_offset(2)
        self._canon.SET_G92_OFFSET(
            EmcPose(PmCartesian(self._setup.g92_offset_x, self._setup.g92_offset_y, self._setup.g92_offset_z)))

    def _convert_set_tool_table(self, block: Block):
        """G10 L1/L2/L20 设置刀具/坐标系参数"""
        l_val = block.l
        p_val = int(block.p) if block.p is not None else 1

        if l_val == 1 or l_val == 10 or l_val == 11:
            # G10 L1/Pn: 设置刀具参数
            axes_dict = {}
            if block.x is not None: axes_dict['x_offset'] = block.x
            if block.y is not None: axes_dict['y_offset'] = block.y
            if block.z is not None: axes_dict['z_offset'] = block.z
            if block.a is not None: axes_dict['a_offset'] = block.a
            if block.b is not None: axes_dict['b_offset'] = block.b
            if block.c is not None: axes_dict['c_offset'] = block.c
            if block.r is not None: axes_dict['diameter'] = block.r * 2
            if block.q is not None: axes_dict['front_angle'] = block.q
            self._toolTable.setToolParameter(p_val, pocket=p_val, **axes_dict)

        elif l_val == 2:
            # G10 L2/Pn: 设置坐标系原点偏移
            axes_dict = {}
            if block.x is not None: axes_dict['x'] = block.x
            if block.y is not None: axes_dict['y'] = block.y
            if block.z is not None: axes_dict['z'] = block.z
            if block.a is not None: axes_dict['a'] = block.a
            if block.b is not None: axes_dict['b'] = block.b
            if block.c is not None: axes_dict['c'] = block.c
            self._workOffset.setToolTable(
                self._setup.coord_system, p_val, axes_dict)

            # 如果设置的是当前坐标系，更新setup偏移
            p_to_cs = {1: 54, 2: 55, 3: 56, 4: 57, 5: 58,
                       6: 59, 7: 591, 8: 592, 9: 593}
            target = p_to_cs.get(p_val, self._setup.coord_system)
            if target == self._setup.coord_system:
                offset = self._workOffset.getCurrentOffset()
                self._setup.origin_offset_x = offset.x
                self._setup.origin_offset_y = offset.y
                self._setup.origin_offset_z = offset.z
                self._canon.SET_G5X_OFFSET(EmcPose(
                    tran=PmCartesian(offset.x, offset.y, offset.z),
                    a=offset.a, b=offset.b, c=offset.c))

        elif l_val == 20:
            # G10 L20/Pn: 相对当前位置设置坐标系原点
            axes_dict = {}
            if block.x is not None: axes_dict['x'] = block.x
            if block.y is not None: axes_dict['y'] = block.y
            if block.z is not None: axes_dict['z'] = block.z
            if block.a is not None: axes_dict['a'] = block.a
            if block.b is not None: axes_dict['b'] = block.b
            if block.c is not None: axes_dict['c'] = block.c
            current_pos = {
                'X': self._setup.current_x,
                'Y': self._setup.current_y,
                'Z': self._setup.current_z,
                'A': self._setup.current_a,
                'B': self._setup.current_b,
                'C': self._setup.current_c,
            }
            self._workOffset.setToolTableRelative(
                self._setup.coord_system, p_val, current_pos, axes_dict)

    def _get_coord_offset(self, axis: int) -> float:
        offsets = self._setup.coord_offsets.get(self._setup.coord_system, [0]*6)
        if axis < len(offsets):
            return offsets[axis]
        return 0.0

    def _get_total_offset(self, axis: str) -> float:
        coord_off = self._get_coord_offset('XYZ'.find(axis) if axis in 'XYZ' else 0)
        g92_off = {'X': self._setup.g92_offset_x, 'Y': self._setup.g92_offset_y,
                    'Z': self._setup.g92_offset_z}.get(axis, 0)
        tool_off = self._setup.tool_offset_z if axis == 'Z' else 0
        return coord_off + g92_off + tool_off

    # ==================== M代码转换 ====================

    def _convert_m(self, block: Block):
        for mcode in block.m_modes:
            if mcode == 0:
                self._canon.PROGRAM_STOP()
            elif mcode == 1:
                if self._setup.optional_stop:
                    self._canon.PROGRAM_STOP()
            elif mcode == 2 or mcode == 30:
                self._canon.PROGRAM_END()
                self._setup.program_running = False
            elif mcode == 3:
                self._setup.spindle_turning = SpindleState.CW
                self._canon.START_SPINDLE_CLOCKWISE(0, 0)
            elif mcode == 4:
                self._setup.spindle_turning = SpindleState.CCW
                self._canon.START_SPINDLE_COUNTERCLOCKWISE(0, 0)
            elif mcode == 5:
                self._setup.spindle_turning = SpindleState.OFF
                self._canon.STOP_SPINDLE_TURNING(0)
            elif mcode == 6:
                # M6 换刀流程
                tool_no = block.t if block.t is not None else self._setup.current_tool
                # 从刀具表获取新刀数据
                tool_data = self._toolTable.changeTool(tool_no)
                self._setup.current_tool = tool_no
                # 应用刀具长度偏移
                if tool_data and tool_data.z_offset != 0.0:
                    self._setup.tool_offset_z = tool_data.z_offset
                self._canon.CHANGE_TOOL(0, tool_no)
            elif mcode == 7:
                self._setup.mist = True
                self._canon.MIST_ON()
            elif mcode == 8:
                self._setup.flood = True
                self._canon.FLOOD_ON()
            elif mcode == 9:
                self._setup.mist = False
                self._setup.flood = False
                self._canon.FLOOD_OFF()
                self._canon.MIST_OFF()
            elif mcode == 48:
                self._setup.feed_override = True
                self._setup.spindle_override = True
                self._canon.ENABLE_FEED_OVERRIDE()
                self._canon.ENABLE_SPEED_OVERRIDE()
            elif mcode == 49:
                self._setup.feed_override = False
                self._setup.spindle_override = False
                self._canon.DISABLE_FEED_OVERRIDE()
                self._canon.DISABLE_SPEED_OVERRIDE()
            elif mcode == 60:
                self._canon.MESSAGE("M60: 托盘交换")
            elif mcode == 98:
                pass
            elif mcode == 99:
                pass

    # ==================== 运动转换 ====================

    def _convert_motion(self, block: Block):
        motion = block.motion_to_be
        if motion == 0:
            self._convert_straight(block, rapid=True)
        elif motion == 1:
            self._convert_straight(block, rapid=False)
        elif motion == 2 or motion == 3:
            self._convert_arc(block, cw=(motion == 2))
        elif motion in (73, 81, 82, 83, 84, 85, 86, 87, 88, 89):
            self._convert_canned_cycle(block, motion)
        elif motion == 4:
            self._convert_dwell(block)
        elif motion == 80:
            self._setup.cycle_active = False
        elif motion == 38:
            self._convert_probe(block)

    def _get_target_xyz(self, block: Block) -> Tuple[float, float, float]:
        if self._setup.distance_mode == DistanceMode.INCREMENTAL:
            x = self._setup.current_x + (block.x if block.x is not None else 0)
            y = self._setup.current_y + (block.y if block.y is not None else 0)
            z = self._setup.current_z + (block.z if block.z is not None else 0)
        else:
            x = block.x if block.x is not None else self._setup.current_x
            y = block.y if block.y is not None else self._setup.current_y
            z = block.z if block.z is not None else self._setup.current_z
        return (x, y, z)

    def _convert_straight(self, block: Block, rapid: bool):
        x, y, z = self._get_target_xyz(block)
        if block.f is not None and block.f > 0:
            self._setup.feed_rate = block.f
            self._canon.SET_FEED_RATE(block.f)

        if rapid:
            self._canon.STRAIGHT_TRAVERSE(
                EmcPose(PmCartesian(x, y, z)))
        else:
            if self._setup.feed_rate <= 0:
                self._error_msg = "进给率未设置"
                return
            self._canon.STRAIGHT_FEED(
                EmcPose(PmCartesian(x, y, z)))

        self._setup.current_x = x
        self._setup.current_y = y
        self._setup.current_z = z
        self._setup.program_x = x
        self._setup.program_y = y
        self._setup.program_z = z

    def _convert_arc(self, block: Block, cw: bool):
        x, y, z = self._get_target_xyz(block)

        if block.r is not None:
            cx, cy = self._arc_from_radius(block, x, y, cw)
        elif block.i is not None or block.j is not None:
            i_val = block.i if block.i is not None else 0.0
            j_val = block.j if block.j is not None else 0.0
            if self._setup.ijk_distance_mode == DistanceMode.INCREMENTAL:
                cx = self._setup.current_x + i_val
                cy = self._setup.current_y + j_val
            else:
                cx = i_val
                cy = j_val
        else:
            self._error_msg = "圆弧缺少I/J或R参数"
            return

        if block.f is not None and block.f > 0:
            self._setup.feed_rate = block.f
            self._canon.SET_FEED_RATE(block.f)

        if self._setup.feed_rate <= 0:
            self._error_msg = "进给率未设置"
            return

        radius = math.sqrt(
            (self._setup.current_x - cx)**2 +
            (self._setup.current_y - cy)**2
        )

        if radius < 0.0001:
            self._error_msg = "圆弧半径为零"
            return

        start_angle = math.atan2(
            self._setup.current_y - cy,
            self._setup.current_x - cx
        )
        end_angle = math.atan2(y - cy, x - cx)

        if cw:
            if end_angle > start_angle:
                end_angle -= 2 * math.pi
        else:
            if end_angle < start_angle:
                end_angle += 2 * math.pi

        actual_end_x = cx + radius * math.cos(end_angle)
        actual_end_y = cy + radius * math.sin(end_angle)

        self._canon.ARC_FEED(
            EmcPose(PmCartesian(actual_end_x, actual_end_y, z)),
            PmCartesian(cx, cy, 0),
            0.0,
            -1 if cw else 1,
            self._setup.feed_rate
        )

        self._setup.current_x = actual_end_x
        self._setup.current_y = actual_end_y
        self._setup.current_z = z
        self._setup.program_x = actual_end_x
        self._setup.program_y = actual_end_y
        self._setup.program_z = z

    def _arc_from_radius(self, block: Block, end_x: float, end_y: float, cw: bool) -> Tuple[float, float]:
        r = block.r
        sx, sy = self._setup.current_x, self._setup.current_y
        dx = end_x - sx
        dy = end_y - sy
        d = math.sqrt(dx*dx + dy*dy)
        if d < 0.0001:
            return ((sx + end_x)/2, (sy + end_y)/2)
        h = math.sqrt(max(0, r*r - (d/2)*(d/2)))
        mx = (sx + end_x) / 2
        my = (sy + end_y) / 2
        nx = -dy / d
        ny = dx / d
        if r > d / 2:
            if cw:
                cx = mx - h * nx
                cy = my - h * ny
            else:
                cx = mx + h * nx
                cy = my + h * ny
        else:
            if cw:
                cx = mx + h * nx
                cy = my + h * ny
            else:
                cx = mx - h * nx
                cy = my - h * ny
        return (cx, cy)

    def _convert_dwell(self, block: Block):
        seconds = 0.0
        if block.p is not None:
            seconds = block.p / 1000.0 if block.p < 1000 else block.p
        elif block.x is not None:
            seconds = block.x
        self._canon.DWELL(seconds)

    def _convert_probe(self, block: Block):
        x, y, z = self._get_target_xyz(block)
        self._canon.PROBE_OPEN(0)
        self._canon.STRAIGHT_FEED(
            EmcPose(PmCartesian(x, y, z)))
        self._canon.PROBE_CLOSE(0)

    def _convert_canned_cycle(self, block: Block, motion: int):
        x, y, z_target = self._get_target_xyz(block)
        r_clear = block.r if block.r is not None else self._setup.current_z
        q_depth = block.q if block.q is not None else 0.0
        l_count = block.l if block.l is not None and block.l > 0 else 1
        p_dwell = block.p if block.p is not None else 0.0

        if self._setup.retract_mode == 98:
            retract_z = self._setup.current_z
        else:
            retract_z = r_clear

        self._setup.cycle_active = True

        for cycle_count in range(l_count):
            self._canon.STRAIGHT_TRAVERSE(
                EmcPose(PmCartesian(x, y, retract_z)))
            self._canon.STRAIGHT_TRAVERSE(
                EmcPose(PmCartesian(x, y, r_clear)))

            if motion == 73:
                depth = r_clear
                while depth > z_target:
                    next_depth = max(depth - q_depth, z_target)
                    self._canon.STRAIGHT_FEED(
                        EmcPose(PmCartesian(x, y, next_depth)))
                    self._canon.STRAIGHT_TRAVERSE(
                        EmcPose(PmCartesian(x, y, r_clear)))
                    depth = next_depth
            elif motion == 81:
                self._canon.STRAIGHT_FEED(
                    EmcPose(PmCartesian(x, y, z_target)))
                self._canon.STRAIGHT_TRAVERSE(
                    EmcPose(PmCartesian(x, y, r_clear)))
            elif motion == 82:
                self._canon.STRAIGHT_FEED(
                    EmcPose(PmCartesian(x, y, z_target)))
                if p_dwell > 0:
                    self._canon.DWELL(p_dwell / 1000.0)
                self._canon.STRAIGHT_TRAVERSE(
                    EmcPose(PmCartesian(x, y, r_clear)))
            elif motion == 83:
                depth = r_clear
                while depth > z_target:
                    next_depth = max(depth - q_depth, z_target)
                    self._canon.STRAIGHT_FEED(
                        EmcPose(PmCartesian(x, y, next_depth)))
                    self._canon.STRAIGHT_TRAVERSE(
                        EmcPose(PmCartesian(x, y, r_clear)))
                    depth = next_depth
            elif motion == 84:
                self._canon.STRAIGHT_FEED(
                    EmcPose(PmCartesian(x, y, z_target)))
                self._canon.STRAIGHT_FEED(
                    EmcPose(PmCartesian(x, y, r_clear)))
            elif motion == 85:
                self._canon.STRAIGHT_FEED(
                    EmcPose(PmCartesian(x, y, z_target)))
                self._canon.STRAIGHT_FEED(
                    EmcPose(PmCartesian(x, y, r_clear)))
            elif motion == 86:
                self._canon.STRAIGHT_FEED(
                    EmcPose(PmCartesian(x, y, z_target)))
                self._canon.STOP_SPINDLE_TURNING(0)
                self._canon.STRAIGHT_TRAVERSE(
                    EmcPose(PmCartesian(x, y, r_clear)))
            else:
                self._canon.STRAIGHT_FEED(
                    EmcPose(PmCartesian(x, y, z_target)))
                self._canon.STRAIGHT_TRAVERSE(
                    EmcPose(PmCartesian(x, y, r_clear)))

        self._canon.STRAIGHT_TRAVERSE(
            EmcPose(PmCartesian(x, y, retract_z)))

        self._setup.current_x = x
        self._setup.current_y = y
        self._setup.current_z = retract_z

    # ==================== O-word控制流 ====================

    def _execute_oword(self, block: Block) -> Tuple[bool, str]:
        ot = block.o_type
        on = block.o_num

        if ot == OWordType.SUB:
            self._setup.in_sub_definition = True
            self._setup.current_sub_name = on
            self._setup.sub_lines = []
            return (True, "")

        if ot == OWordType.ENDSUB:
            self._setup.in_sub_definition = False
            self._setup.subroutines[self._setup.current_sub_name] = self._setup.sub_lines[:]
            self._setup.current_sub_name = None
            self._setup.sub_lines = []
            return (True, "")

        if ot == OWordType.CALL:
            return self._oword_call(block)

        if ot == OWordType.IF:
            val = block.p if block.p is not None else 0.0
            cond = (val != 0.0)
            self._setup.if_stack.append(cond)
            self._setup.skipping = not cond
            return (True, "")

        if ot == OWordType.ELSEIF:
            if self._setup.if_stack:
                prev = self._setup.if_stack[-1]
                if prev:
                    self._setup.skipping = True
                else:
                    val = block.p if block.p is not None else 0.0
                    cond = (val != 0.0)
                    self._setup.if_stack[-1] = cond
                    self._setup.skipping = not cond
            return (True, "")

        if ot == OWordType.ELSE:
            if self._setup.if_stack:
                prev = self._setup.if_stack[-1]
                self._setup.if_stack[-1] = not prev
                self._setup.skipping = prev
            return (True, "")

        if ot == OWordType.ENDIF:
            if self._setup.if_stack:
                self._setup.if_stack.pop()
            self._setup.skipping = False
            return (True, "")

        if ot == OWordType.DO:
            return (True, "")

        if ot == OWordType.WHILE_CONDITION:
            val = block.p if block.p is not None else 0.0
            cond = (val != 0.0)
            if not cond:
                self._setup.skipping = True
                self._setup.skipping_to = 'endwhile'
            return (True, "")

        if ot == OWordType.ENDWHILE:
            if self._setup.skipping and self._setup.skipping_to == 'endwhile':
                self._setup.skipping = False
                self._setup.skipping_to = None
            return (True, "")

        if ot == OWordType.REPEAT:
            count = int(block.p) if block.p is not None else 0
            self._setup.repeat_stack.append((count, 0, self._current_line))
            return (True, "")

        if ot == OWordType.ENDREPEAT:
            if self._setup.repeat_stack:
                count, iteration, start_line = self._setup.repeat_stack[-1]
                iteration += 1
                if iteration < count:
                    self._setup.repeat_stack[-1] = (count, iteration, start_line)
                    self._current_line = start_line
                else:
                    self._setup.repeat_stack.pop()
            return (True, "")

        if ot == OWordType.BREAK:
            self._setup.break_flag = True
            return (True, "")

        if ot == OWordType.CONTINUE:
            self._setup.continue_flag = True
            return (True, "")

        if ot == OWordType.RETURN:
            if self._setup.sub_call_stack:
                ctx = self._setup.sub_call_stack.pop()
                self._setup.local_params.pop() if self._setup.local_params else None
                self._setup.feed_rate = ctx.saved_params.get(-1, self._setup.feed_rate)
            return (True, "")

        if ot == OWordType.GOTO:
            if on in self._setup.subroutines:
                pass
            return (True, "")

        if self._setup.in_sub_definition and self._setup.current_sub_name is not None:
            self._setup.sub_lines.append(block.line_text)
            return (True, "")

        if self._setup.skipping:
            return (True, "")

        return (True, "")

    def _oword_call(self, block: Block) -> Tuple[bool, str]:
        on = block.o_num
        if on not in self._setup.subroutines:
            self._error_msg = f"子程序 O{on} 未定义"
            return (False, self._error_msg)

        ctx = SubContext(
            line_number=self._current_line,
            sub_name=on,
            saved_g_codes=list(self._setup.active_g_codes),
            saved_m_codes=list(self._setup.active_m_codes),
        )

        local_params = {}
        self._setup.local_params.append(local_params)

        self._setup.sub_call_stack.append(ctx)

        sub_lines = self._setup.subroutines[on]
        for sub_line in sub_lines:
            sub_line = sub_line.strip()
            if not sub_line or sub_line.startswith('('):
                continue
            ok, err = self.execute(sub_line)
            if not ok:
                return (False, err)

        if self._setup.sub_call_stack:
            self._setup.sub_call_stack.pop()
        if self._setup.local_params:
            self._setup.local_params.pop()

        return (True, "")

    # ==================== 状态更新 ====================

    def _write_active_codes(self, block: Block):
        for group, gcode in block.g_modes.items():
            if 1 <= group <= 16:
                idx = group - 1
                if idx < len(self._setup.active_g_codes):
                    self._setup.active_g_codes[idx] = gcode

        for mcode in block.m_modes:
            if mcode in (3, 4, 5):
                self._setup.active_m_codes[4] = mcode
            elif mcode in (6,):
                self._setup.active_m_codes[6] = mcode
            elif mcode in (7, 8, 9):
                self._setup.active_m_codes[7] = mcode

        self._setup.active_settings[0] = self._setup.feed_rate
        self._setup.active_settings[1] = self._setup.spindle_speed
        self._setup.active_settings[2] = self._setup.current_tool

    def get_position(self) -> Tuple[float, float, float]:
        return (self._setup.current_x, self._setup.current_y, self._setup.current_z)

    def get_status(self) -> Dict[str, Any]:
        return {
            'position': self.get_position(),
            'motion_mode': self._setup.motion_mode,
            'distance_mode': self._setup.distance_mode,
            'coord_system': self._setup.coord_system,
            'feed_rate': self._setup.feed_rate,
            'spindle_speed': self._setup.spindle_speed,
            'spindle_state': self._setup.spindle_turning,
            'tool': self._setup.current_tool,
            'mist': self._setup.mist,
            'flood': self._setup.flood,
        }
