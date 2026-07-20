"""RAPID机器人程序编辑器 - 带语法高亮和行号"""
from PySide6.QtWidgets import QPlainTextEdit, QWidget, QTextEdit
from PySide6.QtCore import Qt, QRect, QSize
from PySide6.QtGui import (
  QColor, QTextFormat, QPainter, QFont, QFontMetrics,
  QSyntaxHighlighter, QTextCharFormat
)
import re


class RapidHighlighter(QSyntaxHighlighter):
  """RAPID语法高亮器"""

  def __init__(self, parent=None):
    super().__init__(parent)
    self._rules = []

    # 运动指令
    moveFormat = QTextCharFormat()
    moveFormat.setForeground(QColor(0, 180, 0))
    moveFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(
      r'\b(MoveJ|MoveL|MoveC|MoveAbsJ|MoveExtJ)\b', re.IGNORECASE), moveFormat))

    # 流程控制
    flowFormat = QTextCharFormat()
    flowFormat.setForeground(QColor(180, 100, 0))
    flowFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(
      r'\b(IF|ELSEIF|ELSE|ENDIF|FOR|ENDFOR|WHILE|ENDWHILE|'
      r'TEST|CASE|DEFAULT|ENDTEST|GOTO|BREAK|EXIT|EXITCYCLE|'
      r'COMPACT|STOP)\b', re.IGNORECASE), flowFormat))

    # 程序结构
    procFormat = QTextCharFormat()
    procFormat.setForeground(QColor(100, 0, 200))
    procFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(
      r'\b(MODULE|ENDMODULE|PROC|ENDPROC|FUNC|ENDFUNC|TRAP|ENDTRAP|'
      r'CONN|DISC|ERROR|RECORD)\b', re.IGNORECASE), procFormat))

    # 数据类型
    typeFormat = QTextCharFormat()
    typeFormat.setForeground(QColor(0, 150, 180))
    self._rules.append((re.compile(
      r'\b(PERS|VAR|CONST|LOCAL)\s+'
      r'(num|string|robtarget|jointtarget|speeddata|zonedata|tooldata|loaddata)',
      re.IGNORECASE), typeFormat))

    # 赋值操作符
    assignFormat = QTextCharFormat()
    assignFormat.setForeground(QColor(200, 50, 50))
    assignFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(r':='), assignFormat))

    # 速度数据
    speedFormat = QTextCharFormat()
    speedFormat.setForeground(QColor(150, 180, 0))
    self._rules.append((re.compile(
      r'\b(v\d+|vmax)\b', re.IGNORECASE), speedFormat))

    # 区域数据
    zoneFormat = QTextCharFormat()
    zoneFormat.setForeground(QColor(0, 150, 150))
    self._rules.append((re.compile(
      r'\b(fine|z\d+)\b', re.IGNORECASE), zoneFormat))

    # 工具数据
    toolFormat = QTextCharFormat()
    toolFormat.setForeground(QColor(200, 100, 200))
    self._rules.append((re.compile(
      r'\b(tool\d+|load\d+)\b', re.IGNORECASE), toolFormat))

    # TP功能
    tpFormat = QTextCharFormat()
    tpFormat.setForeground(QColor(180, 180, 0))
    tpFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(
      r'\b(TPWRITE|TPREADFK|TPREADNUM|TPERASE)\b', re.IGNORECASE), tpFormat))

    # Socket通信
    sockFormat = QTextCharFormat()
    sockFormat.setForeground(QColor(150, 150, 255))
    self._rules.append((re.compile(
      r'\b(SOCKETCREATE|SOCKETCONNECT|SOCKETSEND|SOCKETRECEIVE|SOCKETCLOSE)\b',
      re.IGNORECASE), sockFormat))

    # 内置函数
    funcFormat = QTextCharFormat()
    funcFormat.setForeground(QColor(100, 200, 200))
    self._rules.append((re.compile(
      r'\b(STRTOVAL|VAL|ABS|SIN|COS|SQRT|ROUND)\b', re.IGNORECASE), funcFormat))

    # 注释 (! 开头到行尾)
    commentFormat = QTextCharFormat()
    commentFormat.setForeground(QColor(128, 128, 128))
    commentFormat.setFontItalic(True)
    self._rules.append((re.compile(r'!.*'), commentFormat))

    # 数字
    numFormat = QTextCharFormat()
    numFormat.setForeground(QColor(200, 100, 100))
    self._rules.append((re.compile(r'\b\d+\.?\d*\b'), numFormat))

    # 字符串 ("...")
    strFormat = QTextCharFormat()
    strFormat.setForeground(QColor(0, 180, 100))
    self._rules.append((re.compile(r'"[^"]*"'), strFormat))

    # 标签 (行首的 "标签名:")
    labelFormat = QTextCharFormat()
    labelFormat.setForeground(QColor(200, 200, 0))
    labelFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(r'^[A-Za-z_]\w*:', re.MULTILINE), labelFormat))

  def highlightBlock(self, text: str):
    """对当前文本块应用语法高亮"""
    for pattern, fmt in self._rules:
      for match in pattern.finditer(text):
        start = match.start()
        length = match.end() - start
        self.setFormat(start, length, fmt)


class LineNumberArea(QWidget):
  """行号区域"""

  def __init__(self, editor: 'RobotEditor'):
    super().__init__(editor)
    self._editor = editor

  def sizeHint(self):
    return QSize(self._editor.lineNumberAreaWidth(), 0)

  def paintEvent(self, event):
    self._editor.lineNumberAreaPaintEvent(event)


class RobotEditor(QPlainTextEdit):
  """带行号的RAPID机器人程序编辑器"""

  def __init__(self, parent=None):
    super().__init__(parent)

    font = QFont("Consolas", 11)
    font.setStyleHint(QFont.Monospace)
    self.setFont(font)
    self.setTabStopDistance(
      QFontMetrics(font).horizontalAdvance(' ') * 4
    )

    self._lineNumberArea = LineNumberArea(self)
    self.blockCountChanged.connect(self._updateLineNumberAreaWidth)
    self.updateRequest.connect(self._updateLineNumberArea)
    self.cursorPositionChanged.connect(self._highlightCurrentLine)
    self._updateLineNumberAreaWidth(0)
    self._highlightCurrentLine()

    self._highlighter = RapidHighlighter(self.document())

    self._executingLine: int = -1
    self._errorLine: int = -1

  def lineNumberAreaWidth(self):
    digits = max(1, len(str(self.blockCount())))
    space = 10 + self.fontMetrics().horizontalAdvance('9') * digits + 10
    return space

  def _updateLineNumberAreaWidth(self, newBlockCount):
    self.setViewportMargins(self.lineNumberAreaWidth(), 0, 0, 0)

  def _updateLineNumberArea(self, rect, dy):
    if dy:
      self._lineNumberArea.scroll(0, dy)
    else:
      self._lineNumberArea.update(
        0, rect.y(),
        self._lineNumberArea.width(), rect.height()
      )
    if rect.contains(self.viewport().rect()):
      self._updateLineNumberAreaWidth(0)

  def resizeEvent(self, event):
    super().resizeEvent(event)
    cr = self.contentsRect()
    self._lineNumberArea.setGeometry(
      QRect(cr.left(), cr.top(), self.lineNumberAreaWidth(), cr.height())
    )

  def _highlightCurrentLine(self):
    extraSelections = []
    if not self.isReadOnly():
      selection = QTextEdit.ExtraSelection()
      selection.format.setBackground(QColor(40, 40, 60))
      selection.format.setProperty(QTextFormat.FullWidthSelection, True)
      selection.cursor = self.textCursor()
      selection.cursor.clearSelection()
      extraSelections.append(selection)
    self.setExtraSelections(extraSelections)

  def lineNumberAreaPaintEvent(self, event):
    painter = QPainter(self._lineNumberArea)
    painter.fillRect(event.rect(), QColor(30, 30, 45))

    block = self.firstVisibleBlock()
    blockNumber = block.blockNumber()
    top = round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top())
    bottom = top + round(self.blockBoundingRect(block).height())

    while block.isValid() and top <= event.rect().bottom():
      if block.isVisible() and bottom >= event.rect().top():
        number = str(blockNumber + 1)

        painter.setPen(QColor(120, 120, 140))
        if blockNumber == self._executingLine:
          painter.setPen(QColor(255, 200, 0))
          painter.fillRect(
            0, top,
            self._lineNumberArea.width(),
            self.fontMetrics().height(),
            QColor(60, 60, 40)
          )
        elif blockNumber == self._errorLine:
          painter.setPen(QColor(255, 80, 80))

        painter.drawText(
          0, top,
          self._lineNumberArea.width() - 5,
          self.fontMetrics().height(),
          Qt.AlignRight | Qt.AlignVCenter,
          number
        )

      block = block.next()
      top = bottom
      bottom = top + round(self.blockBoundingRect(block).height())
      blockNumber += 1

    painter.fillRect(
      event.rect().bottom() + 1, event.rect().top(),
      self._lineNumberArea.width(), event.rect().height(),
      QColor(30, 30, 45)
    )

  # ==================== 公共接口 ====================

  def set_executing_line(self, line: int):
    self._executingLine = line
    self._lineNumberArea.update()

  def clear_executing_line(self):
    self._executingLine = -1
    self._lineNumberArea.update()

  def set_error_line(self, line: int):
    self._errorLine = line
    self._lineNumberArea.update()

  def clear_error_line(self):
    self._errorLine = -1
    self._lineNumberArea.update()

  def get_text(self) -> str:
    return self.toPlainText()

  def set_text(self, text: str):
    self.setPlainText(text)

  def load_file(self, filename: str) -> bool:
    try:
      with open(filename, 'r', encoding='utf-8', errors='replace') as f:
        content = f.read()
      self.setPlainText(content)
      return True
    except Exception:
      return False
