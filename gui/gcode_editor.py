"""G代码编辑器 - 带语法高亮和行号"""
from PySide6.QtWidgets import (
  QPlainTextEdit, QWidget, QTextEdit, QVBoxLayout, QLabel
)
from PySide6.QtCore import Qt, QRect, QSize, Signal
from PySide6.QtGui import (
  QColor, QTextFormat, QPainter, QFont, QFontMetrics,
  QSyntaxHighlighter, QTextCharFormat
)
from typing import Optional
import re


class GCodeHighlighter(QSyntaxHighlighter):
  """G代码语法高亮器"""

  def __init__(self, parent=None):
    super().__init__(parent)
    self._rules = []

    gFormat = QTextCharFormat()
    gFormat.setForeground(QColor(0, 100, 200))
    gFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(r'\bG\d+\.?\d*\b'), gFormat))

    mFormat = QTextCharFormat()
    mFormat.setForeground(QColor(150, 0, 150))
    mFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(r'\bM\d+\b'), mFormat))

    coordFormat = QTextCharFormat()
    coordFormat.setForeground(QColor(0, 130, 0))
    self._rules.append((re.compile(r'[XYZIJKRFSPTQHDEABCUVW]-?\d*\.?\d*'), coordFormat))

    oFormat = QTextCharFormat()
    oFormat.setForeground(QColor(200, 100, 0))
    oFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(r'\bO\d+\b'), oFormat))

    commentFormat = QTextCharFormat()
    commentFormat.setForeground(QColor(128, 128, 128))
    commentFormat.setFontItalic(True)
    self._rules.append((re.compile(r'\(.*?\)'), commentFormat))
    self._rules.append((re.compile(r';.*'), commentFormat))

    numFormat = QTextCharFormat()
    numFormat.setForeground(QColor(180, 0, 0))
    self._rules.append((re.compile(r'\b\d+\.?\d*\b'), numFormat))

    pctFormat = QTextCharFormat()
    pctFormat.setForeground(QColor(180, 180, 0))
    pctFormat.setFontWeight(QFont.Bold)
    self._rules.append((re.compile(r'^%$', re.MULTILINE), pctFormat))

  def highlightBlock(self, text: str):
    """对当前文本块应用语法高亮"""
    for pattern, fmt in self._rules:
      for match in pattern.finditer(text):
        start = match.start()
        length = match.end() - start
        self.setFormat(start, length, fmt)


class LineNumberArea(QWidget):
  """行号区域"""

  def __init__(self, editor: 'GCodeEditor'):
    super().__init__(editor)
    self._editor = editor

  def sizeHint(self):
    return QSize(self._editor.lineNumberAreaWidth(), 0)

  def paintEvent(self, event):
    self._editor.lineNumberAreaPaintEvent(event)


class GCodeEditor(QPlainTextEdit):
  """带行号的G代码编辑器"""

  fileLoaded = Signal(str)
  lineChanged = Signal(int)

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

    self._highlighter = GCodeHighlighter(self.document())

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

  def setExecutingLine(self, line: int):
    self._executingLine = line
    self._lineNumberArea.update()

  def set_executing_line(self, line: int):
    """设置当前执行行 (snake_case 别名)"""
    self.setExecutingLine(line)

  def clearExecutingLine(self):
    self._executingLine = -1
    self._lineNumberArea.update()

  def clear_executing_line(self):
    """清除执行行标记 (snake_case 别名)"""
    self.clearExecutingLine()

  def setErrorLine(self, line: int):
    self._errorLine = line
    self._lineNumberArea.update()

  def set_error_line(self, line: int):
    """设置错误行 (snake_case 别名)"""
    self.setErrorLine(line)

  def clearErrorLine(self):
    self._errorLine = -1
    self._lineNumberArea.update()

  def clear_error_line(self):
    """清除错误行标记 (snake_case 别名)"""
    self.clearErrorLine()

  def getText(self) -> str:
    return self.toPlainText()

  def get_text(self) -> str:
    """获取全部文本 (snake_case 别名)"""
    return self.getText()

  def getLines(self) -> list:
    return self.toPlainText().split('\n')

  def setText(self, text: str):
    self.setPlainText(text)

  def loadFile(self, filename: str) -> bool:
    try:
      with open(filename, 'r', encoding='utf-8', errors='replace') as f:
        content = f.read()
      self.setPlainText(content)
      self.fileLoaded.emit(filename)
      return True
    except Exception:
      return False

  def load_file(self, filename: str) -> bool:
    """加载G代码文件 (snake_case 别名)"""
    return self.loadFile(filename)

  def getLineCount(self) -> int:
    return self.blockCount()
