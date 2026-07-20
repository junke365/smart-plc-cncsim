"""刀具管理对话框 - 管理刀具表、刀具参数、换刀"""
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QLineEdit, QDoubleSpinBox, QSpinBox,
    QTableWidget, QTableWidgetItem, QHeaderView, QMessageBox,
    QTabWidget, QWidget, QComboBox, QTextEdit
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont

from ..core.tool_table import ToolTable, ToolData, createDefaultToolTable


class ToolDialog(QDialog):
    """刀具管理对话框"""

    tool_changed = Signal(int)  # 换刀信号，传递新刀号

    def __init__(self, tool_table: ToolTable, parent=None):
        super().__init__(parent)
        self._toolTable = tool_table
        self.setWindowTitle("刀具管理")
        self.setMinimumSize(650, 500)
        self._setup_ui()
        self._refreshTable()

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        # ========== 当前刀具信息 ==========
        current_group = QGroupBox("当前刀具")
        current_layout = QHBoxLayout()

        current_layout.addWidget(QLabel("主轴刀具:"))
        self._current_tool_label = QLabel("T0")
        self._current_tool_label.setStyleSheet(
            "font-size: 16px; font-weight: bold; color: #00ff00;")
        current_layout.addWidget(self._current_tool_label)

        current_layout.addWidget(QLabel("待换刀具:"))
        self._selected_tool_label = QLabel("无")
        self._selected_tool_label.setStyleSheet(
            "font-size: 16px; color: #ffcc00;")
        current_layout.addWidget(self._selected_tool_label)

        current_layout.addStretch()
        current_group.setLayout(current_layout)
        layout.addWidget(current_group)

        # ========== 刀具表 ==========
        self._table = QTableWidget()
        self._table.setColumnCount(8)
        self._table.setHorizontalHeaderLabels(
            ["刀号", "刀位", "直径", "X偏移", "Y偏移", "Z偏移", "补偿号", "注释"])
        self._table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        self._table.setSelectionBehavior(QTableWidget.SelectRows)
        self._table.setSelectionMode(QTableWidget.SingleSelection)
        self._table.currentCellChanged.connect(self._onSelectionChanged)
        layout.addWidget(self._table)

        # ========== 刀具参数编辑 ==========
        edit_group = QGroupBox("刀具参数编辑")
        edit_layout = QGridLayout()

        row = 0
        edit_layout.addWidget(QLabel("刀号:"), row, 0)
        self._edit_tool_no = QSpinBox()
        self._edit_tool_no.setRange(1, 999)
        edit_layout.addWidget(self._edit_tool_no, row, 1)

        edit_layout.addWidget(QLabel("刀位:"), row, 2)
        self._edit_pocket = QSpinBox()
        self._edit_pocket.setRange(0, 99)
        edit_layout.addWidget(self._edit_pocket, row, 3)

        row += 1
        edit_layout.addWidget(QLabel("直径:"), row, 0)
        self._edit_diameter = QDoubleSpinBox()
        self._edit_diameter.setRange(0, 1000)
        self._edit_diameter.setSuffix(" mm")
        edit_layout.addWidget(self._edit_diameter, row, 1)

        edit_layout.addWidget(QLabel("Z偏移:"), row, 2)
        self._edit_z_offset = QDoubleSpinBox()
        self._edit_z_offset.setRange(-1000, 1000)
        self._edit_z_offset.setSuffix(" mm")
        edit_layout.addWidget(self._edit_z_offset, row, 3)

        row += 1
        edit_layout.addWidget(QLabel("X偏移:"), row, 0)
        self._edit_x_offset = QDoubleSpinBox()
        self._edit_x_offset.setRange(-1000, 1000)
        self._edit_x_offset.setSuffix(" mm")
        edit_layout.addWidget(self._edit_x_offset, row, 1)

        edit_layout.addWidget(QLabel("Y偏移:"), row, 2)
        self._edit_y_offset = QDoubleSpinBox()
        self._edit_y_offset.setRange(-1000, 1000)
        self._edit_y_offset.setSuffix(" mm")
        edit_layout.addWidget(self._edit_y_offset, row, 3)

        row += 1
        edit_layout.addWidget(QLabel("注释:"), row, 0)
        self._edit_comment = QLineEdit()
        edit_layout.addWidget(self._edit_comment, row, 1, 1, 3)

        edit_group.setLayout(edit_layout)
        layout.addWidget(edit_group)

        # ========== 按钮栏 ==========
        btn_layout = QHBoxLayout()

        btn_add = QPushButton("添加/更新刀具")
        btn_add.setStyleSheet("QPushButton { color: green; font-weight: bold; }")
        btn_add.clicked.connect(self._onAddTool)
        btn_layout.addWidget(btn_add)

        btn_delete = QPushButton("删除刀具")
        btn_delete.setStyleSheet("QPushButton { color: red; }")
        btn_delete.clicked.connect(self._onDeleteTool)
        btn_layout.addWidget(btn_delete)

        btn_layout.addSpacing(20)

        btn_select = QPushButton("选刀(T)")
        btn_select.clicked.connect(self._onSelectTool)
        btn_layout.addWidget(btn_select)

        btn_change = QPushButton("换刀(M6)")
        btn_change.setStyleSheet("QPushButton { color: #00aaff; font-weight: bold; }")
        btn_change.clicked.connect(self._onChangeTool)
        btn_layout.addWidget(btn_change)

        btn_layout.addStretch()

        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self._refreshTable)
        btn_layout.addWidget(btn_refresh)

        btn_close = QPushButton("关闭")
        btn_close.clicked.connect(self.close)
        btn_layout.addWidget(btn_close)

        layout.addLayout(btn_layout)

    def _refreshTable(self):
        """刷新刀具表显示"""
        tools = self._toolTable.listTools()
        self._table.setRowCount(len(tools))

        for row, td in enumerate(tools):
            self._table.setItem(row, 0, QTableWidgetItem(str(td.tool_no)))
            self._table.setItem(row, 1, QTableWidgetItem(str(td.pocket)))
            self._table.setItem(row, 2, QTableWidgetItem(f"{td.diameter:.1f}"))
            self._table.setItem(row, 3, QTableWidgetItem(f"{td.x_offset:.3f}"))
            self._table.setItem(row, 4, QTableWidgetItem(f"{td.y_offset:.3f}"))
            self._table.setItem(row, 5, QTableWidgetItem(f"{td.z_offset:.3f}"))
            self._table.setItem(row, 6, QTableWidgetItem(str(td.pocket)))
            self._table.setItem(row, 7, QTableWidgetItem(td.comment))

        self._current_tool_label.setText(f"T{self._toolTable.currentTool}")
        sel = self._toolTable.selectedTool
        self._selected_tool_label.setText(f"T{sel}" if sel > 0 else "无")
        print(f"[刀具对话框] 刷新显示 {len(tools)} 把刀具")

    def _onSelectionChanged(self, row, col, prev_row, prev_col):
        """表格选中行变更"""
        if row < 0:
            return
        tools = self._toolTable.listTools()
        if row < len(tools):
            td = tools[row]
            self._edit_tool_no.setValue(td.tool_no)
            self._edit_pocket.setValue(td.pocket)
            self._edit_diameter.setValue(td.diameter)
            self._edit_x_offset.setValue(td.x_offset)
            self._edit_y_offset.setValue(td.y_offset)
            self._edit_z_offset.setValue(td.z_offset)
            self._edit_comment.setText(td.comment)

    def _onAddTool(self):
        """添加/更新刀具"""
        td = ToolData(
            tool_no=self._edit_tool_no.value(),
            pocket=self._edit_pocket.value(),
            diameter=self._edit_diameter.value(),
            x_offset=self._edit_x_offset.value(),
            y_offset=self._edit_y_offset.value(),
            z_offset=self._edit_z_offset.value(),
            comment=self._edit_comment.text(),
        )
        self._toolTable.setTool(td)
        self._refreshTable()

    def _onDeleteTool(self):
        """删除选中刀具"""
        tool_no = self._edit_tool_no.value()
        reply = QMessageBox.question(
            self, "确认删除", f"确定要删除 T{tool_no} 吗？",
            QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self._toolTable.removeTool(tool_no)
            self._refreshTable()

    def _onSelectTool(self):
        """选刀"""
        tool_no = self._edit_tool_no.value()
        self._toolTable.selectTool(tool_no)
        self._refreshTable()
        print(f"[刀具对话框] 选刀 T{tool_no}")

    def _onChangeTool(self):
        """换刀"""
        tool_no = self._edit_tool_no.value()
        td = self._toolTable.changeTool(tool_no)
        self._refreshTable()
        self.tool_changed.emit(tool_no)
        comment = f" ({td.comment})" if td and td.comment else ""
        print(f"[刀具对话框] 换刀 T{tool_no}{comment}")
