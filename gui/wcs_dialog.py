"""工件坐标系管理对话框 - 管理 G54-G59.3 偏移"""
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QDoubleSpinBox, QTableWidget, QTableWidgetItem,
    QHeaderView, QComboBox, QMessageBox, QWidget
)
from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont

from ..core.work_offset import WorkOffsetManager, CoordSystemId, CoordOffset


class WCSDialog(QDialog):
    """工件坐标系管理对话框"""

    wcs_changed = Signal(int)  # 坐标系切换信号

    def __init__(self, work_offset: WorkOffsetManager, parent=None):
        super().__init__(parent)
        self._workOffset = work_offset
        self.setWindowTitle("工件坐标系管理 (G54-G59.3)")
        self.setMinimumSize(600, 450)
        self._setup_ui()
        self._refreshTable()

    def _setup_ui(self):
        layout = QVBoxLayout(self)

        # ========== 当前坐标系 ==========
        current_group = QGroupBox("当前工件坐标系")
        current_layout = QHBoxLayout()

        current_layout.addWidget(QLabel("激活坐标系:"))
        self._current_cs_label = QLabel("G54")
        self._current_cs_label.setStyleSheet(
            "font-size: 16px; font-weight: bold; color: #00ff00;")
        current_layout.addWidget(self._current_cs_label)

        current_layout.addWidget(QLabel("切换到:"))
        self._cs_combo = QComboBox()
        for cs in CoordSystemId:
            self._cs_combo.addItem(f"G{cs.value}", cs.value)
        self._cs_combo.currentIndexChanged.connect(self._onCSChanged)
        current_layout.addWidget(self._cs_combo)

        current_layout.addStretch()
        current_group.setLayout(current_layout)
        layout.addWidget(current_group)

        # ========== 偏移量表格 ==========
        self._table = QTableWidget()
        self._table.setColumnCount(10)
        self._table.setHorizontalHeaderLabels(
            ["坐标系", "X", "Y", "Z", "A", "B", "C", "U", "V", "W"])
        self._table.horizontalHeader().setSectionResizeMode(
            QHeaderView.Stretch)
        self._table.setSelectionBehavior(QTableWidget.SelectRows)
        self._table.setSelectionMode(QTableWidget.SingleSelection)
        self._table.currentCellChanged.connect(self._onSelectionChanged)
        layout.addWidget(self._table)

        # ========== 偏移量编辑 ==========
        edit_group = QGroupBox("编辑偏移量")
        edit_layout = QGridLayout()

        row = 0
        edit_layout.addWidget(QLabel("坐标系:"), row, 0)
        self._edit_cs = QComboBox()
        for cs in CoordSystemId:
            self._edit_cs.addItem(f"G{cs.value}", cs.value)
        edit_layout.addWidget(self._edit_cs, row, 1)

        axes = [("X", 0, 2), ("Y", 0, 3), ("Z", 0, 4),
                ("A", 1, 0), ("B", 1, 1), ("C", 1, 2),
                ("U", 1, 3), ("V", 1, 4), ("W", 2, 0)]
        self._edit_axes = {}
        for name, r, c in axes:
            edit_layout.addWidget(QLabel(f"{name}:"), r, c * 2)
            spin = QDoubleSpinBox()
            spin.setRange(-10000, 10000)
            spin.setDecimals(3)
            spin.setSuffix(" mm" if name in "XYZUVW" else " deg")
            self._edit_axes[name] = spin
            edit_layout.addWidget(spin, r, c * 2 + 1)

        edit_group.setLayout(edit_layout)
        layout.addWidget(edit_group)

        # ========== 按钮栏 ==========
        btn_layout = QHBoxLayout()

        btn_apply = QPushButton("应用偏移量")
        btn_apply.setStyleSheet(
            "QPushButton { color: green; font-weight: bold; }")
        btn_apply.clicked.connect(self._onApplyOffset)
        btn_layout.addWidget(btn_apply)

        btn_zero = QPushButton("当前坐标系清零")
        btn_zero.clicked.connect(self._onZeroOffset)
        btn_layout.addWidget(btn_zero)

        btn_all_zero = QPushButton("所有坐标系清零")
        btn_all_zero.clicked.connect(self._onAllZero)
        btn_layout.addWidget(btn_all_zero)

        btn_layout.addStretch()

        btn_refresh = QPushButton("刷新")
        btn_refresh.clicked.connect(self._refreshTable)
        btn_layout.addWidget(btn_refresh)

        btn_close = QPushButton("关闭")
        btn_close.clicked.connect(self.close)
        btn_layout.addWidget(btn_close)

        layout.addLayout(btn_layout)

    def _refreshTable(self):
        """刷新坐标系偏移表格"""
        cs_list = self._workOffset.listCoordSystems()
        self._table.setRowCount(len(cs_list))

        for row, (cs_id, offset) in enumerate(cs_list):
            name_item = QTableWidgetItem(f"G{cs_id}")
            name_item.setFont(QFont("Consolas", 10, QFont.Bold))
            self._table.setItem(row, 0, name_item)

            vals = [offset.x, offset.y, offset.z,
                    offset.a, offset.b, offset.c,
                    offset.u, offset.v, offset.w]
            for col, val in enumerate(vals):
                item = QTableWidgetItem(f"{val:.3f}")
                if val != 0.0:
                    item.setForeground(Qt.yellow)
                self._table.setItem(row, col + 1, item)

        # 高亮当前坐标系
        cur = self._workOffset.currentCoord
        self._current_cs_label.setText(f"G{cur}")
        idx = self._cs_combo.findData(cur)
        if idx >= 0:
            self._cs_combo.blockSignals(True)
            self._cs_combo.setCurrentIndex(idx)
            self._cs_combo.blockSignals(False)

        print(f"[WCS对话框] 刷新显示 {len(cs_list)} 个坐标系")

    def _onSelectionChanged(self, row, col, prev_row, prev_col):
        """表格选中行变更"""
        if row < 0:
            return
        cs_list = self._workOffset.listCoordSystems()
        if row < len(cs_list):
            cs_id, offset = cs_list[row]
            # 设置编辑区的坐标系选择
            idx = self._edit_cs.findData(cs_id)
            if idx >= 0:
                self._edit_cs.setCurrentIndex(idx)
            # 填充偏移值
            for name in ['X', 'Y', 'Z', 'A', 'B', 'C', 'U', 'V', 'W']:
                self._edit_axes[name].setValue(offset.getAxis(name))

    def _onCSChanged(self, index):
        """切换坐标系"""
        cs_id = self._cs_combo.currentData()
        if cs_id is not None:
            self._workOffset.currentCoord = cs_id
            self._current_cs_label.setText(f"G{cs_id}")
            self.wcs_changed.emit(cs_id)
            print(f"[WCS对话框] 切换到 G{cs_id}")

    def _onApplyOffset(self):
        """应用编辑的偏移量"""
        cs_id = self._edit_cs.currentData()
        if cs_id is None:
            return
        axes_dict = {}
        for name, spin in self._edit_axes.items():
            axes_dict[name.lower()] = spin.value()
        offset = CoordOffset(**axes_dict)
        self._workOffset.setOffset(cs_id, offset)
        self._refreshTable()

    def _onZeroOffset(self):
        """当前坐标系偏移清零"""
        cs_id = self._edit_cs.currentData()
        if cs_id is None:
            return
        self._workOffset.setOffset(cs_id, CoordOffset())
        self._refreshTable()

    def _onAllZero(self):
        """所有坐标系偏移清零"""
        reply = QMessageBox.question(
            self, "确认", "确定要清除所有坐标系偏移吗？",
            QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            for cs in CoordSystemId:
                self._workOffset.setOffset(cs.value, CoordOffset())
            self._refreshTable()
