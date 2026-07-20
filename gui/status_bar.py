"""状态显示面板 - 显示机床位置、主轴状态、G代码模态等"""
from PySide6.QtWidgets import (
    QWidget, QGridLayout, QLabel, QGroupBox, QVBoxLayout,
    QFrame, QHBoxLayout
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QFont, QColor
from typing import Dict, Any, Optional


class DROLabel(QLabel):
    """数字读出器(DRO)标签 - 显示坐标值"""

    def __init__(self, title: str = "", parent=None):
        super().__init__(parent)
        self._title = title
        self.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        font = QFont("Consolas", 14, QFont.Bold)
        self.setFont(font)
        self.setMinimumWidth(120)
        self.setStyleSheet(
            "QLabel { background-color: #1a1a2e; color: #00ff00; "
            "padding: 4px; border: 1px solid #333; }"
        )
        self.setText("0.000")

    def set_value(self, value: float, decimals: int = 3):
        self.setText(f"{value:>10.{decimals}f}")


class StatusPanel(QWidget):
    """状态显示面板"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._setup_ui()

    def _setup_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setSpacing(3)

        # ========== 机器坐标显示 (动态) ==========
        self._pos_group = QGroupBox("机器坐标 (MCS)")
        self._pos_layout = QGridLayout()
        self._pos_group.setLayout(self._pos_layout)
        main_layout.addWidget(self._pos_group)

        # ========== 工件坐标显示 (动态) ==========
        self._wpos_group = QGroupBox("工件坐标 (WCS)")
        self._wpos_layout = QGridLayout()
        self._wpos_group.setLayout(self._wpos_layout)
        main_layout.addWidget(self._wpos_group)

        self._dro_labels: Dict[str, DROLabel] = {}

        # ========== 状态信息 ==========
        info_group = QGroupBox("状态")
        info_layout = QGridLayout()

        info_layout.addWidget(QLabel("模式:"), 0, 0)
        self._mode_label = QLabel("ESTOP")
        self._mode_label.setStyleSheet("color: red; font-weight: bold;")
        info_layout.addWidget(self._mode_label, 0, 1)

        info_layout.addWidget(QLabel("运动:"), 0, 2)
        self._motion_label = QLabel("停止")
        info_layout.addWidget(self._motion_label, 0, 3)

        info_layout.addWidget(QLabel("主轴:"), 1, 0)
        self._spindle_label = QLabel("停止")
        info_layout.addWidget(self._spindle_label, 1, 1)

        info_layout.addWidget(QLabel("转速:"), 1, 2)
        self._rpm_label = QLabel("0 RPM")
        info_layout.addWidget(self._rpm_label, 1, 3)

        info_layout.addWidget(QLabel("进给:"), 2, 0)
        self._feed_label = QLabel("0.0 mm/min")
        info_layout.addWidget(self._feed_label, 2, 1)

        info_layout.addWidget(QLabel("刀具:"), 2, 2)
        self._tool_label = QLabel("T0")
        info_layout.addWidget(self._tool_label, 2, 3)

        info_layout.addWidget(QLabel("坐标系:"), 3, 0)
        self._wcs_label = QLabel("G54")
        info_layout.addWidget(self._wcs_label, 3, 1)

        info_layout.addWidget(QLabel("单位:"), 3, 2)
        self._units_label = QLabel("mm")
        info_layout.addWidget(self._units_label, 3, 3)

        info_group.setLayout(info_layout)
        main_layout.addWidget(info_group)

        # ========== G/M代码模态显示 ==========
        modal_group = QGroupBox("模态代码")
        modal_layout = QGridLayout()
        modal_layout.setSpacing(2)
        modal_layout.setContentsMargins(4, 8, 4, 4)

        self._gcode_labels = []
        for i in range(6):
            lbl = QLabel("---")
            lbl.setStyleSheet(
                "QLabel { background-color: #1a1a2e; color: #ffcc00; "
                "padding: 2px 4px; border: 1px solid #333; font-family: Consolas; font-size: 11px; }"
            )
            modal_layout.addWidget(lbl, i // 3, i % 3)
            self._gcode_labels.append(lbl)

        self._mcode_labels = []
        for i in range(3):
            lbl = QLabel("---")
            lbl.setStyleSheet(
                "QLabel { background-color: #1a1a2e; color: #ff66cc; "
                "padding: 2px 4px; border: 1px solid #333; font-family: Consolas; font-size: 11px; }"
            )
            modal_layout.addWidget(lbl, 2, i)
            self._mcode_labels.append(lbl)

        modal_group.setLayout(modal_layout)
        main_layout.addWidget(modal_group)

        # ========== 消息/错误 ==========
        msg_group = QGroupBox("消息")
        msg_layout = QVBoxLayout()
        self._message_label = QLabel("")
        self._message_label.setStyleSheet(
            "QLabel { background-color: #1a1a2e; color: #ff8800; "
            "padding: 4px; border: 1px solid #333; min-height: 20px; }"
        )
        self._message_label.setWordWrap(True)
        msg_layout.addWidget(self._message_label)
        msg_group.setLayout(msg_layout)
        main_layout.addWidget(msg_group)

        main_layout.addStretch()

    # ==================== 公共接口 ====================

    def setAxes(self, axes):
        """动态重建DRO显示，根据轴名称列表

        参数:
            axes: 轴名称列表，如 ["X","Y","Z"] 或 ["J1","J2","J3","J4","J5","J6"]
        """
        self._dro_labels.clear()

        # 清除旧布局
        self._clearLayout(self._pos_layout)
        self._clearLayout(self._wpos_layout)

        cols = min(len(axes), 3)
        for i, name in enumerate(axes):
            row = i // cols
            col = i % cols

            # 机器坐标
            lbl = QLabel(f"{name}:")
            lbl.setStyleSheet("QLabel { color: #888; font-weight: bold; font-size: 12px; }")
            self._pos_layout.addWidget(lbl, row * 2, col)
            dro = DROLabel(name)
            self._dro_labels[name] = dro
            self._pos_layout.addWidget(dro, row * 2 + 1, col)

            # 工件坐标
            wlbl = QLabel(f"{name}:")
            wlbl.setStyleSheet("QLabel { color: #888; font-weight: bold; font-size: 12px; }")
            self._wpos_layout.addWidget(wlbl, row * 2, col)
            wdro = DROLabel(name)
            wdro.setStyleSheet(
                "QLabel { background-color: #1a1a2e; color: #00aaff; "
                "padding: 4px; border: 1px solid #333; }"
            )
            self._dro_labels[f'w_{name}'] = wdro
            self._wpos_layout.addWidget(wdro, row * 2 + 1, col)

        print(f"[面板] DRO轴已更新: {axes}")

    def _clearLayout(self, layout):
        """清空布局中所有控件"""
        while layout.count():
            item = layout.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)
                w.deleteLater()

    def update_position(self, pos_dict: dict):
        """更新机器坐标（字典格式 {轴名: 位置}）"""
        for axis, value in pos_dict.items():
            if axis in self._dro_labels:
                self._dro_labels[axis].set_value(value)

    def update_work_position(self, pos_dict: dict):
        """更新工件坐标（字典格式 {轴名: 位置}）"""
        for axis, value in pos_dict.items():
            key = f'w_{axis}'
            if key in self._dro_labels:
                self._dro_labels[key].set_value(value)

    def update_mode(self, mode: str):
        self._mode_label.setText(mode)
        if mode in ("ESTOP",):
            self._mode_label.setStyleSheet("color: red; font-weight: bold;")
        elif mode in ("ON", "运行"):
            self._mode_label.setStyleSheet("color: #00cc00; font-weight: bold;")
        else:
            self._mode_label.setStyleSheet("color: #cccc00; font-weight: bold;")

    def update_motion_state(self, state: str):
        self._motion_label.setText(state)

    def update_spindle(self, state: str, speed: float = 0):
        self._spindle_label.setText(state)
        self._rpm_label.setText(f"{speed:.0f} RPM")

    def update_feed(self, rate: float):
        self._feed_label.setText(f"{rate:.1f} mm/min")

    def update_tool(self, tool_num: int):
        self._tool_label.setText(f"T{tool_num}")

    def update_wcs(self, wcs: str):
        self._wcs_label.setText(wcs)

    def update_units(self, units: str):
        self._units_label.setText(units)

    def update_gcodes(self, gcodes: list):
        for i, lbl in enumerate(self._gcode_labels):
            if i < len(gcodes) and gcodes[i] != 0:
                lbl.setText(f"G{gcodes[i]}")
            else:
                lbl.setText("---")

    def update_mcodes(self, mcodes: list):
        for i, lbl in enumerate(self._mcode_labels):
            if i < len(mcodes) and mcodes[i] != 0:
                lbl.setText(f"M{mcodes[i]}")
            else:
                lbl.setText("---")

    def show_message(self, msg: str):
        self._message_label.setText(msg)

    def clear_message(self):
        self._message_label.setText("")
