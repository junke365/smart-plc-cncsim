"""机床控制面板 - 提供JOG、主轴控制、模式切换等"""
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QGroupBox,
    QPushButton, QLabel, QDoubleSpinBox, QComboBox, QSlider,
    QButtonGroup, QRadioButton, QSizePolicy, QFrame, QScrollArea
)
from PySide6.QtCore import Qt, Signal, QTimer
from PySide6.QtGui import QFont


class ControlPanel(QWidget):
    """机床控制面板"""

    jog_axis_changed = Signal(str)
    jog_velocity_changed = Signal(float)
    jog_continuous = Signal(str, float)
    jog_increment = Signal(str, float)
    jog_stop = Signal(str)

    cycle_start = Signal()
    cycle_stop = Signal()
    cycle_resume = Signal()

    estop_on = Signal()
    estop_off = Signal()
    power_on = Signal()
    power_off = Signal()

    spindle_cw = Signal()
    spindle_ccw = Signal()
    spindle_stop = Signal()
    spindle_speed_changed = Signal(float)

    home_joint = Signal(int)
    home_all = Signal()

    feed_override_changed = Signal(float)
    rapid_override_changed = Signal(float)
    spindle_override_changed = Signal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(280)
        self._jogButtons = []
        self._homeButtons = []

        # 持续JOG支持
        self._jogAxis: str = ""
        self._jogVelocity: float = 0.0
        self._jogTimer = QTimer(self)
        self._jogTimer.setInterval(20)  # 50Hz JOG刷新
        self._jogTimer.timeout.connect(self._onJogTick)

        self._setup_ui()
        self.setAxes(["X", "Y", "Z"])

    def _setup_ui(self):
        # 外层滚动区域，防止轴多时控件被挤
        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setFrameShape(QFrame.NoFrame)
        outer.addWidget(scroll)

        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(5)

        # ========== 电源和模式 ==========
        power_group = QGroupBox("电源 / 模式")
        power_layout = QGridLayout()

        self._btn_estop = QPushButton("急停")
        self._btn_estop.setMinimumHeight(40)
        self._btn_estop.setStyleSheet(
            "QPushButton { background-color: #cc3333; color: white; font-weight: bold; font-size: 14px; }"
            "QPushButton:hover { background-color: #ff4444; }"
        )
        self._btn_estop.clicked.connect(self.estop_on.emit)
        power_layout.addWidget(self._btn_estop, 0, 0, 1, 2)

        self._btn_estop_release = QPushButton("解除急停")
        self._btn_estop_release.setMinimumHeight(30)
        self._btn_estop_release.clicked.connect(self.estop_off.emit)
        power_layout.addWidget(self._btn_estop_release, 1, 0)

        self._btn_power_on = QPushButton("上电")
        self._btn_power_on.setMinimumHeight(30)
        self._btn_power_on.setStyleSheet(
            "QPushButton { background-color: #33aa33; color: white; font-weight: bold; }"
        )
        self._btn_power_on.clicked.connect(self.power_on.emit)
        power_layout.addWidget(self._btn_power_on, 1, 1)

        power_group.setLayout(power_layout)
        layout.addWidget(power_group)

        # ========== 运行控制 ==========
        run_group = QGroupBox("运行控制")
        run_layout = QGridLayout()

        self._btn_cycle_start = QPushButton("循环启动 (F5)")
        self._btn_cycle_start.setMinimumHeight(35)
        self._btn_cycle_start.setStyleSheet(
            "QPushButton { background-color: #2266aa; color: white; font-weight: bold; }"
        )
        self._btn_cycle_start.clicked.connect(self.cycle_start.emit)
        run_layout.addWidget(self._btn_cycle_start, 0, 0)

        self._btn_cycle_stop = QPushButton("循环停止 (F6)")
        self._btn_cycle_stop.setMinimumHeight(35)
        self._btn_cycle_stop.clicked.connect(self.cycle_stop.emit)
        run_layout.addWidget(self._btn_cycle_stop, 0, 1)

        self._btn_cycle_resume = QPushButton("恢复运行")
        self._btn_cycle_resume.setMinimumHeight(30)
        self._btn_cycle_resume.clicked.connect(self.cycle_resume.emit)
        run_layout.addWidget(self._btn_cycle_resume, 1, 0, 1, 2)

        run_group.setLayout(run_layout)
        layout.addWidget(run_group)

        # ========== JOG 控制 ==========
        jog_group = QGroupBox("手动JOG")
        jog_layout = QGridLayout()

        jog_layout.addWidget(QLabel("轴:"), 0, 0)
        self._jog_axis = QComboBox()
        self._jog_axis.currentTextChanged.connect(self.jog_axis_changed.emit)
        jog_layout.addWidget(self._jog_axis, 0, 1)

        jog_layout.addWidget(QLabel("速度:"), 1, 0)
        self._jog_velocity = QDoubleSpinBox()
        self._jog_velocity.setRange(1, 5000)
        self._jog_velocity.setValue(500)
        self._jog_velocity.setSuffix(" mm/min")
        self._jog_velocity.valueChanged.connect(self.jog_velocity_changed.emit)
        jog_layout.addWidget(self._jog_velocity, 1, 1)

        jog_layout.addWidget(QLabel("步长:"), 2, 0)
        self._jog_increment = QDoubleSpinBox()
        self._jog_increment.setRange(0.001, 100)
        self._jog_increment.setValue(1.0)
        self._jog_increment.setSuffix(" mm")
        jog_layout.addWidget(self._jog_increment, 2, 1)

        # 动态JOG按钮区域 - 由 setAxes() 重建
        self._jogButtonContainer = QWidget()
        self._jogButtonLayout = QGridLayout(self._jogButtonContainer)
        self._jogButtonLayout.setContentsMargins(0, 0, 0, 0)
        self._jogButtonLayout.setSpacing(4)
        jog_layout.addWidget(self._jogButtonContainer, 3, 0, 1, 4)

        # 全轴回零
        self._btn_home_all = QPushButton("全轴回零")
        self._btn_home_all.clicked.connect(self.home_all.emit)
        jog_layout.addWidget(self._btn_home_all, 4, 0, 1, 4)

        jog_group.setLayout(jog_layout)
        layout.addWidget(jog_group)

        # ========== 主轴控制 ==========
        spindle_group = QGroupBox("主轴控制")
        spindle_layout = QGridLayout()

        spindle_layout.addWidget(QLabel("转速:"), 0, 0)
        self._spindle_speed = QDoubleSpinBox()
        self._spindle_speed.setRange(0, 20000)
        self._spindle_speed.setValue(1000)
        self._spindle_speed.setSuffix(" RPM")
        self._spindle_speed.valueChanged.connect(self.spindle_speed_changed.emit)
        spindle_layout.addWidget(self._spindle_speed, 0, 1)

        self._btn_spindle_cw = QPushButton("正转 M3")
        self._btn_spindle_cw.setStyleSheet("QPushButton { color: green; font-weight: bold; }")
        self._btn_spindle_cw.clicked.connect(self.spindle_cw.emit)
        spindle_layout.addWidget(self._btn_spindle_cw, 1, 0)

        self._btn_spindle_ccw = QPushButton("反转 M4")
        self._btn_spindle_ccw.setStyleSheet("QPushButton { color: blue; font-weight: bold; }")
        self._btn_spindle_ccw.clicked.connect(self.spindle_ccw.emit)
        spindle_layout.addWidget(self._btn_spindle_ccw, 1, 1)

        self._btn_spindle_stop = QPushButton("停止 M5")
        self._btn_spindle_stop.clicked.connect(self.spindle_stop.emit)
        spindle_layout.addWidget(self._btn_spindle_stop, 2, 0, 1, 2)

        spindle_group.setLayout(spindle_layout)
        layout.addWidget(spindle_group)

        # ========== 倍率控制 ==========
        override_group = QGroupBox("倍率")
        override_layout = QGridLayout()

        override_layout.addWidget(QLabel("进给:"), 0, 0)
        self._feed_override_slider = QSlider(Qt.Horizontal)
        self._feed_override_slider.setRange(0, 200)
        self._feed_override_slider.setValue(100)
        self._feed_override_slider.setTickPosition(QSlider.TicksBelow)
        self._feed_override_slider.setTickInterval(10)
        self._feed_override_slider.valueChanged.connect(
            lambda v: self.feed_override_changed.emit(v / 100.0))
        override_layout.addWidget(self._feed_override_slider, 0, 1)
        self._feed_override_label = QLabel("100%")
        self._feed_override_slider.valueChanged.connect(
            lambda v: self._feed_override_label.setText(f"{v}%"))
        override_layout.addWidget(self._feed_override_label, 0, 2)

        override_layout.addWidget(QLabel("快移:"), 1, 0)
        self._rapid_override_slider = QSlider(Qt.Horizontal)
        self._rapid_override_slider.setRange(0, 200)
        self._rapid_override_slider.setValue(100)
        self._rapid_override_slider.setTickPosition(QSlider.TicksBelow)
        self._rapid_override_slider.setTickInterval(10)
        self._rapid_override_slider.valueChanged.connect(
            lambda v: self.rapid_override_changed.emit(v / 100.0))
        override_layout.addWidget(self._rapid_override_slider, 1, 1)
        self._rapid_override_label = QLabel("100%")
        self._rapid_override_slider.valueChanged.connect(
            lambda v: self._rapid_override_label.setText(f"{v}%"))
        override_layout.addWidget(self._rapid_override_label, 1, 2)

        override_layout.addWidget(QLabel("主轴:"), 2, 0)
        self._spindle_override_slider = QSlider(Qt.Horizontal)
        self._spindle_override_slider.setRange(0, 200)
        self._spindle_override_slider.setValue(100)
        self._spindle_override_slider.setTickPosition(QSlider.TicksBelow)
        self._spindle_override_slider.setTickInterval(10)
        self._spindle_override_slider.valueChanged.connect(
            lambda v: self.spindle_override_changed.emit(v / 100.0))
        override_layout.addWidget(self._spindle_override_slider, 2, 1)
        self._spindle_override_label = QLabel("100%")
        self._spindle_override_slider.valueChanged.connect(
            lambda v: self._spindle_override_label.setText(f"{v}%"))
        override_layout.addWidget(self._spindle_override_label, 2, 2)

        override_group.setLayout(override_layout)
        layout.addWidget(override_group)

        layout.addStretch()

        # 设置滚动区域内容
        scroll.setWidget(content)

    def setAxes(self, axes):
        """动态重建JOG按钮，根据轴列表

        参数:
            axes: 轴名称列表，如 ["X","Y","Z"] 或 ["J1","J2","J3","J4","J5","J6"]
        """
        # 更新下拉框
        self._jog_axis.blockSignals(True)
        self._jog_axis.clear()
        self._jog_axis.addItems(axes)
        self._jog_axis.blockSignals(False)

        # 清除旧按钮
        for btn in self._jogButtons:
            btn.setParent(None)
            btn.deleteLater()
        for btn in self._homeButtons:
            btn.setParent(None)
            btn.deleteLater()
        self._jogButtons.clear()
        self._homeButtons.clear()

        # 清空布局中所有残留项
        while self._jogButtonLayout.count():
            item = self._jogButtonLayout.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)
                w.deleteLater()

        btn_size = 50

        # 为每个轴创建 +/- 按钮，每轴一行
        for i, axis_name in enumerate(axes):
            row = i

            btn_minus = QPushButton(f"{axis_name}-")
            btn_minus.setFixedSize(btn_size, 36)
            btn_minus.pressed.connect(lambda a=axis_name: self._startJog(a, -self._jog_velocity.value()))
            btn_minus.released.connect(lambda a=axis_name: self._stopJog())
            self._jogButtonLayout.addWidget(btn_minus, row, 0)
            self._jogButtons.append(btn_minus)

            btn_home = QPushButton(f"{axis_name}零")
            btn_home.setFixedSize(55, 36)
            idx = i
            btn_home.clicked.connect(lambda j=idx: self.home_joint.emit(j))
            self._jogButtonLayout.addWidget(btn_home, row, 1)
            self._homeButtons.append(btn_home)

            btn_plus = QPushButton(f"{axis_name}+")
            btn_plus.setFixedSize(btn_size, 36)
            btn_plus.pressed.connect(lambda a=axis_name: self._startJog(a, self._jog_velocity.value()))
            btn_plus.released.connect(lambda a=axis_name: self._stopJog())
            self._jogButtonLayout.addWidget(btn_plus, row, 2)
            self._jogButtons.append(btn_plus)

        print(f"[面板] JOG轴已更新: {axes}")

    def _startJog(self, axis: str, velocity: float):
        """开始持续JOG - 按下按钮时调用"""
        self._jogAxis = axis
        self._jogVelocity = velocity
        # 立即执行一次
        self.jog_continuous.emit(axis, velocity)
        # 启动定时器持续执行
        if not self._jogTimer.isActive():
            self._jogTimer.start()

    def _stopJog(self):
        """停止JOG - 松开按钮时调用"""
        self._jogTimer.stop()
        if self._jogAxis:
            self.jog_stop.emit(self._jogAxis)
            self._jogAxis = ""

    def _onJogTick(self):
        """JOG定时器回调 - 持续移动"""
        if self._jogAxis and self._jogVelocity != 0:
            self.jog_continuous.emit(self._jogAxis, self._jogVelocity)
