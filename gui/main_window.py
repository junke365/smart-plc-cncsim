"""主窗口 - 整合所有GUI组件和仿真引擎"""
import os
import sys
from PySide6.QtWidgets import (
    QMainWindow, QSplitter, QMenuBar, QMenu,
    QFileDialog, QMessageBox, QToolBar, QStatusBar,
    QDockWidget, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QInputDialog,
    QApplication, QTextEdit, QStackedWidget, QPushButton
)
from PySide6.QtCore import Qt, QTimer, QSize
from PySide6.QtGui import QAction, QIcon, QKeySequence, QFont

from .opengl_view import OpenGLView
from .gcode_editor import GCodeEditor
from .robot_editor import RobotEditor
from .control_panel import ControlPanel
from .status_bar import StatusPanel
from .tool_dialog import ToolDialog
from .wcs_dialog import WCSDialog

from ..core.interpreter import Interpreter, MotionMode, SpindleState
from ..core.robot_interpreter import RobotInterpreter
from ..core.motion import MotionController, MotionState, MachineState, MotionConfig
from ..core.canon import CanonCommand
from ..core.coordinates import PmCartesian, EmcPose
from ..core.machines import getMachineNames, getMachineConfig, loadMachineModels


class MainWindow(QMainWindow):
    """LinuxCNC Windows 仿真器主窗口"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("智能PLC仿真器 v1.0")
        self.setMinimumSize(1200, 800)

        # 核心引擎 - G代码解释器
        self._interpreter = Interpreter()
        # 机器人解释器
        self._robotInterpreter = RobotInterpreter()
        # 当前活跃解释器 ("gcode" 或 "robot")
        self._activeInterpreter: str = "gcode"

        self._config = MotionConfig(
            num_joints=3,
            traj_cycle_time=0.001,
            max_velocity=5000.0,
            max_acceleration=50000.0,
            coordinates="XYZ"
        )
        self._motion = MotionController(self._config)

        # 文件状态
        self._current_file: str = ""
        self._file_lines: list = []
        self._executing: bool = False
        self._exec_line_idx: int = 0
        self._current_machine: str = ""

        # 设置UI
        self._setup_ui()
        self._setup_menu()
        self._setup_toolbar()
        self._setup_statusbar()
        self._setup_connections()
        self._setup_timer()

        # GL就绪后加载默认机型
        from PySide6.QtCore import QTimer
        QTimer.singleShot(200, lambda: self._on_switch_machine("VMC三轴铣床"))

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(2, 2, 2, 2)

        # 主分割器: 左(控制面板) + 中(编辑器+3D) + 右(状态)
        splitter = QSplitter(Qt.Horizontal)

        # 左侧 - 控制面板
        self._control_panel = ControlPanel()
        splitter.addWidget(self._control_panel)

        # 中间 - 编辑器和3D视图
        center_splitter = QSplitter(Qt.Vertical)

        # 编辑器容器 (支持G代码/机器人程序切换)
        editor_container = QWidget()
        editor_layout = QVBoxLayout(editor_container)
        editor_layout.setContentsMargins(0, 0, 0, 0)
        editor_layout.setSpacing(2)

        # 编辑器标签栏 (带切换按钮)
        editor_header = QHBoxLayout()
        editor_header.setContentsMargins(0, 0, 0, 0)
        editor_header.setSpacing(4)

        self._editor_label = QLabel("G代码编辑器")
        self._editor_label.setStyleSheet("QLabel { color: #888; padding: 2px; font-weight: bold; }")
        editor_header.addWidget(self._editor_label)

        self._btn_switch_interpreter = QPushButton("切换到机器人")
        self._btn_switch_interpreter.setFixedHeight(24)
        self._btn_switch_interpreter.setStyleSheet(
            "QPushButton { background-color: #45475a; color: #cdd6f4; border: 1px solid #585b70; "
            "border-radius: 3px; padding: 2px 8px; font-size: 11px; }"
            "QPushButton:hover { background-color: #585b70; }"
        )
        self._btn_switch_interpreter.clicked.connect(self._on_switch_interpreter)
        editor_header.addWidget(self._btn_switch_interpreter)

        editor_header.addStretch()
        editor_layout.addLayout(editor_header)

        # 编辑器堆栈 (G代码编辑器 / 机器人编辑器)
        self._editorStack = QStackedWidget()
        self._editor = GCodeEditor()
        self._robotEditor = RobotEditor()
        self._editorStack.addWidget(self._editor)      # index 0
        self._editorStack.addWidget(self._robotEditor)  # index 1
        editor_layout.addWidget(self._editorStack)

        center_splitter.addWidget(editor_container)

        # 3D视图
        view_container = QWidget()
        view_layout = QVBoxLayout(view_container)
        view_layout.setContentsMargins(0, 0, 0, 0)
        view_layout.setSpacing(2)

        view_header = QHBoxLayout()
        view_header.setContentsMargins(0, 0, 0, 0)
        view_header.setSpacing(4)

        view_label = QLabel("3D仿真视图")
        view_label.setStyleSheet("QLabel { color: #888; padding: 2px; }")
        view_header.addWidget(view_label)

        self._btn_toggle_grid = QPushButton("网格: 开")
        self._btn_toggle_grid.setFixedHeight(22)
        self._btn_toggle_grid.setFixedWidth(60)
        self._btn_toggle_grid.setStyleSheet(
            "QPushButton { background-color: #313244; color: #a6e3a1; border: 1px solid #585b70; "
            "border-radius: 3px; padding: 1px 4px; font-size: 11px; }"
            "QPushButton:hover { background-color: #45475a; }"
        )
        self._btn_toggle_grid.clicked.connect(self._on_toggle_grid)
        view_header.addWidget(self._btn_toggle_grid)

        view_header.addStretch()
        view_layout.addLayout(view_header)

        self._gl_view = OpenGLView()
        view_layout.addWidget(self._gl_view)

        center_splitter.addWidget(view_container)
        center_splitter.setSizes([400, 400])

        splitter.addWidget(center_splitter)

        # 右侧 - 状态面板
        self._status_panel = StatusPanel()
        splitter.addWidget(self._status_panel)

        splitter.setSizes([280, 700, 280])

        main_layout.addWidget(splitter)

        # 设置深色主题
        self.setStyleSheet(self._get_stylesheet())

    def _get_stylesheet(self) -> str:
        return """
        QMainWindow { background-color: #1e1e2e; }
        QWidget { background-color: #1e1e2e; color: #cdd6f4; }
        QGroupBox {
            border: 1px solid #45475a;
            border-radius: 4px;
            margin-top: 8px;
            padding-top: 8px;
            font-weight: bold;
            color: #89b4fa;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 5px;
        }
        QPushButton {
            background-color: #313244;
            border: 1px solid #45475a;
            border-radius: 3px;
            padding: 4px 8px;
            color: #cdd6f4;
        }
        QPushButton:hover { background-color: #45475a; }
        QPushButton:pressed { background-color: #585b70; }
        QLabel { color: #a6adc8; }
        QComboBox {
            background-color: #313244;
            border: 1px solid #45475a;
            border-radius: 3px;
            padding: 2px 8px;
        }
        QDoubleSpinBox, QSpinBox {
            background-color: #313244;
            border: 1px solid #45475a;
            border-radius: 3px;
            padding: 2px 4px;
        }
        QSlider::groove:horizontal {
            height: 4px;
            background: #45475a;
            border-radius: 2px;
        }
        QSlider::handle:horizontal {
            background: #89b4fa;
            width: 12px;
            margin: -4px 0;
            border-radius: 6px;
        }
        QSlider::sub-page:horizontal {
            background: #89b4fa;
            border-radius: 2px;
        }
        QPlainTextEdit {
            background-color: #181825;
            color: #cdd6f4;
            border: 1px solid #313244;
            selection-background-color: #45475a;
        }
        QMenuBar {
            background-color: #181825;
            color: #cdd6f4;
        }
        QMenuBar::item:selected { background-color: #45475a; }
        QMenu {
            background-color: #1e1e2e;
            border: 1px solid #45475a;
        }
        QMenu::item:selected { background-color: #45475a; }
        QToolBar { background-color: #181825; border: none; spacing: 2px; }
        QToolBar QToolButton {
            background-color: #313244;
            border: 1px solid #45475a;
            border-radius: 3px;
            padding: 4px 8px;
            color: #cdd6f4;
        }
        QToolBar QToolButton:hover { background-color: #45475a; }
        QStatusBar { background-color: #181825; color: #a6adc8; }
        """

    def _setup_menu(self):
        menubar = self.menuBar()

        file_menu = menubar.addMenu("文件(&F)")

        open_action = QAction("打开(&O)", self)
        open_action.setShortcut(QKeySequence.Open)
        open_action.triggered.connect(self._on_open)
        file_menu.addAction(open_action)

        save_action = QAction("保存(&S)", self)
        save_action.setShortcut(QKeySequence.Save)
        save_action.triggered.connect(self._on_save)
        file_menu.addAction(save_action)

        file_menu.addSeparator()

        exit_action = QAction("退出(&X)", self)
        exit_action.setShortcut(QKeySequence.Quit)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)

        # 仿真菜单
        sim_menu = menubar.addMenu("仿真(&S)")

        run_action = QAction("运行程序(F5)", self)
        run_action.setShortcut("F5")
        run_action.triggered.connect(self._on_cycle_start)
        sim_menu.addAction(run_action)

        stop_action = QAction("停止(F6)", self)
        stop_action.setShortcut("F6")
        stop_action.triggered.connect(self._on_cycle_stop)
        sim_menu.addAction(stop_action)

        reset_action = QAction("复位", self)
        reset_action.setShortcut("F9")
        reset_action.triggered.connect(self._on_reset)
        sim_menu.addAction(reset_action)

        clear_action = QAction("清除路径", self)
        clear_action.triggered.connect(self._on_clear_paths)
        sim_menu.addAction(clear_action)

        # MDI菜单
        mdi_menu = menubar.addMenu("MDI(&M)")

        mdi_action = QAction("MDI命令(&C)", self)
        mdi_action.setShortcut("F1")
        mdi_action.triggered.connect(self._on_mdi_command)
        mdi_menu.addAction(mdi_action)

        # 刀具菜单
        tool_menu = menubar.addMenu("刀具(&T)")

        tool_mng_action = QAction("刀具管理...", self)
        tool_mng_action.setShortcut("F3")
        tool_mng_action.triggered.connect(self._on_tool_management)
        tool_menu.addAction(tool_mng_action)

        # 工件坐标菜单
        wcs_menu = menubar.addMenu("工件坐标(&W)")

        wcs_mng_action = QAction("工件坐标系管理...", self)
        wcs_mng_action.setShortcut("F4")
        wcs_mng_action.triggered.connect(self._on_wcs_management)
        wcs_menu.addAction(wcs_mng_action)

        # 视图菜单
        view_menu = menubar.addMenu("视图(&V)")

        reset_view_action = QAction("重置视图", self)
        reset_view_action.setShortcut("F2")
        reset_view_action.triggered.connect(self._gl_view.reset_view)
        view_menu.addAction(reset_view_action)

        # 模型菜单
        model_menu = menubar.addMenu("机型(&L)")
        self._machine_actions = {}

        for machineName in getMachineNames():
            cfg = getMachineConfig(machineName)
            desc = cfg["desc"] if cfg else ""
            action = QAction(f"{machineName} ({desc})", self)
            action.triggered.connect(lambda checked=False, n=machineName: self._on_switch_machine(n))
            model_menu.addAction(action)
            self._machine_actions[machineName] = action

        model_menu.addSeparator()

        load_stl_action = QAction("加载自定义STL...", self)
        load_stl_action.triggered.connect(self._on_load_stl)
        model_menu.addAction(load_stl_action)

        load_obj_action = QAction("加载自定义OBJ...", self)
        load_obj_action.triggered.connect(self._on_load_obj)
        model_menu.addAction(load_obj_action)

        model_menu.addSeparator()

        clear_models_action = QAction("清除所有模型", self)
        clear_models_action.triggered.connect(self._on_clear_models)
        model_menu.addAction(clear_models_action)

        # 解释器菜单
        interp_menu = menubar.addMenu("解释器(&I)")

        gcode_action = QAction("G代码解释器 (RS274)", self)
        gcode_action.setShortcut("Ctrl+1")
        gcode_action.triggered.connect(lambda: self._switch_to_interpreter("gcode"))
        interp_menu.addAction(gcode_action)

        robot_action = QAction("机器人解释器 (RAPID)", self)
        robot_action.setShortcut("Ctrl+2")
        robot_action.triggered.connect(lambda: self._switch_to_interpreter("robot"))
        interp_menu.addAction(robot_action)

        interp_menu.addSeparator()

        switch_action = QAction("切换解释器", self)
        switch_action.setShortcut("Ctrl+Tab")
        switch_action.triggered.connect(self._on_switch_interpreter)
        interp_menu.addAction(switch_action)

        # 帮助菜单
        help_menu = menubar.addMenu("帮助(&H)")
        about_action = QAction("关于(&A)", self)
        about_action.triggered.connect(self._on_about)
        help_menu.addAction(about_action)

    def _setup_toolbar(self):
        toolbar = QToolBar("工具栏")
        toolbar.setIconSize(QSize(24, 24))
        self.addToolBar(toolbar)

        toolbar.addAction("打开", self._on_open)
        toolbar.addAction("保存", self._on_save)
        toolbar.addSeparator()
        toolbar.addAction("▶ 运行", self._on_cycle_start)
        toolbar.addAction("⏹ 停止", self._on_cycle_stop)
        toolbar.addAction("⏸ 暂停", self._on_cycle_stop)
        toolbar.addSeparator()
        toolbar.addAction("复位", self._on_reset)
        toolbar.addAction("清除路径", self._on_clear_paths)
        toolbar.addAction("重置视图", self._gl_view.reset_view)

    def _setup_statusbar(self):
        self._statusbar = QStatusBar()
        self.setStatusBar(self._statusbar)
        self._statusbar.showMessage("就绪 - 智能PLC仿真器")

    def _setup_connections(self):
        # 控制面板信号
        self._control_panel.estop_on.connect(self._on_estop)
        self._control_panel.estop_off.connect(self._on_estop_release)
        self._control_panel.power_on.connect(self._on_power_on)
        self._control_panel.cycle_start.connect(self._on_cycle_start)
        self._control_panel.cycle_stop.connect(self._on_cycle_stop)
        self._control_panel.cycle_resume.connect(self._on_cycle_resume)
        self._control_panel.jog_continuous.connect(self._on_jog_continuous)
        self._control_panel.jog_stop.connect(self._on_jog_stop)
        self._control_panel.home_all.connect(self._on_home_all)
        self._control_panel.home_joint.connect(self._on_home_joint)
        self._control_panel.spindle_cw.connect(self._on_spindle_cw)
        self._control_panel.spindle_ccw.connect(self._on_spindle_ccw)
        self._control_panel.spindle_stop.connect(self._on_spindle_stop)
        self._control_panel.feed_override_changed.connect(
            lambda v: self._motion.set_feed_override(v))
        self._control_panel.rapid_override_changed.connect(
            lambda v: self._motion.set_rapid_override(v))

        # 运动控制器回调
        self._motion.set_callbacks(
            on_position=self._on_position_update,
            on_state=self._on_state_change,
            on_message=self._on_message
        )

    def _setup_timer(self):
        self._sim_timer = QTimer()
        self._sim_timer.timeout.connect(self._on_sim_tick)
        self._sim_timer.start(10)

        self._servo_timer = QTimer()
        self._servo_timer.timeout.connect(self._on_servo_tick)
        self._servo_timer.start(1)

    # ==================== 定时器回调 ====================

    def _on_sim_tick(self):
        """10ms UI更新"""
        if self._executing:
            self._update_status_display()

    def _on_servo_tick(self):
        """1ms 伺服周期"""
        if self._executing:
            self._motion.run_cycle()

    # ==================== 解释器切换 ====================

    def _on_switch_interpreter(self):
        """切换G代码/机器人解释器"""
        if self._activeInterpreter == "gcode":
            self._switch_to_interpreter("robot")
        else:
            self._switch_to_interpreter("gcode")

    def _switch_to_interpreter(self, target: str):
        """切换到指定解释器"""
        if target == self._activeInterpreter:
            return
        if target == "robot":
            self._activeInterpreter = "robot"
            self._editorStack.setCurrentIndex(1)
            self._editor_label.setText("机器人程序编辑器 (RAPID)")
            self._btn_switch_interpreter.setText("切换到G代码")
            self._statusbar.showMessage("已切换到机器人解释器 (RAPID)")
            print("[UI] 切换到机器人解释器")
        else:
            self._activeInterpreter = "gcode"
            self._editorStack.setCurrentIndex(0)
            self._editor_label.setText("G代码编辑器 (RS274)")
            self._btn_switch_interpreter.setText("切换到机器人")
            self._statusbar.showMessage("已切换到G代码解释器")
            print("[UI] 切换到G代码解释器")

    def _get_active_editor(self):
        """获取当前活跃的编辑器"""
        if self._activeInterpreter == "robot":
            return self._robotEditor
        return self._editor

    # ==================== 文件操作 ====================

    def _on_open(self):
        if self._activeInterpreter == "robot":
            filename, _ = QFileDialog.getOpenFileName(
                self, "打开RAPID程序文件", "",
                "RAPID文件 (*.rap *.mod *.src);;所有文件 (*)"
            )
        else:
            filename, _ = QFileDialog.getOpenFileName(
                self, "打开G代码文件", "",
                "G代码文件 (*.ngc *.gcode *.nc *.gc);;所有文件 (*)"
            )
        if filename:
            self._load_file(filename)

    def _load_file(self, filename: str):
        editor = self._get_active_editor()
        if editor.load_file(filename):
            self._current_file = filename
            ext = os.path.splitext(filename)[1].lower()
            if ext in ('.rap', '.mod', '.src'):
                # 自动切换到机器人解释器
                if self._activeInterpreter != "robot":
                    self._on_switch_interpreter()
            elif ext in ('.ngc', '.gcode', '.nc', '.gc'):
                # 自动切换到G代码解释器
                if self._activeInterpreter != "gcode":
                    self._on_switch_interpreter()
            self.setWindowTitle(f"智能PLC仿真器 - {os.path.basename(filename)}")
            self._statusbar.showMessage(f"已加载: {filename}")

    def _on_save(self):
        if self._current_file:
            try:
                editor = self._get_active_editor()
                with open(self._current_file, 'w', encoding='utf-8') as f:
                    f.write(editor.get_text())
                self._statusbar.showMessage(f"已保存: {self._current_file}")
            except Exception as e:
                QMessageBox.warning(self, "保存失败", str(e))
        else:
            self._on_save_as()

    def _on_save_as(self):
        if self._activeInterpreter == "robot":
            filename, _ = QFileDialog.getSaveFileName(
                self, "保存RAPID程序", "",
                "RAPID文件 (*.rap);;所有文件 (*)"
            )
        else:
            filename, _ = QFileDialog.getSaveFileName(
                self, "保存G代码文件", "",
                "G代码文件 (*.ngc);;所有文件 (*)"
            )
        if filename:
            self._current_file = filename
            self._on_save()

    # ==================== 运行控制 ====================

    def _on_estop(self):
        self._motion.estop_on()
        print("[UI] 急停按下")
        self._statusbar.showMessage("急停!")
        self._status_panel.update_mode("ESTOP")

    def _on_estop_release(self):
        self._motion.estop_off()
        print("[UI] 急停已解除")
        self._statusbar.showMessage("急停已解除")
        self._status_panel.update_mode("OFF")

    def _on_power_on(self):
        self._motion.power_on()
        print("[UI] 已上电")
        self._statusbar.showMessage("已上电")
        self._status_panel.update_mode("ON")

    def _on_cycle_start(self):
        if self._motion.machine_state != MachineState.ON:
            self._on_power_on()

        editor = self._get_active_editor()
        text = editor.get_text()
        if not text.strip():
            print("[UI] 没有程序可执行")
            self._statusbar.showMessage("没有程序可执行")
            return

        if self._activeInterpreter == "robot":
            self._run_robot_program(text)
        else:
            self._run_gcode_program(text)

    def _run_gcode_program(self, text: str):
        """执行G代码程序"""
        self._interpreter.init()
        self._interpreter.reset()

        self._file_lines = text.split('\n')
        self._exec_line_idx = 0
        self._executing = True

        print(f"[UI] 开始执行G代码, 共{len(self._file_lines)}行")
        self._execute_next_line()

        self._motion.processCanonCommands(self._interpreter.canon)
        self._motion.set_mode(MotionState.COORD)
        self._motion.cycle_start()

        self._generate_visual_paths(self._interpreter.canon)

        self._statusbar.showMessage("G代码程序运行中...")
        self._status_panel.update_mode("运行")

    def _run_robot_program(self, text: str):
        """执行机器人程序"""
        self._robotInterpreter.init()
        self._robotInterpreter.reset()

        self._file_lines = text.split('\n')
        self._exec_line_idx = 0
        self._executing = True

        print(f"[UI] 开始执行机器人程序, 共{len(self._file_lines)}行")

        # 逐行执行
        self._execute_next_robot_line()

        # 处理规范命令
        self._motion.processCanonCommands(self._robotInterpreter.canon)
        self._motion.set_mode(MotionState.COORD)
        self._motion.cycle_start()

        # 生成可视化路径
        self._generate_visual_paths(self._robotInterpreter.canon)

        self._statusbar.showMessage("机器人程序运行中...")
        self._status_panel.update_mode("运行")

    def _execute_next_line(self):
        """逐行执行G代码"""
        editor = self._get_active_editor()
        while self._exec_line_idx < len(self._file_lines):
            line = self._file_lines[self._exec_line_idx].strip()
            editor.set_executing_line(self._exec_line_idx)

            if not line or line.startswith('(') or line.startswith(';') or line == '%':
                self._exec_line_idx += 1
                continue

            ok, err = self._interpreter.execute(line)
            if not ok:
                print(f"[UI] G代码错误 行{self._exec_line_idx + 1}: {err}")
                editor.set_error_line(self._exec_line_idx)
                self._statusbar.showMessage(f"错误 行{self._exec_line_idx + 1}: {err}")
                self._status_panel.show_message(f"错误: {err}")
                self._executing = False
                return

            print(f"[UI] 执行行{self._exec_line_idx + 1}: {line}")
            self._exec_line_idx += 1
            break

    def _execute_next_robot_line(self):
        """逐行执行机器人程序"""
        editor = self._get_active_editor()
        while self._exec_line_idx < len(self._file_lines):
            line = self._file_lines[self._exec_line_idx].strip()
            editor.set_executing_line(self._exec_line_idx)

            # 跳过空行、注释、模块声明等
            if not line or line.startswith('!'):
                self._exec_line_idx += 1
                continue
            if line.startswith(('MODULE', 'ENDMODULE')):
                self._exec_line_idx += 1
                continue

            # 设置解释器行号
            self._robotInterpreter._current_line = self._exec_line_idx

            ok, err = self._robotInterpreter.execute(line)
            if not ok:
                print(f"[UI] 机器人程序错误 行{self._exec_line_idx + 1}: {err}")
                editor.set_error_line(self._exec_line_idx)
                self._statusbar.showMessage(f"错误 行{self._exec_line_idx + 1}: {err}")
                self._status_panel.show_message(f"错误: {err}")
                self._executing = False
                return

            print(f"[UI] 执行行{self._exec_line_idx + 1}: {line}")
            self._exec_line_idx += 1
            break

    def _on_cycle_stop(self):
        self._executing = False
        editor = self._get_active_editor()
        editor.clear_executing_line()
        self._motion.cycle_stop()
        print("[UI] 程序已停止")
        self._statusbar.showMessage("程序已停止")
        self._status_panel.update_motion_state("停止")

    def _on_cycle_resume(self):
        self._motion.cycle_resume()
        print("[UI] 程序已恢复")
        self._statusbar.showMessage("程序已恢复")

    def _on_reset(self):
        self._executing = False
        editor = self._get_active_editor()
        editor.clear_executing_line()
        editor.clear_error_line()
        if self._activeInterpreter == "robot":
            self._robotInterpreter.reset()
        else:
            self._interpreter.reset()
        self._motion.reset()
        self._gl_view.clear_paths()
        zero_dict = {name: 0.0 for name in self._motion.axisNames}
        self._gl_view.update_position(zero_dict)
        self._gl_view.updateMachinePositionDict(zero_dict)
        print("[UI] 已复位")
        self._statusbar.showMessage("已复位")
        self._status_panel.update_position(zero_dict)
        self._status_panel.update_work_position(zero_dict)
        self._status_panel.update_mode("OFF")
        self._status_panel.clear_message()

    def _on_clear_paths(self):
        self._gl_view.clear_paths()
        self._statusbar.showMessage("路径已清除")

    def _on_toggle_grid(self):
        """切换网格显示/隐藏"""
        vis = not self._gl_view._showGrid
        self._gl_view._showGrid = vis
        self._btn_toggle_grid.setText("网格: 开" if vis else "网格: 关")
        self._btn_toggle_grid.setStyleSheet(
            "QPushButton { background-color: #313244; color: #a6e3a1; border: 1px solid #585b70; "
            "border-radius: 3px; padding: 1px 4px; font-size: 11px; }"
            "QPushButton:hover { background-color: #45475a; }"
            if vis else
            "QPushButton { background-color: #313244; color: #f38ba8; border: 1px solid #585b70; "
            "border-radius: 3px; padding: 1px 4px; font-size: 11px; }"
            "QPushButton:hover { background-color: #45475a; }"
        )
        self._gl_view.update()

    def _generate_visual_paths(self, canon: CanonCommand):
        """从规范命令生成3D可视化路径"""
        prevX, prevY, prevZ = 0.0, 0.0, 0.0
        for cmdEntry in canon.getCommands():
            name = cmdEntry.command_name
            params = cmdEntry.params

            if name == "STRAIGHT_TRAVERSE":
                end = params.get('end')
                if isinstance(end, EmcPose):
                    ex, ey, ez = end.tran.x, end.tran.y, end.tran.z
                elif isinstance(end, PmCartesian):
                    ex, ey, ez = end.x, end.y, end.z
                else:
                    continue
                self._gl_view.addRapidMove(prevX, prevY, prevZ, ex, ey, ez)
                prevX, prevY, prevZ = ex, ey, ez

            elif name == "STRAIGHT_FEED":
                end = params.get('end')
                if isinstance(end, EmcPose):
                    ex, ey, ez = end.tran.x, end.tran.y, end.tran.z
                elif isinstance(end, PmCartesian):
                    ex, ey, ez = end.x, end.y, end.z
                else:
                    continue
                self._gl_view.addFeedMove(prevX, prevY, prevZ, ex, ey, ez)
                prevX, prevY, prevZ = ex, ey, ez

            elif name == "ARC_FEED":
                end = params.get('end')
                center = params.get('center')
                rotation = params.get('rotation', 1)
                if isinstance(end, EmcPose) and isinstance(center, PmCartesian):
                    self._gl_view.addArcFromParams(
                        prevX, prevY, prevZ,
                        end.tran.x, end.tran.y, end.tran.z,
                        center.x, center.y,
                        clockwise=(rotation == -1)
                    )
                    prevX, prevY, prevZ = end.tran.x, end.tran.y, end.tran.z

    # ==================== MDI ====================

    def _on_mdi_command(self):
        if self._activeInterpreter == "robot":
            command, ok = QInputDialog.getText(
                self, "MDI命令", "输入RAPID命令:",
                text="MoveL p10, v100, fine, tool0"
            )
        else:
            command, ok = QInputDialog.getText(
                self, "MDI命令", "输入G代码命令:",
                text="G0 X0 Y0 Z10"
            )
        if ok and command.strip():
            print(f"[UI] MDI命令 ({self._activeInterpreter}): {command}")
            if self._motion.machine_state != MachineState.ON:
                self._on_power_on()

            if self._activeInterpreter == "robot":
                self._robotInterpreter.init()
                self._robotInterpreter.reset()
                ok, err = self._robotInterpreter.execute(command.strip())
                if ok:
                    self._motion.processCanonCommands(self._robotInterpreter.canon)
                    self._motion.set_mode(MotionState.COORD)
                    self._motion.cycle_start()
                    self._generate_visual_paths(self._robotInterpreter.canon)
                    self._statusbar.showMessage(f"MDI: {command}")
                else:
                    self._statusbar.showMessage(f"MDI错误: {err}")
                    QMessageBox.warning(self, "MDI错误", err)
            else:
                self._interpreter.init()
                self._interpreter.reset()
                ok, err = self._interpreter.execute(command.strip())
                if ok:
                    self._motion.processCanonCommands(self._interpreter.canon)
                    self._motion.set_mode(MotionState.COORD)
                    self._motion.cycle_start()
                    self._statusbar.showMessage(f"MDI: {command}")
                else:
                    self._statusbar.showMessage(f"MDI错误: {err}")
                    QMessageBox.warning(self, "MDI错误", err)

    # ==================== 刀具管理 ====================

    def _on_tool_management(self):
        """打开刀具管理对话框"""
        if self._activeInterpreter == "robot":
            self._statusbar.showMessage("机器人模式暂不支持刀具管理")
            return
        dlg = ToolDialog(self._interpreter.toolTable, self)
        dlg.tool_changed.connect(self._on_tool_changed)
        dlg.exec()

    def _on_tool_changed(self, tool_no: int):
        """换刀回调"""
        self._status_panel.update_tool(tool_no)
        self._statusbar.showMessage(f"已换刀 T{tool_no}")

    # ==================== 工件坐标管理 ====================

    def _on_wcs_management(self):
        """打开工件坐标系管理对话框"""
        if self._activeInterpreter == "robot":
            self._statusbar.showMessage("机器人模式使用WORLD坐标系")
            return
        dlg = WCSDialog(self._interpreter.workOffset, self)
        dlg.wcs_changed.connect(self._on_wcs_changed)
        dlg.exec()

    def _on_wcs_changed(self, cs_id: int):
        """坐标系切换回调"""
        names = {54: "G54", 55: "G55", 56: "G56", 57: "G57",
                 58: "G58", 59: "G59", 591: "G59.1", 592: "G59.2", 593: "G59.3"}
        self._status_panel.update_wcs(names.get(cs_id, f"G{cs_id}"))

    # ==================== 3D模型 ====================

    def _on_load_stl(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "加载STL模型", "",
            "STL文件 (*.stl);;所有文件 (*)"
        )
        if filepath:
            pos, rot = self._ask_model_transform()
            ok = self._gl_view.load_model(filepath, pos, rot)
            if ok:
                print(f"[UI] 已加载STL模型: {filepath}")
                self._statusbar.showMessage(f"已加载模型: {os.path.basename(filepath)}")
            else:
                print(f"[UI] STL模型加载失败: {filepath}")
                QMessageBox.warning(self, "加载失败", f"无法加载模型:\n{filepath}")

    def _on_load_obj(self):
        filepath, _ = QFileDialog.getOpenFileName(
            self, "加载OBJ模型", "",
            "OBJ文件 (*.obj);;所有文件 (*)"
        )
        if filepath:
            pos, rot = self._ask_model_transform()
            ok = self._gl_view.load_model(filepath, pos, rot)
            if ok:
                print(f"[UI] 已加载OBJ模型: {filepath}")
                self._statusbar.showMessage(f"已加载模型: {os.path.basename(filepath)}")
            else:
                print(f"[UI] OBJ模型加载失败: {filepath}")
                QMessageBox.warning(self, "加载失败", f"无法加载模型:\n{filepath}")

    def _ask_model_transform(self):
        """弹窗询问模型平移和旋转"""
        # 简化: 返回默认位置，用户可通过对话框自定义
        return ([0, 0, 0], [0, 0, 0])

    def _on_clear_models(self):
        self._gl_view.clear_models()
        print("[UI] 已清除所有模型")
        self._statusbar.showMessage("已清除所有模型")

    def _on_switch_machine(self, machineName: str):
        """切换机型"""
        cfg = getMachineConfig(machineName)
        if cfg is None:
            return

        print(f"[UI] 切换机型: {machineName}")
        self._current_machine = machineName

        # 1. 清除旧模型和路径
        self._gl_view.clearModels()
        self._gl_view.clear_paths()

        # 2. 设置机型名称到GL视图 (用于过程化模型)
        self._gl_view._machineName = machineName

        # 3. 加载新机型的3D模型 (STL/OBJ模式)
        if not cfg.get("use_procedural", False):
            loaded = loadMachineModels(machineName)
            for item in loaded:
                model, offset, axis_bind, parent_bind, part_name, rot_axis = item
                self._gl_view._vmcParts.append(
                    (model, offset, [0, 0, 0], axis_bind, parent_bind, part_name, rot_axis)
                )
            print(f"[UI] 加载 {len(loaded)} 个STL/OBJ模型")
        else:
            print(f"[UI] 使用过程化模型: {machineName}")

        # 4. 设置GL视图参数
        gr = cfg.get("grid_range", (-500, 500))
        gl = self._gl_view
        gl._machineXRange = gr
        gl._machineYRange = gr
        gl._gridSize = max(10.0, (gr[1] - gr[0]) / 50.0)
        gl._zoom = cfg.get("view_zoom", -300.0)
        gl._rotX = cfg.get("view_rotX", -25.0)
        gl._rotY = cfg.get("view_rotY", 225.0)
        gl._panX = 0.0
        gl._panY = 0.0
        gl._showWorktable = cfg.get("show_worktable", True)
        print(f"[UI] GL: zoom={gl._zoom} rotX={gl._rotX} rotY={gl._rotY} "
              f"grid={gr} wt={gl._showWorktable}")

        # 5. 重建JOG轴按钮
        self._control_panel.setAxes(cfg["axes"])

        # 6. 重建运动控制器
        axes = cfg["axes"]
        self._config = MotionConfig(
            num_joints=len(axes),
            coordinates="".join(axes),
            axis_names=axes,
            kinematics_name=cfg.get("kinematics", "identity"),
        )
        self._motion = MotionController(self._config)
        self._motion.set_callbacks(
            on_position=self._on_position_update,
            on_state=self._on_state_change,
            on_message=self._on_message
        )

        # 7. 重建DRO
        self._status_panel.setAxes(axes)

        # 8. 重置位置
        zero_dict = {name: 0.0 for name in axes}
        gl._currentX = 0.0
        gl._currentY = 0.0
        gl._currentZ = 0.0
        gl._machineX = 0.0
        gl._machineY = 0.0
        gl._machineZ = 0.0
        gl._axisPositions = {name: 0.0 for name in axes}
        self._status_panel.update_position(zero_dict)
        self._statusbar.showMessage(f"已切换到: {machineName} ({len(axes)}轴)")

        # 9. 延迟强制重绘
        from PySide6.QtCore import QTimer
        QTimer.singleShot(50, gl.update)

    # ==================== JOG ====================

    def _on_jog_continuous(self, axis: str, velocity: float):
        # 自动上电 - 如果未上电则自动解除急停并上电
        if self._motion.machine_state != MachineState.ON:
            if self._motion.machine_state == MachineState.ESTOP:
                self._motion.estop_off()
            self._motion.power_on()
            self._status_panel.update_mode("ON")
            print("[UI] JOG自动上电")
        self._motion.jog(axis, velocity)

    def _on_jog_stop(self, axis: str):
        print(f"[UI] JOG停止 轴={axis}")

    def _on_home_all(self):
        for i in range(self._config.num_joints):
            self._motion.home_joint(i)
        zero_dict = {name: 0.0 for name in self._motion.axisNames}
        self._gl_view.update_position(zero_dict)
        self._gl_view.updateMachinePositionDict(zero_dict)
        self._statusbar.showMessage("全轴回零完成")

    def _on_home_joint(self, joint: int):
        self._motion.home_joint(joint)
        pos_dict = self._motion.getPositionDict()
        self._gl_view.update_position(pos_dict)
        self._gl_view.updateMachinePositionDict(pos_dict)
        self._status_panel.update_position(pos_dict)
        self._statusbar.showMessage(f"轴{joint}回零完成")

    # ==================== 主轴 ====================

    def _on_spindle_cw(self):
        if self._activeInterpreter == "gcode" and self._interpreter:
            self._interpreter._setup.spindle_turning = SpindleState.CW
        speed = self._control_panel._spindle_speed.value()
        self._status_panel.update_spindle("正转", speed)

    def _on_spindle_ccw(self):
        if self._activeInterpreter == "gcode" and self._interpreter:
            self._interpreter._setup.spindle_turning = SpindleState.CCW
        speed = self._control_panel._spindle_speed.value()
        self._status_panel.update_spindle("反转", speed)

    def _on_spindle_stop(self):
        if self._activeInterpreter == "gcode" and self._interpreter:
            self._interpreter._setup.spindle_turning = SpindleState.OFF
        self._status_panel.update_spindle("停止", 0)

    # ==================== 回调 ====================

    def _on_position_update(self, pos_dict: dict):
        """位置更新回调 - 接收 {轴名: 位置} 字典"""
        self._gl_view.update_position(pos_dict)
        # 传递完整轴位置到GL视图（支持机器人轴）
        self._gl_view.updateMachinePositionDict(pos_dict)
        self._status_panel.update_position(pos_dict)

        if self._activeInterpreter == "robot":
            # 机器人模式: 工件坐标 = 机器坐标 (简化)
            self._status_panel.update_work_position(pos_dict)
        elif self._interpreter:
            # G代码模式: 工件坐标 = 机器坐标 - 坐标系偏移 - G92偏移
            work_dict = {}
            wo = self._interpreter.workOffset
            for axis, value in pos_dict.items():
                coord_off = wo.getCurrentOffset().getAxis(axis)
                g92_off = wo.g92Offset.getAxis(axis)
                work_dict[axis] = value - coord_off - g92_off
            self._status_panel.update_work_position(work_dict)

    def _on_state_change(self, state: str):
        print(f"[UI] 状态变更: {state}")
        self._status_panel.update_mode(state)

    def _on_message(self, msg: str):
        print(f"[UI] 消息: {msg}")
        self._status_panel.show_message(msg)

    def _update_status_display(self):
        if self._activeInterpreter == "robot":
            self._update_robot_status()
        elif self._interpreter:
            self._update_gcode_status()

    def _update_gcode_status(self):
        """更新G代码解释器状态"""
        s = self._interpreter._setup
        self._status_panel.update_feed(s.feed_rate)
        self._status_panel.update_tool(s.current_tool)

        wcs_names = {54: "G54", 55: "G55", 56: "G56", 57: "G57",
                     58: "G58", 59: "G59", 591: "G59.1", 592: "G59.2", 593: "G59.3"}
        self._status_panel.update_wcs(wcs_names.get(s.coord_system, "G54"))
        self._status_panel.update_units("mm" if s.length_units == 21 else "inch")

        if self._motion.status.tcq_len > 0:
            self._status_panel.update_motion_state("运行")
        else:
            self._status_panel.update_motion_state("停止")

        tool = self._interpreter.toolTable.currentTool
        self._status_panel.update_tool(tool)

        wcs = self._interpreter.workOffset.currentCoord
        self._status_panel.update_wcs(wcs_names.get(wcs, f"G{wcs}"))

    def _update_robot_status(self):
        """更新机器人解释器状态"""
        self._status_panel.update_feed(0)
        self._status_panel.update_tool(0)
        self._status_panel.update_wcs("WORLD")
        self._status_panel.update_units("mm")

        if self._motion.status.tcq_len > 0:
            self._status_panel.update_motion_state("运行")
        else:
            self._status_panel.update_motion_state("停止")

        # 显示TP缓冲区消息
        status = self._robotInterpreter.get_status()
        if status['tp_buffer']:
            last_msg = status['tp_buffer'][-1]
            self._status_panel.show_message(f"TP: {last_msg}")

    # ==================== 帮助 ====================

    def _on_about(self):
        QMessageBox.about(
            self, "关于 智能PLC仿真器",
            "<h2>智能PLC仿真器</h2>"
            "<p>版本 1.0</p>"
            "<p>基于 LinuxCNC RS274NGC 解释器的智能PLC仿真软件。</p>"
            "<p>功能：</p>"
            "<ul>"
            "<li>完整的 G 代码解释器 (G0/G1/G2/G3/G4/G10/G28/G30/G53/G54-G59.3/G80-G89)</li>"
            "<li>梯形速度规划器</li>"
            "<li>直角坐标/SCARA/Delta 运动学</li>"
            "<li>3D OpenGL 刀具路径仿真</li>"
            "<li>STL/OBJ 3D模型加载和渲染</li>"
            "<li>O-word 子程序支持</li>"
            "<li>MDI 命令输入</li>"
            "<li>手动 JOG 控制</li>"
            "</ul>"
            "<p>移植自 LinuxCNC 开源项目</p>"
        )

    def keyPressEvent(self, event):
        key = event.key()
        if key == Qt.Key_F5:
            self._on_cycle_start()
        elif key == Qt.Key_F6:
            self._on_cycle_stop()
        elif key == Qt.Key_F9:
            self._on_reset()
        elif key == Qt.Key_F1:
            self._on_mdi_command()
        elif key == Qt.Key_F2:
            self._gl_view.reset_view()
        elif key == Qt.Key_F3:
            self._on_tool_management()
        elif key == Qt.Key_F4:
            self._on_wcs_management()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        self._sim_timer.stop()
        self._servo_timer.stop()
        event.accept()
