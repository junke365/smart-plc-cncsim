"""LinuxCNC Windows 仿真器 - 启动入口

移植自 LinuxCNC 源码的 CNC 机床仿真软件。
支持 G 代码解释、轨迹规划、运动学、3D 刀具路径可视化。

运行方式:
    python -m win_sim.app
    或
    python win_sim/app.py
"""
import sys
import os

# 将项目根目录添加到路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)


def main():
    from PySide6.QtWidgets import QApplication
    from PySide6.QtCore import Qt

    app = QApplication(sys.argv)
    app.setApplicationName("LinuxCNC Windows 仿真器")
    app.setOrganizationName("LinuxCNC")
    app.setApplicationVersion("1.0.0")

    # 设置应用字体
    from PySide6.QtGui import QFont
    font = QFont("Microsoft YaHei", 9)
    app.setFont(font)

    from win_sim.gui.main_window import MainWindow
    window = MainWindow()
    window.show()

    # 如果命令行提供了文件，自动加载
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        if os.path.exists(filename):
            window._load_file(filename)

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
