"""机型配置表 - 定义各种机床/机器人的轴数、模型和运动学类型

5轴机型使用过程化模型 (machine_models.py)，无需STL文件。
3轴机型和机器人使用 STL/OBJ 模型文件。
"""

import os

# 模型根目录
_MODELS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'models'
)

MACHINES = {
    # ============================================================
    # VMC三轴铣床 - 使用STL模型
    # 参考: configs/sim/axis/vismach/VMC_toolchange/vmcgui
    #   lat=-75, lon=215, size=150 → zoom=-450
    # ============================================================
    "VMC三轴铣床": {
        "desc": "立式加工中心 XYZ 3轴",
        "axes": ["X", "Y", "Z"],
        "kinematics": "identity",
        "use_procedural": True,
        "models_dir": os.path.join(_MODELS_DIR, "vmc"),
        "model_files": {
            "base":      "base.stl",
            "saddle":    "saddle.stl",
            "table":     "table.stl",
            "head":      "head.stl",
            "carousel":  "carousel.stl",
            "arm":       "arm.stl",
        },
        "axis_bindings": [
            ("base",     [0, 0, 0],   None,   None),
            ("saddle",   [0, 0, 0],   "Y",    "base"),
            ("table",    [0, 0, 0],   "X",    "saddle"),
            ("head",     [0, 0, 0],   "Z",    "base"),
            ("carousel", [0, 0, 0],   None,   "base"),
            ("arm",      [0, 0, 0],   None,   "base"),
        ],
        "view_zoom": -450.0,
        "view_rotX": -75.0,
        "view_rotY": 215.0,
        "grid_range": (-300, 300),
        "show_worktable": True,
    },

    # ============================================================
    # 5轴铣床 - 使用过程化模型 (参照 LinuxCNC vismach)
    # ============================================================
    "五轴铣床(XYZAC)": {
        "desc": "五轴铣床 A+C 转台型 (Hermle风格)",
        "axes": ["X", "Y", "Z", "A", "C"],
        "kinematics": "identity",
        "use_procedural": True,
        # xyzac-trt-gui.py: lat=-60, lon=25, size=500 → zoom=-1500
        "view_zoom": -1500.0,
        "view_rotX": -60.0,
        "view_rotY": 25.0,
        "grid_range": (-800, 800),
        "show_worktable": False,
    },
    "五轴铣床(XYZBC)": {
        "desc": "五轴铣床 B+C 转台型 (Hermle风格)",
        "axes": ["X", "Y", "Z", "B", "C"],
        "kinematics": "identity",
        "use_procedural": True,
        # xyzbc-trt-gui.py: lat=-60, lon=25, size=500 → zoom=-1500
        "view_zoom": -1500.0,
        "view_rotX": -60.0,
        "view_rotY": 25.0,
        "grid_range": (-800, 800),
        "show_worktable": False,
    },
    "五轴铣床(XYZBCW)": {
        "desc": "五轴龙门铣 摆头+转台型",
        "axes": ["X", "Y", "Z", "B", "C"],
        "kinematics": "5axis",
        "use_procedural": True,
        # 5axisgui.py: lat=-65, lon=45, size=1500 → zoom=-4500
        "view_zoom": -4500.0,
        "view_rotX": -65.0,
        "view_rotY": 45.0,
        "grid_range": (-1500, 1500),
        "show_worktable": False,
    },
    "五轴铣床(XYZAB)": {
        "desc": "五轴铣床 双旋转工作台",
        "axes": ["X", "Y", "Z", "A", "B"],
        "kinematics": "identity",
        "use_procedural": True,
        # xyzab-tdr-gui.py: lat=-60, lon=0, size=500 → zoom=-1500
        "view_zoom": -1500.0,
        "view_rotX": -60.0,
        "view_rotY": 0.0,
        "grid_range": (-800, 800),
        "show_worktable": False,
    },
    "五轴铣床(MaxNC)": {
        "desc": "MaxNC 5轴 摆头+转台",
        "axes": ["X", "Y", "Z", "B", "C"],
        "kinematics": "maxkins",
        "use_procedural": True,
        # max5gui.py: 默认 lat=0, lon=0, size=500 → zoom=-1500
        "view_zoom": -1500.0,
        "view_rotX": 0.0,
        "view_rotY": 0.0,
        "grid_range": (-800, 800),
        "show_worktable": False,
    },

    # ============================================================
    # 机器人 - 使用STL/OBJ模型 + 旋转关节
    # ============================================================
    "SCARA机器人": {
        "desc": "SCARA水平关节机器人 4轴",
        "axes": ["J1", "J2", "J3", "J4"],
        "kinematics": "scara",
        "use_procedural": True,
        # scara gui: lat=-50, lon=-65, size默认→zoom=-900
        "view_zoom": -900.0,
        "view_rotX": -50.0,
        "view_rotY": -65.0,
        "grid_range": (-400, 400),
        "show_worktable": False,
    },
    "Delta并联机器人": {
        "desc": "Delta并联机器人 3轴",
        "axes": ["J1", "J2", "J3"],
        "kinematics": "delta",
        "use_procedural": True,
        # 新Delta模型高度~400单位, 俯视角度
        "view_zoom": -800.0,
        "view_rotX": -45.0,
        "view_rotY": 30.0,
        "grid_range": (-500, 500),
        "show_worktable": False,
    },
    "PUMA六轴机器人": {
        "desc": "PUMA 560 六轴工业机器人",
        "axes": ["J1", "J2", "J3", "J4", "J5", "J6"],
        "kinematics": "puma",
        "models_dir": os.path.join(_MODELS_DIR, "puma"),
        "model_files": {
            "link1": "puma_link1.obj",
            "link2": "puma_link2.obj",
            "link3": "puma_link3.obj",
            "link4": "puma_link4.obj",
            "link5": "puma_link5.obj",
            "link6": "puma_link6.obj",
            "link7": "puma_link7.obj",
        },
        "axis_bindings": [
            ("link1", [0, 0, 0],  None,   None,  None),
            ("link2", [0, 0, 0],  "J1",   "link1", "Z"),
            ("link3", [0, 0, 0],  "J2",   "link2", "Y"),
            ("link4", [0, 0, 0],  "J3",   "link3", "Y"),
            ("link5", [0, 0, 0],  "J4",   "link4", "X"),
            ("link6", [0, 0, 0],  "J5",   "link5", "Y"),
            ("link7", [0, 0, 0],  "J6",   "link6", "X"),
        ],
        # PUMA OBJ模型实际尺寸~29单位, 相机距离需匹配
        "view_zoom": -120.0,
        "view_rotX": -45.0,
        "view_rotY": 60.0,
        "grid_range": (-100, 100),
        "show_worktable": False,
    },
    "Fanuc机器人": {
        "desc": "Fanuc 200F 六轴工业机器人",
        "axes": ["J1", "J2", "J3", "J4", "J5", "J6"],
        "kinematics": "puma",
        "models_dir": os.path.join(_MODELS_DIR, "fanuc"),
        "model_files": {
            "base": "r08_base.obj",
            "j1":   "r08_j1.obj",
            "j2":   "r08_j2.obj",
            "j3":   "r08_j3.obj",
            "j4":   "r08_j4.obj",
            "j5":   "r08_j5.obj",
            "j6":   "r08_j6.obj",
        },
        "axis_bindings": [
            ("base", [0, 0, 0],  None,   None,  None),
            ("j1",   [0, 0, 0],  "J1",   "base", "Z"),
            ("j2",   [0, 0, 0],  "J2",   "j1",   "Y"),
            ("j3",   [0, 0, 0],  "J3",   "j2",   "Y"),
            ("j4",   [0, 0, 0],  "J4",   "j3",   "X"),
            ("j5",   [0, 0, 0],  "J5",   "j4",   "Y"),
            ("j6",   [0, 0, 0],  "J6",   "j5",   "X"),
        ],
        # Fanuc OBJ模型实际尺寸~88单位, 相机距离需匹配
        "view_zoom": -250.0,
        "view_rotX": -45.0,
        "view_rotY": 60.0,
        "grid_range": (-200, 200),
        "show_worktable": False,
    },
    "Router开料机": {
        "desc": "Router ATC龙门开料机 XYZ 3轴",
        "axes": ["X", "Y", "Z"],
        "kinematics": "identity",
        "models_dir": os.path.join(_MODELS_DIR, "router"),
        "model_files": {
            "bed":    "bed.obj",
            "rangka": "rangka.obj",
            "gantri": "gantri.obj",
            "head":   "head.obj",
            "headz":  "headz.obj",
            "atc":    "atc.obj",
        },
        "axis_bindings": [
            ("bed",    [0, 0, 0],  None,   None),
            ("rangka", [0, 0, 0],  None,   "bed"),
            ("gantri", [0, 0, 0],  "Y",    "bed"),
            ("head",   [0, 0, 0],  "X",    "gantri"),
            ("headz",  [0, 0, 0],  "Z",    "head"),
            ("atc",    [0, 0, 0],  None,   "head"),
        ],
        "view_zoom": -3000.0,
        "view_rotX": -25.0,
        "view_rotY": 225.0,
        "grid_range": (-2000, 2000),
        "show_worktable": True,
    },
}


def getMachineNames():
    return list(MACHINES.keys())


def getMachineConfig(name):
    return MACHINES.get(name)


def loadMachineModels(machineName):
    """加载指定机型的3D模型

    返回:
        list of (model_obj, base_offset, axis_bind, parent_name, part_name, rot_axis)
    """
    cfg = getMachineConfig(machineName)
    if cfg is None:
        return []

    models_dir = cfg.get("models_dir")
    if models_dir is None or not os.path.isdir(models_dir):
        return []

    from .models import StlModel, ObjModel
    result = []

    for part_name, filename in cfg["model_files"].items():
        filepath = os.path.join(models_dir, filename)
        if not os.path.exists(filepath):
            print(f"[机型] 缺少模型文件: {filepath}")
            continue

        ext = os.path.splitext(filename)[1].lower()
        if ext == '.stl':
            model = StlModel(filename=filepath)
        elif ext == '.obj':
            model = ObjModel(filename=filepath)
        else:
            continue

        offset = [0, 0, 0]
        axis_bind = None
        parent_bind = None
        rot_axis = None
        for bind_entry in cfg["axis_bindings"]:
            if bind_entry[0] == part_name:
                offset = bind_entry[1]
                axis_bind = bind_entry[2]
                if len(bind_entry) > 3:
                    parent_bind = bind_entry[3]
                if len(bind_entry) > 4:
                    rot_axis = bind_entry[4]
                break

        result.append((model, offset, axis_bind, parent_bind, part_name, rot_axis))

    return result
