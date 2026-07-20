"""机型配置表 - 定义各种机床/机器人的轴数、模型和运动学类型"""

import os

# 模型根目录
_MODELS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'models'
)

MACHINES = {
    "VMC三轴铣床": {
        "desc": "立式加工中心 XYZ 3轴",
        "axes": ["X", "Y", "Z"],
        "kinematics": "identity",
        "models_dir": os.path.join(_MODELS_DIR, "vmc"),
        "model_files": {
            "base":      "base.stl",
            "head":      "head.stl",
            "table":     "table.stl",
            "saddle":    "saddle.stl",
            "carousel":  "carousel.stl",
            "arm":       "arm.stl",
        },
        "axis_bindings": [
            ("base",     [0, 0, 0],   None),
            ("head",     [0, 0, 4],   "Z"),
            ("table",    [0, 8, 0],   "X"),
            ("saddle",   [0, 7, 0],   "Y"),
            ("carousel", [0, 0, 0],   None),
            ("arm",      [0, 0, 0],   None),
        ],
        "view_zoom": -300.0,
        "view_rotX": -25.0,
        "view_rotY": 225.0,
        "grid_range": (-500, 500),
        "show_worktable": True,
    },
    "五轴铣床(XYZBC)": {
        "desc": "五轴联动铣床 XYZ + B摆 C转",
        "axes": ["X", "Y", "Z", "B", "C"],
        "kinematics": "maxkins",
        "models_dir": os.path.join(_MODELS_DIR, "vmc"),
        "model_files": {
            "base":      "base.stl",
            "head":      "head.stl",
            "table":     "table.stl",
            "saddle":    "saddle.stl",
            "carousel":  "carousel.stl",
            "arm":       "arm.stl",
        },
        "axis_bindings": [
            ("base",     [0, 0, 0],   None),
            ("head",     [0, 0, 4],   "Z"),
            ("table",    [0, 8, 0],   "X"),
            ("saddle",   [0, 7, 0],   "Y"),
            ("carousel", [0, 0, 0],   None),
            ("arm",      [0, 0, 0],   None),
        ],
        "view_zoom": -300.0,
        "view_rotX": -25.0,
        "view_rotY": 225.0,
        "grid_range": (-500, 500),
        "show_worktable": True,
    },
    "SCARA机器人": {
        "desc": "SCARA水平关节机器人 4轴",
        "axes": ["J1", "J2", "J3", "J4"],
        "kinematics": "scara",
        "models_dir": None,
        "model_files": {},
        "axis_bindings": [],
        "view_zoom": -400.0,
        "view_rotX": -30.0,
        "view_rotY": 45.0,
        "grid_range": (-500, 500),
        "show_worktable": False,
    },
    "Delta并联机器人": {
        "desc": "Delta并联机器人 3轴",
        "axes": ["J1", "J2", "J3"],
        "kinematics": "delta",
        "models_dir": None,
        "model_files": {},
        "axis_bindings": [],
        "view_zoom": -400.0,
        "view_rotX": -30.0,
        "view_rotY": 45.0,
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
            ("link1", [0, 0, 0],  None),
            ("link2", [0, 0, 0],  "J1"),
            ("link3", [0, 0, 0],  "J2"),
            ("link4", [0, 0, 0],  "J3"),
            ("link5", [0, 0, 0],  "J4"),
            ("link6", [0, 0, 0],  "J5"),
            ("link7", [0, 0, 0],  "J6"),
        ],
        "view_zoom": -200.0,
        "view_rotX": -30.0,
        "view_rotY": 45.0,
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
            ("base", [0, 0, 0],  None),
            ("j1",   [0, 0, 0],  "J1"),
            ("j2",   [0, 0, 0],  "J2"),
            ("j3",   [0, 0, 0],  "J3"),
            ("j4",   [0, 0, 0],  "J4"),
            ("j5",   [0, 0, 0],  "J5"),
            ("j6",   [0, 0, 0],  "J6"),
        ],
        "view_zoom": -200.0,
        "view_rotX": -30.0,
        "view_rotY": 45.0,
        "grid_range": (-100, 100),
        "show_worktable": False,
    },
    "Router开料机": {
        "desc": "Router ATC龙门开料机 XYZ 3轴",
        "axes": ["X", "Y", "Z"],
        "kinematics": "identity",
        "models_dir": os.path.join(_MODELS_DIR, "router"),
        "model_files": {
            "bed":    "bed.obj",
            "gantri": "gantri.obj",
            "head":   "head.obj",
            "headz":  "headz.obj",
            "rangka": "rangka.obj",
            "atc":    "atc.obj",
        },
        "axis_bindings": [
            ("bed",    [0, 0, 0],  None),
            ("gantri", [0, 0, 0],  "Y"),
            ("head",   [0, 0, 0],  "X"),
            ("headz",  [0, 0, 0],  "Z"),
            ("rangka", [0, 0, 0],  None),
            ("atc",    [0, 0, 0],  None),
        ],
        "view_zoom": -3000.0,
        "view_rotX": -25.0,
        "view_rotY": 225.0,
        "grid_range": (-2000, 2000),
        "show_worktable": True,
    },
}


def getMachineNames():
    """获取所有机型名称"""
    return list(MACHINES.keys())


def getMachineConfig(name):
    """获取指定机型配置"""
    return MACHINES.get(name)


def loadMachineModels(machineName):
    """加载指定机型的3D模型

    返回:
        list of (model_obj, base_offset, axis_binding_name)
        或 空列表（无模型）
    """
    cfg = getMachineConfig(machineName)
    if cfg is None:
        return []

    models_dir = cfg.get("models_dir")
    if models_dir is None or not os.path.isdir(models_dir):
        return []

    # 根据文件扩展名选择加载器
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

        # 查找对应的轴绑定
        offset = [0, 0, 0]
        axis_bind = None
        for bind_name, bind_offset, bind_axis in cfg["axis_bindings"]:
            if bind_name == part_name:
                offset = bind_offset
                axis_bind = bind_axis
                break

        result.append((model, offset, axis_bind))

    return result
