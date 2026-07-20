"""综合单元测试 - 验证核心仿真引擎所有模块"""
import sys
import os
import math

# 添加项目路径
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from win_sim.core.coordinates import (
    PmCartesian, EmcPose, PmCircle, PmRotationMatrix,
    pmMatRotX, pmMatRotY, pmMatRotZ, pmMatMult, pmMatVecMult,
    pmRpyMatConvert, pmPoseDistance, pmCircleLength, LINEAR_INTERPOLATION
)
from win_sim.core.canon import CanonCommand, CanonCommandEntry
from win_sim.core.interpreter import Interpreter, MotionMode, Block
from win_sim.core.trajectory import TrajectoryPlanner, TC, MotionType, TermCond
from win_sim.core.kinematics import IdentityKinematics, ScaraKinematics, DeltaKinematics, SwitchKinematics
from win_sim.core.motion import MotionController, MotionState, MachineState, MotionConfig

passed = 0
failed = 0
errors = []

def test(name, condition, detail=""):
    global passed, failed
    if condition:
        passed += 1
        print(f"  [PASS] {name}")
    else:
        failed += 1
        msg = f"  [FAIL] {name}"
        if detail:
            msg += f" - {detail}"
        print(msg)
        errors.append(name)

print("=" * 60)
print("综合单元测试")
print("=" * 60)

# ==================== 1. 坐标模块 ====================
print("\n--- 1. 坐标模块 (coordinates) ---")

p1 = PmCartesian(1.0, 2.0, 3.0)
p2 = PmCartesian(4.0, 6.0, 3.0)
test("PmCartesian 创建", p1.x == 1.0 and p1.y == 2.0 and p1.z == 3.0)
test("PmCartesian 向量长度", math.isclose(p1.mag(), math.sqrt(14)))
test("PmCartesian 加法", (p1 + p2) == PmCartesian(5.0, 8.0, 6.0))
test("PmCartesian 减法", (p2 - p1) == PmCartesian(3.0, 4.0, 0.0))
test("PmCartesian 标量乘法", (p1 * 2) == PmCartesian(2.0, 4.0, 6.0))
test("PmCartesian 点积", math.isclose(p1.dot(p2), 25.0))
test("PmCartesian 距离", math.isclose(p1.distance_to(p2), 5.0))

pose1 = EmcPose(tran=PmCartesian(10, 20, 30), a=45.0)
test("EmcPose 创建", pose1.tran.x == 10 and pose1.a == 45.0)
test("EmcPose 复制", pose1.copy() == pose1)

mat_id = PmRotationMatrix.identity()
test("旋转矩阵 单位矩阵", mat_id.s[0][0] == 1.0 and mat_id.s[1][1] == 1.0)

mat_x = pmMatRotX(math.pi / 2)
v = pmMatVecMult(mat_x, PmCartesian(0, 1, 0))
test("绕X旋转90°", math.isclose(v.x, 0, abs_tol=0.001) and math.isclose(v.y, 0, abs_tol=0.001) and math.isclose(v.z, 1, abs_tol=0.001))

mat_y = pmMatRotY(math.pi / 2)
v = pmMatVecMult(mat_y, PmCartesian(1, 0, 0))
test("绕Y旋转90°", math.isclose(v.x, 0, abs_tol=0.001) and math.isclose(v.z, -1, abs_tol=0.001))

circle = PmCircle(center=PmCartesian(0, 0, 0), radius=10.0, start_angle=0, end_angle=math.pi)
test("圆弧长度", math.isclose(pmCircleLength(circle), math.pi * 10))

# ==================== 2. 规范命令模块 ====================
print("\n--- 2. 规范命令模块 (canon) ---")

canon = CanonCommand()
test("CanonCommand 创建", len(canon.getCommands()) == 0)

canon.STRAIGHT_TRAVERSE(PmCartesian(10, 20, 30))
test("STRAIGHT_TRAVERSE 记录", len(canon.getCommands()) == 1)
test("STRAIGHT_TRAVERSE 命令名", canon.getCommands()[0].command_name == "STRAIGHT_TRAVERSE")

canon.STRAIGHT_FEED(PmCartesian(5, 10, 0))
canon.ARC_FEED(PmCartesian(5, 10, 0), PmCartesian(0, 10, 0), 0, 1, 500.0)
canon.SET_FEED_RATE(1000.0)
canon.START_SPINDLE_CLOCKWISE(1000, 0)
test("多命令记录", len(canon.getCommands()) == 5)

canon.clear()
test("清空命令", len(canon.getCommands()) == 0)

# ==================== 3. 解释器模块 ====================
print("\n--- 3. 解释器模块 (interpreter) ---")

interp = Interpreter()
interp.init()
test("解释器初始化", interp.error == "")

# 基本G代码执行
ok, err = interp.execute("G0 X10 Y20 Z5")
test("G0 快移执行", ok, err)
test("G0 目标位置", math.isclose(interp._setup.current_x, 10) and math.isclose(interp._setup.current_y, 20) and math.isclose(interp._setup.current_z, 5))

ok, err = interp.execute("G1 X30 Y40 F500")
test("G1 直线进给", ok, err)
test("G1 进给率设置", math.isclose(interp._setup.feed_rate, 500))

# 圆弧 G2/G3
interp.reset()
ok, err = interp.execute("G1 X10 Y0 F1000")
ok, err = interp.execute("G2 X0 Y10 I0 J0")
test("G2 顺时针圆弧", ok, err)
test("G2 终点坐标", math.isclose(interp._setup.current_x, 0, abs_tol=0.01) and math.isclose(interp._setup.current_y, 10, abs_tol=0.01))

# 模态G代码
interp.reset()
ok, err = interp.execute("G1 X5 Y5 F500")
ok, err = interp.execute("X10 Y10")
test("模态G1 (续)", math.isclose(interp._setup.current_x, 10) and math.isclose(interp._setup.current_y, 10))

# M代码
interp.reset()
ok, err = interp.execute("M3 S1000")
test("M3 主轴正转", ok)
test("M3 主轴状态", interp._setup.spindle_turning == 1)

ok, err = interp.execute("M5")
test("M5 主轴停止", interp._setup.spindle_turning == 0)

# G90/G91 距离模式
interp.reset()
ok, err = interp.execute("G91")
test("G91 增量模式", interp._setup.distance_mode == 91)

ok, err = interp.execute("G0 X10 Y0 Z0")
ok, err = interp.execute("X5 Y5 Z-2")
test("增量模式运动", math.isclose(interp._setup.current_x, 15) and math.isclose(interp._setup.current_y, 5))

# 参数系统
interp.reset()
ok, err = interp.execute("#1 = 42")
test("参数赋值 #1=42", ok)
test("参数读取", math.isclose(interp._setup.parameters[1], 42))

# 注释处理
ok, err = interp.execute("(这是注释) G0 X0")
test("注释跳过", ok)

# 坐标系
interp.reset()
ok, err = interp.execute("G55")
test("G55 坐标系切换", interp._setup.coord_system == 55)

# 暂停
interp.reset()
ok, err = interp.execute("G4 P1000")
test("G4 暂停", ok)

# 文件执行
interp.reset()
ok, err = interp.execute_file()
test("空文件执行", ok)

# 检查 canon 输出
interp.reset()
interp.execute("G0 X10 Y0 Z0")
interp.execute("G1 X20 Y10 Z-5 F500")
cmds = interp.canon.getCommands()
cmd_names = [c.command_name for c in cmds]
test("canon 包含 STRAIGHT_TRAVERSE", "STRAIGHT_TRAVERSE" in cmd_names)
test("canon 包含 STRAIGHT_FEED", "STRAIGHT_FEED" in cmd_names)

# ==================== 4. 轨迹规划器 ====================
print("\n--- 4. 轨迹规划器 (trajectory) ---")

tp = TrajectoryPlanner()
test("TP 创建", tp.is_idle())
test("TP 队列为空", tp.queue_length == 0)

tc = tp.add_line(100, 0, 0, 1000, 50000)
test("add_line 创建TC", tc is not None)
test("TC 行程长度", math.isclose(tc.target, 100.0))
test("TP 队列长度", tp.queue_length == 1)

# 执行周期
for _ in range(100):
    result = tp.run_cycle(0.001)
test("run_cycle 返回结果", result is not None)
test("位置更新", result['x'] > 0)

# 圆弧
tp2 = TrajectoryPlanner()
tp2.current_x = 0
tp2.current_y = 0
tc_arc = tp2.add_arc(10, 10, 0, 0, 10, 0, 1, 500, 50000)
test("add_arc 创建圆弧TC", tc_arc is not None)
test("圆弧类型", tc_arc.motion_type == MotionType.CIRCULAR)
test("圆弧半径", math.isclose(tc_arc._radius, 10.0, rel_tol=0.01))

# 速度规划
tp3 = TrajectoryPlanner()
tp3.add_line(1000, 0, 0, 60000, 50000)
for _ in range(50):
    r = tp3.run_cycle(0.001)
tc = tp3.queue[0] if tp3.queue else None
if tc:
    test("速度规划 进行中", tc.progress > 0)
    test("速度规划 速度>0", tc.currentvel > 0)

# ==================== 5. 运动学模块 ====================
print("\n--- 5. 运动学模块 (kinematics) ---")

kins = IdentityKinematics("XYZ")
test("Identity 创建", kins.num_joints == 3)

pos, ok = kins.forward([10.0, 20.0, 30.0])
test("正运动学", ok)
test("正运动学 X", math.isclose(pos.tran.x, 10.0))
test("正运动学 Y", math.isclose(pos.tran.y, 20.0))
test("正运动学 Z", math.isclose(pos.tran.z, 30.0))

joints, ok = kins.inverse(EmcPose(tran=PmCartesian(10, 20, 30)))
test("逆运动学", ok)
test("逆运动学 X", math.isclose(joints[0], 10.0))
test("逆运动学 Y", math.isclose(joints[1], 20.0))
test("逆运动学 Z", math.isclose(joints[2], 30.0))

# SCARA
scara = ScaraKinematics(200, 150, 100)
pos, ok = scara.forward([30, 45, 50])
test("SCARA 正运动学", ok)
test("SCARA X范围", -350 < pos.tran.x < 350)

joints, ok = scara.inverse(pos)
test("SCARA 逆运动学", ok)
if ok:
    pos2, ok2 = scara.forward(joints)
    test("SCARA 正逆一致性 X", math.isclose(pos.tran.x, pos2.tran.x, rel_tol=0.01))
    test("SCARA 正逆一致性 Y", math.isclose(pos.tran.y, pos2.tran.y, rel_tol=0.01))

# Delta
delta = DeltaKinematics(150, 300)
pos, ok = delta.forward([30, 30, 30])
test("Delta 正运动学", ok)
joints, ok = delta.inverse(pos)
test("Delta 逆运动学", ok)

# Switch
sw = SwitchKinematics()
sw.register("identity", IdentityKinematics("XYZ"))
test("Switch 注册", len(sw.list_available()) == 1)
test("Switch 切换", sw.switch("identity"))
test("Switch 就绪", sw.is_ready)

# ==================== 6. 运动控制器 ====================
print("\n--- 6. 运动控制器 (motion) ---")

config = MotionConfig(num_joints=3, max_velocity=5000, max_acceleration=50000)
mc = MotionController(config)
test("MotionController 创建", mc.machineState == MachineState.ESTOP)

mc.estopOff()
test("解除急停", mc.machineState == MachineState.OFF)

mc.powerOn()
test("上电", mc.machineState == MachineState.ON)
test("运动模式 FREE", mc.motionState == MotionState.FREE)

# JOG
mc.jog('X', 1000)
test("JOG X", math.isclose(mc.currentX, 1000 * 0.001 / 60.0, rel_tol=0.01))

mc.jog('Y', 1000)
mc.jog('Z', 500)
test("JOG 多轴", mc.currentY > 0)

# 回零
mc.homeAll()
test("全轴回零", mc.currentX == 0 and mc.currentY == 0 and mc.currentZ == 0)
test("全轴已回零", mc.isAllHomed())

# 完整流程: 解释器 → 规范命令 → 运动控制器
print("\n  --- 6a. 端到端流程测试 ---")
interp2 = Interpreter()
interp2.init()

interp2.execute("G0 X50 Y0 Z10")
interp2.execute("G1 X100 Y50 Z-5 F1000")

mc2 = MotionController(config)
mc2.powerOn()
mc2.set_mode(MotionState.COORD)
mc2.processCanonCommands(interp2.canon)
mc2.cycle_start()

# 运行足够周期完成所有运动
for _ in range(5000):
    mc2.runCycle()

test("端到端 最终X", math.isclose(mc2.currentX, 100, rel_tol=0.01))
test("端到端 最终Y", math.isclose(mc2.currentY, 50, rel_tol=0.01))
test("端到端 最终Z", math.isclose(mc2.currentZ, -5, rel_tol=0.01))
test("端到端 TP空闲", mc2.tp.is_idle())

# 圆弧端到端
print("\n  --- 6b. 圆弧端到端测试 ---")
interp3 = Interpreter()
interp3.init()
interp3.execute("G0 X0 Y0 Z0")
interp3.execute("G1 X10 Y0 Z0 F500")
interp3.execute("G2 X0 Y10 I0 J0 F500")

mc3 = MotionController(config)
mc3.powerOn()
mc3.set_mode(MotionState.COORD)
mc3.processCanonCommands(interp3.canon)
mc3.cycle_start()

for _ in range(5000):
    mc3.runCycle()

test("圆弧端到端 最终X", math.isclose(mc3.currentX, 0, abs_tol=0.1))
test("圆弧端到端 最终Y", math.isclose(mc3.currentY, 10, abs_tol=0.1))

# ==================== 7. O-word 子程序 ====================
print("\n--- 7. O-word 子程序测试 ---")

interp4 = Interpreter()
interp4.init()
interp4.execute("O100 sub")
interp4.execute("G0 X[#1] Y[#2] Z0")
interp4.execute("O100 endsub")
interp4.execute("#1 = 25")
interp4.execute("#2 = 35")
interp4.execute("O100 call")
test("O-word 子程序定义", 100 in interp4._setup.subroutines)
test("O-word 子程序执行", math.isclose(interp4._setup.current_x, 25) and math.isclose(interp4._setup.current_y, 35))

# O-word IF/ENDIF
interp5 = Interpreter()
interp5.init()
interp5.execute("#1 = 1")
interp5.execute("O10 if [#1 EQ 1]")
interp5.execute("G0 X100 Y0 Z0")
interp5.execute("O10 endif")
test("O-word IF 执行", math.isclose(interp5._setup.current_x, 100))

interp5.reset()
interp5.execute("#1 = 0")
interp5.execute("O10 if [#1 EQ 1]")
interp5.execute("G0 X100 Y0 Z0")
interp5.execute("O10 endif")
test("O-word IF 跳过", math.isclose(interp5._setup.current_x, 0))

# ==================== 8. 固定循环 ====================
print("\n--- 8. 固定循环测试 ---")

interp6 = Interpreter()
interp6.init()
interp6.execute("G90")
interp6.execute("G81 X10 Y10 Z-5 R2 F500")
test("G81 固定循环", len(interp6.canon.getCommands()) > 0)

cmds = interp6.canon.getCommands()
traverse_cmds = [c for c in cmds if c.command_name == "STRAIGHT_TRAVERSE"]
feed_cmds = [c for c in cmds if c.command_name == "STRAIGHT_FEED"]
test("G81 有快移命令", len(traverse_cmds) > 0)
test("G81 有进给命令", len(feed_cmds) > 0)

# G83 啄钻循环
interp6.reset()
interp6.execute("G90")
interp6.execute("G83 X20 Y20 Z-10 R2 Q3 F300")
cmds83 = interp6.canon.getCommands()
test("G83 啄钻 循环有多个行程", len(cmds83) > 5)

# ==================== 9. 轨迹规划器 圆弧精确端到端 ====================
print("\n--- 9. 圆弧精确性测试 ---")

tp_arc = TrajectoryPlanner()
tp_arc.current_x = 0
tp_arc.current_y = 0
tp_arc.add_arc(10, 10, 0, 0, 10, 0, 1, 1000, 50000)

final_x, final_y = 0, 0
for _ in range(10000):
    r = tp_arc.run_cycle(0.001)
    if r:
        final_x, final_y = r['x'], r['y']

test("圆弧TP 最终X", math.isclose(final_x, 10, abs_tol=0.5))
test("圆弧TP 最终Y", math.isclose(final_y, 10, abs_tol=0.5))

# ==================== 10. 文件执行 ====================
print("\n--- 10. 文件执行测试 ---")

example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'examples', 'circle.ngc')
if os.path.exists(example_file):
    interp_file = Interpreter()
    interp_file.init()
    ok, err = interp_file.open_file(example_file)
    test("打开示例文件", ok)
    if ok:
        ok, err = interp_file.execute_file()
        test("执行示例文件", ok, err)
        cmds = interp_file.canon.getCommands()
        test("示例文件生成命令", len(cmds) > 5)
else:
    print("  [SKIP] 示例文件不存在")

# ==================== 11. 机器人解释器 ====================
print("\n--- 11. 机器人解释器 (robot_interpreter) ---")

from win_sim.core.robot_interpreter import (
    RobotInterpreter, RobotTarget, JointTarget, SpeedData, ZoneData, ToolData,
    VarType, InstructionType
)

ri = RobotInterpreter()
test("机器人解释器创建", ri is not None)
test("机器人解释器有Canon", ri.canon is not None)

# 测试变量初始化
test("预定义速度变量", ri._var_exists("v100"))
test("预定义区域变量", ri._var_exists("fine"))
test("预定义工具变量", ri._var_exists("tool0"))
test("预定义home点", ri._var_exists("home"))

# 测试单行执行
ri.init()
ok, err = ri.execute('MoveJ p10, v100, fine, tool0')
test("MoveJ执行", ok, err)

ok, err = ri.execute('MoveL p20, v200, z10, tool0')
test("MoveL执行", ok, err)

ok, err = ri.execute('MoveC p30, p40, v100, z5, tool0')
test("MoveC执行", ok, err)

# 测试赋值
ok, err = ri.execute('counter := 0')
test("变量赋值", ok, err)
test("变量值正确", ri._get_var("counter") == 0)

ok, err = ri.execute('counter := counter + 1')
test("变量运算赋值", ok, err)
test("变量运算结果", ri._get_var("counter") == 1)

ok, err = ri.execute('name := "test"')
test("字符串赋值", ok, err)
test("字符串值正确", ri._get_var("name") == "test")

# 测试TPWRITE
ok, err = ri.execute('TPWRITE "Hello Robot"')
test("TPWRITE执行", ok, err)
test("TP缓冲区有内容", len(ri._tp_buffer) > 0)

# 测试IF/ELSEIF/ELSE
ri.init()
ok, err = ri.execute('mode := 2')
ok, err = ri.execute('IF (mode = 1) THEN')
ok, err = ri.execute('result := 10')
ok, err = ri.execute('ELSEIF (mode = 2) THEN')
ok, err = ri.execute('result := 20')
ok, err = ri.execute('ELSE')
ok, err = ri.execute('result := 30')
ok, err = ri.execute('ENDIF')
test("IF/ELSEIF/ELSE正确选择", ri._get_var("result") == 20)

# 测试条件表达式
test("条件 = ", ri._eval_condition("1 = 1"))
test("条件 <> ", ri._eval_condition("1 <> 2"))
test("条件 < ", ri._eval_condition("1 < 2"))
test("条件 > ", ri._eval_condition("2 > 1"))
test("条件 <= ", ri._eval_condition("1 <= 1"))
test("条件 >= ", ri._eval_condition("2 >= 1"))
test("条件 AND ", ri._eval_condition("1 = 1 AND 2 = 2"))
test("条件 OR ", ri._eval_condition("1 = 2 OR 2 = 2"))
test("条件 NOT ", ri._eval_condition("NOT (1 = 2)"))

# 测试GOTO
ri.init()
ri._labels = {"test_label": 2}
ok, err = ri.execute('GOTO test_label')
test("GOTO跳转", ok, err)

# 测试BREAK/EXIT
ri.init()
ok, err = ri.execute('BREAK')
test("BREAK执行", ok)
test("BREAK标志", ri._break_flag)

ri.init()
ok, err = ri.execute('EXIT')
test("EXIT执行", ok)
test("EXIT标志", ri._exit_flag)

# 测试文件执行
robot_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'examples', 'robot_motion_demo.rap')
if os.path.exists(robot_file):
    ri_file = RobotInterpreter()
    ri_file.init()
    ok, err = ri_file.open_file(robot_file)
    test("打开机器人示例文件", ok)
    if ok:
        ok, err = ri_file.execute_file()
        test("执行机器人示例文件", ok, err)
        cmds = ri_file.canon.getCommands()
        test("机器人示例生成命令", len(cmds) > 3)
else:
    print("  [SKIP] 机器人示例文件不存在")

# 测试RobotTarget数据
rt = RobotTarget(x=100, y=200, z=300)
test("RobotTarget创建", rt.x == 100 and rt.y == 200 and rt.z == 300)

# 测试SpeedData
sd = SpeedData(v=500, v_tcp=500)
test("SpeedData创建", sd.v == 500)

# 测试ZoneData
zd = ZoneData(zone_name="z10", pzone_tcp=2)
test("ZoneData创建", zd.zone_name == "z10")

# 测试JointTarget
jt = JointTarget(joints=[10, 20, 30, 40, 50, 60])
test("JointTarget创建", jt.joints[0] == 10 and jt.joints[5] == 60)

# ==================== 总结 ====================
print("\n" + "=" * 60)
print(f"测试结果: {passed} 通过, {failed} 失败")
if errors:
    print(f"失败项: {', '.join(errors)}")
print("=" * 60)

sys.exit(0 if failed == 0 else 1)
