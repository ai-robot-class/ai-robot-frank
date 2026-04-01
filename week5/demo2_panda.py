import pybullet as p
import pybullet_data as pd
import time
import math

# =================================================================
# 1. 환경 초기화 | 环境初始化 | Environment Initialization
# =================================================================
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.8)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

# 카메라 시점 설정 | 设置摄像头视角 | Set Camera View
p.resetDebugVisualizerCamera(cameraDistance=1.8, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.5, 0, 0.65])

# 기본 모델 로드 | 加载基础模型 | Load Basic Models
p.loadURDF("plane.urdf")
table_pos = [0.5, 0, 0] 
p.loadURDF("table/table.urdf", table_pos, useFixedBase=True)

# 로봇 암 로드 및 고정 | 加载并固定机器人 | Load and Fix Robot
panda_pos = [0.1, 0, 0.625] 
pandaId = p.loadURDF("franka_panda/panda.urdf", panda_pos, useFixedBase=True)

# =================================================================
# 2. 매개변수 및 GUI 설정 | 参数与 GUI 设置 | Parameters & GUI
# =================================================================
center_x, center_y, center_z = 0.5, 0.0, 0.95
angle = 0 

slider_r = p.addUserDebugParameter("Radius (크기/半径)", 0.1, 0.35, 0.25)
slider_speed = p.addUserDebugParameter("Speed (속도/速度)", 0.0, 0.1, 0.03)
btn_clear = p.addUserDebugParameter("Clear Trajectory (초기화/清除轨迹)", 1, 0, 0)

# 궤적 및 텍스트 관리 변수 | 轨迹及文本管理变量
prev_pos = None
line_ids = []
last_clear_val = p.readUserDebugParameter(btn_clear)
text_id = -1  # 文本 ID 初始化

print(">>> AI Robot Assignment: Trajectory Tracing Started.")

# =================================================================
# 3. 메인 로직 루프 | 核心逻辑循环 | Main Logic Loop
# =================================================================
try:
    while True:
        # GUI 파라미터 읽기 | 读取 GUI 参数
        radius = p.readUserDebugParameter(slider_r)
        speed = p.readUserDebugParameter(slider_speed)
        current_clear_val = p.readUserDebugParameter(btn_clear)

        # 궤적 초기화 체크 | 轨迹清除逻辑
        if current_clear_val > last_clear_val:
            for line in line_ids:
                p.removeUserDebugItem(line)
            line_ids = []
            last_clear_val = current_clear_val
            print("Trajectory Cleared!")

        # 원형 좌표 계산 | 计算圆周坐标
        angle += speed
        target_x = center_x + radius * math.cos(angle)
        target_y = center_y
        target_z = center_z + radius * math.sin(angle)

        # 역운동학(IK) 계산 | 逆向运动学求解
        joint_poses = p.calculateInverseKinematics(
            pandaId, 11, [target_x, target_y, target_z], 
            p.getQuaternionFromEuler([math.pi, 0, 0])
        )

        # 관절 제어 | 驱动电机
        for i in range(7):
            p.setJointMotorControl2(
                bodyIndex=pandaId, 
                jointIndex=i, 
                controlMode=p.POSITION_CONTROL, 
                targetPosition=joint_poses[i], 
                force=1000
            )

        # 궤적 표시 | 绘制轨迹
        curr_pos = [target_x, target_y, target_z]
        if prev_pos is not None and speed > 0:
            line_id = p.addUserDebugLine(
                lineFromXYZ=prev_pos, 
                lineToXYZ=curr_pos, 
                lineColorRGB=[0, 0.5, 1], 
                lineWidth=4, 
                lifeTime=0
            )
            line_ids.append(line_id)
        
        prev_pos = curr_pos

        # 텍스트 정보 업데이트 (호환성 방식) | 更新文本信息 (兼容方式)
        if text_id != -1:
            p.removeUserDebugItem(text_id)
        
        info_str = f"Target XYZ: [{target_x:.2f}, {target_y:.2f}, {target_z:.2f}]"
        text_id = p.addUserDebugText(info_str, [0.5, -0.5, 1.4], [0,0,0], 1.2)

        p.stepSimulation()
        time.sleep(1./120.)

except Exception as e:
    print(f"Program Interrupted: {e}")
finally:
    p.disconnect()