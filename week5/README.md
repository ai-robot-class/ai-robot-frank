🤖 机器人学核心概念：空间与运动学在机器人控制中，我们通常在两套不同的“坐标系”下描述机器人的状态。  
1. 核心空间对比特性关节空间 (Joint Space)笛卡尔坐标空间 (Cartesian Space)定义维度每一个关节的状态向量

 $q = [\theta_1, \theta_2, ..., \theta_n]$
 
 末端位姿向量
 
  $P = [x, y, z, Roll, Pitch, Yaw]$
  
  描述对象机器人内部：电机的转动角度或伸缩距离外部世界：末端执行器在三维世界的位置坐标单位弧度 (rad) 或 角度 (deg)米 (m) 或 毫米 (mm)直观程度抽象：人很难直观想象角度对应的空间位置直观：符合人类对前后、上下、左右的认知主要用途电机伺服控制、检查是否达到物理限位任务路径规划、目标抓取、避障2. 空间转换：运动学 (Kinematics)机器人系统的核心逻辑就在于这两个空间之间的相互“翻译”：🔄 正向运动学 (Forward Kinematics, FK)逻辑：已知 关节角度 $\rightarrow$ 推算 末端位置。特点：计算简单、结果唯一。只要确定了每个关节的姿态，机械手在空间的位置就是确定的。🔄 逆向运动学 (Inverse Kinematics, IK)逻辑：给定 目标坐标 $\rightarrow$ 求解 关节角度。特点：计算复杂、存在多解或无解。例如：你可以保持手部不动，但通过旋转肘部（手肘向上或向下）来改变姿态，这就是典型的“多解”现象。3. 实验总结关节输入模式 (FK)：通过手动调节 J0-J6 滑块，你是在控制机器人的“骨骼”旋转，观察末端坐标如何随之改变。坐标输入模式 (IK)：通过输入 X, Y, Z 指令，让计算机通过算法计算出 7 个关节该如何协同运动以到达目标点。💡 提示：在实际开发中，我们通常在坐标空间规划任务（比如“去拿杯子”），而在关节空间执行控制（向电机发送脉冲信号）

<img src="../img/img3.png" alt="panda机器人演示" width="500">

在pybullet_robots 目录下新建一个python程序文件并运行

注意：提交作业实在自己的目录复制程序文件提交

# 作业：
1. 使用空间坐标系控制机械臂画一个圆
2. 使用关节坐标系控制机械臂画一段直线
```
import pybullet as p
import pybullet_data as pd
import time
import math

# --- 1. 환경 초기화 | 环境初始化 | Environment Initialization ---
p.connect(p.GUI)
p.setAdditionalSearchPath(pd.getDataPath())
p.setGravity(0, 0, -9.8) # 표준 Z-Up 중력 | 标准Z轴重力 | Standard Z-Up gravity
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
# 카메라 위치 설정 | 设置摄像头视角 | Set camera view
p.resetDebugVisualizerCamera(1.5, 45, -30, [0.5, 0, 0.65])

# 지면 및 테이블 로드 | 加载地面与桌子 | Load plane and table
p.loadURDF("plane.urdf")
table_pos = [0.5, 0, 0] 
p.loadURDF("table/table.urdf", table_pos, useFixedBase=True)

# 로봇 암 로드 (테이블 위에 고정) | 加载并固定机器人 | Load & fix robot on table
panda_pos = [0.5, 0, 0.625] # Z=0.625는 테이블 높이 | 桌面高度 | Table height
pandaId = p.loadURDF("franka_panda/panda.urdf", panda_pos, useFixedBase=True)

# --- 2. 컨트롤 패널 생성 | 创建控制面板 | Create Control Panel ---
# 모드 전환 스위치 (체크 시 IK 모드) | 模式切换开关 | Mode toggle (Checked = IK)
mode_toggle = p.addUserDebugParameter("RUN IK (Checked) / RUN JOINT (Unchecked)", 1, 0, 0)

# A. 데카르트 좌표계 슬라이더 | 笛卡尔坐标滑块 | Cartesian Sliders (X, Y, Z)
p.addUserDebugText("--- CARTESIAN SETTINGS ---", [1.2, 0.5, 1.2], [0,0,1], 1)
ctrl_x = p.addUserDebugParameter("Target_X", 0.3, 0.8, 0.6)
ctrl_y = p.addUserDebugParameter("Target_Y", -0.4, 0.4, 0.0)
ctrl_z = p.addUserDebugParameter("Target_Z", 0.65, 1.2, 0.8)

# B. 관절 공간 슬라이더 | 关节空间滑块 | Joint Space Sliders (J0-J6)
p.addUserDebugText("--- JOINT SETTINGS ---", [1.2, -0.5, 1.2], [0,0.5,0], 1)
joint_params = []
joint_names = ["J0_Base", "J1_Shoulder", "J2_Arm", "J3_Elbow", "J4_Forearm", "J5_Wrist", "J6_Flange"]
# Panda 관절 한계 설정 | 关节限位设置 | Joint limits for Panda
joint_limits = [(-2.89, 2.89), (-1.76, 1.76), (-2.89, 2.89), (-3.07, -0.06), (-2.89, 2.89), (-0.01, 3.75), (-2.89, 2.89)]
for i in range(7):
    joint_params.append(p.addUserDebugParameter(joint_names[i], joint_limits[i][0], joint_limits[i][1], 0.0))

info_id = -1 # 디버그 텍스트 ID | 调试文本ID | Debug text ID

# --- 3. 메인 로직 루프 | 核心逻辑循环 | Main Logic Loop ---
try:
    while True:
        # 스위치 상태 확인 | 读取开关状态 | Read mode toggle state
        run_ik = p.readUserDebugParameter(mode_toggle)
        
        if run_ik > 0.5:
            # ======= 모드 1: 역운동학 (IK) | 模式1: 逆向运动学 | Mode 1: Inverse Kinematics =======
            tx = p.readUserDebugParameter(ctrl_x)
            ty = p.readUserDebugParameter(ctrl_y)
            tz = p.readUserDebugParameter(ctrl_z)
            
            # IK 계산: 목표 좌표 -> 관절 각도 | 计算关节角度 | Calculate joint angles from XYZ
            joint_poses = p.calculateInverseKinematics(
                pandaId, 11, [tx, ty, tz], 
                p.getQuaternionFromEuler([math.pi, 0, 0]) # 집게가 아래를 향하도록 설정 | 保持夹爪向下 | Gripper face down
            )
            
            # 모터 제어 적용 | 应用电机控制 | Apply motor control
            for i in range(7):
                p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, joint_poses[i], force=500)
            
            mode_str = "CURRENT MODE: CARTESIAN (IK)"
        
        else:
            # ======= 모드 2: 관절 직접 제어 (FK) | 模式2: 关节直接控制 | Mode 2: Joint Direct Control =======
            for i in range(7):
                target_val = p.readUserDebugParameter(joint_params[i])
                p.setJointMotorControl2(pandaId, i, p.POSITION_CONTROL, target_val, force=500)
            
            mode_str = "CURRENT MODE: JOINT SPACE (Direct)"

        # --- 4. 데이터 피드백 표시 | 数据反馈显示 | Data Feedback Display ---
        # 엔드 이펙터 실제 위치 획득 (순운동학 결과) | 获取末端实际位置 | Get actual EE position (FK result)
        ee_state = p.getLinkState(pandaId, 11)
        curr_p = ee_state[0]
        # 실제 관절 각도 읽기 | 读取实际关节角度 | Read actual joint states
        curr_j = [p.getJointState(pandaId, i)[0] for i in range(7)]

        # 화면 텍스트 업데이트 | 更新屏幕文本 | Update screen text
        if info_id != -1:
            p.removeUserDebugItem(info_id)
        
        display_text = f"{mode_str}\n"
        display_text += f"End-Effector [X,Y,Z]: [{curr_p[0]:.2f}, {curr_p[1]:.2f}, {curr_p[2]:.2f}]\n"
        display_text += "-"*30 + "\n"
        display_text += "Real-time Joints (rad):\n" + "\n".join([f"J{i}: {curr_j[i]:.2f}" for i in range(7)])
        
        # 텍스트 위치 및 색상 설정 | 设置文本位置与颜色 | Set text position and color
        info_id = p.addUserDebugText(display_text, [0.4, -0.8, 0.8], [0,0,0], 1.2)

        p.stepSimulation() # 시뮬레이션 한 단계 진행 | 步进仿真 | Step simulation
        time.sleep(1./120.) # 실행 속도 조절 | 控制运行频率 | Control execution frequency

except Exception as e:
    print(f"Error occurred: {e}")
finally:
    p.disconnect() # 연결 종료 | 断开连接 | Disconnect
```