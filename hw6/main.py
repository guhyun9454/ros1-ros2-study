import pybullet as p
import pybullet_data
import time
import numpy as np

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

p.loadURDF("plane.urdf")
table = p.loadURDF("table/table.urdf", [0.5, 0, 0], useFixedBase=True)
robot = p.loadURDF("franka_panda/panda.urdf", basePosition=[0.0, 0, 0.66], useFixedBase=True)

box_6 = p.loadURDF("models/box6.xacro", basePosition=[0.3, -0.1, 0.66])
box_5 = p.loadURDF("models/box5.xacro", basePosition=[0.3, 0.0, 0.66])
box_4 = p.loadURDF("models/box4.xacro", basePosition=[0.3, 0.1, 0.66])

box_3 = p.loadURDF("models/box3.xacro", basePosition=[0.4, -0.1, 0.66])
cylinder_0 = p.loadURDF("models/cylinder1.xacro", basePosition=[0.4, 0.0, 0.66])
box_2 = p.loadURDF("models/box2.xacro", basePosition=[0.4, 0.1, 0.66])

box_0 = p.loadURDF("models/box.xacro", basePosition=[0.5, 0.0, 0.66])
triangle = p.loadURDF("models/triangle.xacro", basePosition=[0.5, 0.1, 0.66])

case_collision = p.createCollisionShape(
    shapeType=p.GEOM_MESH,
    fileName="models/case.obj", 
    flags=p.GEOM_FORCE_CONCAVE_TRIMESH,
    meshScale=[1, 1, 1]
)

case_visual = p.createVisualShape(
    shapeType=p.GEOM_MESH,
    fileName="models/case.obj",
    meshScale=[1, 1, 1]
)

case = p.createMultiBody(
    baseMass=0.05,
    baseCollisionShapeIndex=case_collision,
    baseVisualShapeIndex=case_visual,
    basePosition=[0.0, 0.3, 0.64],
    baseOrientation=p.getQuaternionFromEuler([0, 0, 3.14159])
)


p.changeVisualShape(box_6, -1, rgbaColor=[1, 0, 1, 1])  # box
p.changeVisualShape(box_5, -1, rgbaColor=[1, 0, 0, 1])  # box2
p.changeVisualShape(box_4, -1, rgbaColor=[1, 0, 0, 1])  # box3

p.changeVisualShape(box_3, -1, rgbaColor=[1, 0, 0, 1])  # box4
p.changeVisualShape(cylinder_0, -1, rgbaColor=[0, 0, 1, 1])  # box5
p.changeVisualShape(box_2, -1, rgbaColor=[1, 0, 0, 1])  # box6

p.changeVisualShape(box_0, -1, rgbaColor=[1, 0, 0, 1])  # box5
p.changeVisualShape(triangle, -1, rgbaColor=[1, 0, 1, 1])  # box6

p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0.3, 0, 0.5])

## 기본적인 Scene 코드 이후 부분을 제공된 Task를 수행하도록 구현하세요 ##

# ----------------------------------------------------------------
# 1. 초기 설정 및 헬퍼 함수 정의 (IK, 이동, 그리퍼 제어)
# ----------------------------------------------------------------

# Case가 움직이지 않도록 고정 (필수)
p.createConstraint(case, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0],
                   p.getBasePositionAndOrientation(case)[0],
                   parentFrameOrientation=p.getBasePositionAndOrientation(case)[1])

# 로봇 관절 인덱스 찾기
num_joints = p.getNumJoints(robot)
arm_indices = [i for i in range(num_joints) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE][:7]
finger_indices = [9, 10] # Panda 그리퍼 인덱스
ee_index = 11

# 기본 자세 (Home Pose)
home_pose = [0.0, -0.5, 0.0, -2.2, 0.0, 1.7, 0.8]
base_orn = p.getQuaternionFromEuler([np.pi, 0.0, 0.0]) # 아래를 보는 방향

def step(seconds):
    """지정된 시간(초)만큼 시뮬레이션 진행"""
    for _ in range(int(seconds * 240)):
        p.stepSimulation()
        time.sleep(1./240.)

def move_joints(target_positions):
    """관절 각도 제어"""
    p.setJointMotorControlArray(robot, arm_indices, p.POSITION_CONTROL,
                                targetPositions=target_positions, forces=[200.0]*7)

def control_gripper(width):
    """그리퍼 제어 (width: 0.0 ~ 0.04)"""
    for i in finger_indices:
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition=width, force=50)

def get_ee_pose():
    """현재 End-Effector 위치 및 방향 반환"""
    state = p.getLinkState(robot, ee_index)
    return list(state[4]), list(state[5])

def move_cartesian(target_pos, target_orn, duration=1.0):
    """직선 보간을 이용한 카르테시안 이동"""
    start_pos, _ = get_ee_pose()
    steps = int(duration * 240)
    for i in range(steps):
        alpha = (i + 1) / steps
        curr_pos = [s * (1 - alpha) + t * alpha for s, t in zip(start_pos, target_pos)]
        joint_poses = p.calculateInverseKinematics(robot, ee_index, curr_pos, target_orn)
        move_joints(joint_poses[:7])
        step(duration / steps)

def go_home():
    """홈 위치로 이동"""
    move_joints(home_pose)
    step(1.0)

# ----------------------------------------------------------------
# 2. 작업 수행 함수 (Pick & Place 로직)
# ----------------------------------------------------------------

def execute_block(name, body_id, target_xy):
    # 1. 물체 높이 계산 (AABB 이용)
    (min_x, min_y, min_z), (max_x, max_y, max_z) = p.getAABB(body_id)
    obj_pos, _ = p.getBasePositionAndOrientation(body_id)
    
    # 높이 및 파라미터 설정
    pick_xy = list(obj_pos[:2])
    hover_z = 0.86
    pre_z = max_z + 0.01  # 물체 살짝 위
    
    # 기본 파라미터
    pick_orn = base_orn
    place_orn = base_orn
    grasp_width = 0.0 # 꽉 잡기
    grasp_z_ratio = 0.55 # 물체 중간보다 약간 위 잡기
    
    # --- 특수 물체 처리 ---
    if name == "triangle":
        pick_xy[0] += 0.006 # 중심 보정
        # 45도 틀어서 잡기
        pick_orn = p.getQuaternionFromEuler([np.pi, 0.0, np.pi/4.0])
        place_orn = p.getQuaternionFromEuler([np.pi, 0.0, -np.pi/4.0])
        grasp_width = 0.018 # 삼각기둥은 살짝 덜 닫아야 함
        grasp_z_ratio = 0.5
        
    elif name == "box_6":
        # 놓을 때 90도 회전 필요
        place_orn = p.getQuaternionFromEuler([np.pi, 0.0, -np.pi/2])
        
    elif "cylinder" in name:
        grasp_z_ratio = 0.6
        
    grasp_z = min_z + (max_z - min_z) * grasp_z_ratio
    
    # 2. 접근 및 잡기 (Pick)
    move_cartesian([pick_xy[0], pick_xy[1], hover_z], pick_orn, duration=0.8) # 상공 이동
    move_cartesian([pick_xy[0], pick_xy[1], pre_z], pick_orn, duration=0.8)   # 접근
    move_cartesian([pick_xy[0], pick_xy[1], grasp_z], pick_orn, duration=0.5) # 하강
    
    control_gripper(grasp_width) # 잡기
    step(0.6)
    
    move_cartesian([pick_xy[0], pick_xy[1], hover_z], pick_orn, duration=0.6) # 들어올리기
    
    # 3. 공중 회전 (필요 시)
    if name in ["box_6", "triangle"]:
        curr_pos, _ = get_ee_pose()
        move_cartesian(curr_pos, place_orn, duration=0.5)
        
    # 4. 이동 및 놓기 (Place)
    case_z = p.getBasePositionAndOrientation(case)[0][2]
    place_z = case_z + 0.065 # Case 바닥 높이 보정
    
    move_cartesian([target_xy[0], target_xy[1], hover_z], place_orn, duration=1.0) # 목표 상공
    move_cartesian([target_xy[0], target_xy[1], place_z], place_orn, duration=0.8) # 하강
    step(0.2)
    
    control_gripper(0.04) # 놓기
    step(0.5)
    
    move_cartesian([target_xy[0], target_xy[1], hover_z], place_orn, duration=0.5) # 복귀
    
    # 그리퍼 방향 원복
    if name in ["box_6", "triangle"]:
        curr_pos, _ = get_ee_pose()
        move_cartesian(curr_pos, base_orn, duration=0.5)

# ----------------------------------------------------------------
# 3. 실행 시퀀스
# ----------------------------------------------------------------

# 로봇 초기화
go_home()
control_gripper(0.04)
step(1.0)

# 순서대로 작업 실행
# (이름, ID, 목표좌표) 리스트
task_list = [
    ("box_4", box_4, (-0.16, 0.4)),
    ("box_5", box_5, (-0.15, 0.26)),
    ("box_6", box_6, (0.16, 0.26)),
    ("box_2", box_2, (0.05, 0.4)),
    ("cylinder_0", cylinder_0, (-0.05, 0.27)),
    ("box_3", box_3, (-0.05, 0.38)),
    ("box_0", box_0, (0.155, 0.365)),
    ("triangle", triangle, (0.06, 0.2625))
]

for name, obj_id, target in task_list:
    print(f"Moving {name}...")
    execute_block(name, obj_id, target)

print("All tasks completed!")
go_home()
step(2.0)

p.disconnect()

