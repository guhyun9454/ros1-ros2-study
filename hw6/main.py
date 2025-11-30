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

objects = {
    "box_6": box_6, "box_5": box_5, "box_4": box_4,
    "box_3": box_3, "cylinder_0": cylinder_0, "box_2": box_2,
    "box_0": box_0, "triangle": triangle
}

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

num_joints = p.getNumJoints(robot)
arm_indices = [i for i in range(num_joints) if p.getJointInfo(robot, i)[2] == p.JOINT_REVOLUTE][:7]
finger_indices = [9, 10]
ee_index = 11
home_pose = [0.0, -0.5, 0.0, -2.2, 0.0, 1.7, 0.8]
base_orn = p.getQuaternionFromEuler([np.pi, 0.0, 0.0])
def step(seconds):
    for _ in range(int(seconds * 240)):
        p.stepSimulation()
        time.sleep(1./240.)

def move_joints(target_positions):
    p.setJointMotorControlArray(robot, arm_indices, p.POSITION_CONTROL,
                                targetPositions=target_positions, forces=[200.0]*7)

def control_gripper(width):
    for i in finger_indices:
        p.setJointMotorControl2(robot, i, p.POSITION_CONTROL, targetPosition=width, force=50)

def get_ee_pose():
    state = p.getLinkState(robot, ee_index)
    return list(state[4]), list(state[5])

def move_cartesian(target_pos, target_orn, duration=1.0):
    start_pos, _ = get_ee_pose()
    steps = int(duration * 240)
    for i in range(steps):
        alpha = (i + 1) / steps
        curr_pos = [s * (1 - alpha) + t * alpha for s, t in zip(start_pos, target_pos)]
        joint_poses = p.calculateInverseKinematics(robot, ee_index, curr_pos, target_orn)
        move_joints(joint_poses[:7])
        step(duration / steps)

def go_home():
    move_joints(home_pose)
    step(1.0)

def execute_block(name, body_id, target_xy):
    (min_x, min_y, min_z), (max_x, max_y, max_z) = p.getAABB(body_id)
    obj_pos, _ = p.getBasePositionAndOrientation(body_id)
    
    pick_xy = list(obj_pos[:2])
    hover_z = 0.85 
    pre_z = max_z + 0.01 
    
    pick_orn = base_orn
    place_orn = base_orn
    grasp_width = 0.0
    grasp_z_ratio = 0.55 
    
    if name == "triangle":
        pick_xy[0] += 0.006 
        pick_orn = p.getQuaternionFromEuler([np.pi, 0.0, np.pi/4.0])
        place_orn = p.getQuaternionFromEuler([np.pi, 0.0, -np.pi/4.0])
        grasp_width = 0.018 
        grasp_z_ratio = 0.5
        
    elif name == "box_6":
        place_orn = p.getQuaternionFromEuler([np.pi, 0.0, -np.pi/2])
        
    elif "cylinder" in name:
        grasp_z_ratio = 0.6
        
    grasp_z = min_z + (max_z - min_z) * grasp_z_ratio
    
    move_cartesian([pick_xy[0], pick_xy[1], hover_z], pick_orn, duration=0.8)
    move_cartesian([pick_xy[0], pick_xy[1], pre_z], pick_orn, duration=0.8)
    move_cartesian([pick_xy[0], pick_xy[1], grasp_z], pick_orn, duration=0.5)
    
    control_gripper(grasp_width)
    step(0.6)
    
    move_cartesian([pick_xy[0], pick_xy[1], hover_z], pick_orn, duration=0.6)
    
    if name in ["box_6", "triangle"]:
        curr_pos, _ = get_ee_pose()
        move_cartesian(curr_pos, place_orn, duration=0.5)
        
    case_z = p.getBasePositionAndOrientation(case)[0][2]
    place_z = case_z + 0.065
    
    move_cartesian([target_xy[0], target_xy[1], hover_z], place_orn, duration=1.0)
    move_cartesian([target_xy[0], target_xy[1], place_z], place_orn, duration=0.8)
    step(0.2)
    
    control_gripper(0.04)
    step(0.5)
    
    move_cartesian([target_xy[0], target_xy[1], hover_z], place_orn, duration=0.5)
    
    if name in ["box_6", "triangle"]:
        curr_pos, _ = get_ee_pose()
        move_cartesian(curr_pos, base_orn, duration=0.5)

go_home()
control_gripper(0.04)
step(1.0)

task_list = [
    ("box_4", objects["box_4"], (-0.16, 0.38)),
    ("box_5", objects["box_5"], (-0.15, 0.26)),
    ("box_6", objects["box_6"], (0.17, 0.26)),
    ("box_2", objects["box_2"], (0.05, 0.38)),
    ("cylinder_0", objects["cylinder_0"], (-0.05, 0.27)),
    ("box_3", objects["box_3"], (-0.05, 0.38)),
    ("box_0", objects["box_0"], (0.155, 0.365)),
    ("triangle", objects["triangle"], (0.06, 0.27))
]

for name, obj_id, target in task_list:
    print(f"Moving {name}...")
    execute_block(name, obj_id, target)

print("All tasks completed!")
go_home()
step(2.0)

p.disconnect()