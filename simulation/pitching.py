import pybullet as bullet
import numpy as np
from liba1 import A1
from time import sleep

# Initialisation
physics_server_id = bullet.connect(bullet.GUI)
bullet.setRealTimeSimulation(enableRealTimeSimulation=True, physicsClientId=physics_server_id)
bullet.setGravity(0, 0, -9.8, physics_server_id)
import pybullet_data; bullet.setAdditionalSearchPath(pybullet_data.getDataPath())
bullet.loadURDF("plane.urdf")
a1 = A1(physics_server_id)
bullet.resetDebugVisualizerCamera(
    physicsClientId = physics_server_id,
    cameraTargetPosition = [0, 0, 0.4],
    cameraDistance = 1.5,
    cameraYaw = 40,
    cameraPitch = -15)

sleep(1)
motor_positions = a1.motor_indices.applymap(lambda index: bullet.getJointState(a1.id, index, a1.in_physics_client)[0])

# 前为 0, 顺时针为正
pitch_angle = 0.3
for leg, positions in motor_positions.items():
    # 简写以增加可读性
    t0, t1, t2 = positions
    l1 = a1.length_of_thigh
    l2 = a1.length_of_calf
    L = a1.L
    a = a1.a
    δ = pitch_angle

    # 从当前电机位置得出当前足端相对于髋关节的位置
    x = l1 * np.sin(t1) + l2 * np.sin(t1 + t2)
    h = l1 * np.cos(t1) + l2 * np.cos(t1 + t2)
    y = a * np.cos(t0) + h * np.sin(t0)
    z = -h * np.cos(t0) + a * np.sin(t0)

    # pitch 后足端相对于髋关节的位置
    match leg:
        case "fr" | "fl":
            A = np.array([[ np.cos(δ), -np.sin(δ),  L * np.cos(δ) - L ],
                          [ np.sin(δ),  np.cos(δ),  L * np.sin(δ)     ],
                          [ 0,          0,          1                 ]])
        case "hr" | "hl":
            A = np.array([[ np.cos(δ), -np.sin(δ), -L * np.cos(δ) + L ],
                          [ np.sin(δ),  np.cos(δ), -L * np.sin(δ)     ],
                          [ 0,          0,          1                 ]])
    x, z, _ = A.dot(np.array([x, z, 1]))
    h = np.sqrt(z**2 + y**2 - a**2)

    # 从 pitch 后足端相对于髋关节的位置反算电机位置
    c2 = (-l1**2 - l2**2 + x**2 + h**2) / (2 * l1 * l2)
    s2 = -np.sqrt(1 - c2**2)  # sin 取负是因为模型中的 knee position 规定为负
    positions[2] = np.arctan2(s2, c2)
    positions[1] = np.arccos((l1**2 + x**2 + h**2 - l2**2) / (2 * l1 * np.sqrt(x**2 + h**2))) - np.arctan2(x, h)

for positions, indices in zip(motor_positions.itertuples(index=False), a1.motor_indices.itertuples(index=False)):
    print(indices)
    print(positions)
    bullet.setJointMotorControlArray(
        physicsClientId = physics_server_id,
        bodyUniqueId = a1.id,
        jointIndices = indices,
        controlMode = bullet.POSITION_CONTROL,
        targetPositions = positions,)

sleep(20)