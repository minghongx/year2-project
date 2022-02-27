import pybullet as bullet
import numpy as np
from liba1 import A1
from time import sleep

# Initialisation
physics_server_id = bullet.connect(bullet.GUI)
bullet.setRealTimeSimulation(enableRealTimeSimulation=True, physicsClientId=physics_server_id)
bullet.setGravity(0, 0, -30, physics_server_id)
import pybullet_data; bullet.setAdditionalSearchPath(pybullet_data.getDataPath())
bullet.loadURDF("plane.urdf")
a1 = A1(physics_server_id)
bullet.resetDebugVisualizerCamera(
    physicsClientId = physics_server_id,
    cameraTargetPosition = [0, 0, 0.4],
    cameraDistance = 1.5,
    cameraYaw = 40,
    cameraPitch = -15)

debug_yaw_angle = bullet.addUserDebugParameter(
    paramName = "yaw angle",
    rangeMin = -0.36,
    rangeMax = 0.36,
    startValue = 0)

reset = bullet.addUserDebugParameter(
    paramName="Reset Position",
    rangeMin=1,
    rangeMax=0,
    startValue=0)
previous_btn_value = bullet.readUserDebugParameter(reset)

sleep(1)
ref_motor_positions = a1.motor_indices.applymap(lambda index: bullet.getJointState(a1.id, index, a1.in_physics_client)[0])
while True:
    motor_positions = ref_motor_positions.copy()
    yaw_angle = bullet.readUserDebugParameter(debug_yaw_angle)

    for leg, positions in motor_positions.items():
        t0, t1, t2 = positions
        l1 = a1.thigh_len
        l2 = a1.calf_len
        L = a1.body_len / 2
        W = a1.body_width / 2
        a = a1.a
        δ = yaw_angle

        x = -l1 * np.sin(t1) - l2 * np.sin(t1 + t2)
        h = l1 * np.cos(t1) + l2 * np.cos(t1 + t2)
        match leg:
            case "fl" | "hl":
                y = -a * np.cos(t0) - h * np.sin(t0)
                z = a * np.sin(t0) - h * np.cos(t0)
            case "fr" | "hr":
                y = a * np.cos(t0) - h * np.sin(t0)
                z = -a * np.sin(t0) - h * np.cos(t0)

        match leg:
            case "fr":
                A = np.array([[ np.cos(δ), -np.sin(δ),   0,   L*np.cos(δ) - W*np.sin(δ) - L ],
                              [ np.sin(δ),  np.cos(δ),   0,   L*np.sin(δ) + W*np.cos(δ) - W ],
                              [ 0,          0,           1,   0                             ],
                              [ 0,          0,           0,   1                             ]])
            case "fl":
                A = np.array([[ np.cos(δ), -np.sin(δ),   0,   L*np.cos(δ) + W*np.sin(δ) - L ],
                              [ np.sin(δ),  np.cos(δ),   0,   L*np.sin(δ) - W*np.cos(δ) + W ],
                              [ 0,          0,           1,   0                             ],
                              [ 0,          0,           0,   1                             ]])
            case "hr":
                A = np.array([[ np.cos(δ), -np.sin(δ),   0,  -L*np.cos(δ) - W*np.sin(δ) + L ],
                              [ np.sin(δ),  np.cos(δ),   0,  -L*np.sin(δ) + W*np.cos(δ) - W ],
                              [ 0,          0,           1,   0                             ],
                              [ 0,          0,           0,   1                             ]])
            case "hl":
                A = np.array([[ np.cos(δ), -np.sin(δ),   0,  -L*np.cos(δ) + W*np.sin(δ) + L ],
                              [ np.sin(δ),  np.cos(δ),   0,  -L*np.sin(δ) - W*np.cos(δ) + W ],
                              [ 0,          0,           1,   0                             ],
                              [ 0,          0,           0,   1                             ]])
        x, y, z, _ = A.dot(np.array([x, y, z, 1]))
        h = np.sqrt(z**2 + y**2 - a**2)

        c2 = (-l1**2 - l2**2 + x**2 + h**2) / (2 * l1 * l2)
        s2 = -np.sqrt(1 - c2**2)
        positions[2] = np.arctan2(s2, c2)
        positions[1] = np.arccos((l1**2 + x**2 + h**2 - l2**2) / (2 * l1 * np.sqrt(x**2 + h**2))) - np.arctan2(x, h)
        match leg:
            case "fl" | "hl":
                positions[0] = np.arctan2(h, a) - np.arctan2(np.abs(z), np.abs(y))
            case "fr" | "hr":
                positions[0] = np.arctan2(np.abs(z), y) - np.arctan2(h, a)


    for positions, indices in zip(motor_positions.itertuples(index=False), a1.motor_indices.itertuples(index=False)):
        bullet.setJointMotorControlArray(
            physicsClientId = physics_server_id,
            bodyUniqueId = a1.id,
            jointIndices = indices,
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = positions)

    if bullet.readUserDebugParameter(reset) != previous_btn_value:
        bullet.resetBasePositionAndOrientation(a1.id, [0, 0, 0.43], [0, 0, 0, 1])
        previous_btn_value = bullet.readUserDebugParameter(reset)
