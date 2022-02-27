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
    cameraYaw = 90,
    cameraPitch = 0)

debug_roll_angle = bullet.addUserDebugParameter(
    paramName = "roll angle (rad)",
    rangeMin = -0.7,
    rangeMax = 0.7,
    startValue = 0,)

sleep(1)
ref_motor_positions = a1.motor_indices.applymap(lambda index: bullet.getJointState(a1.id, index, a1.in_physics_client)[0])
while True:
    motor_positions = ref_motor_positions.copy()  # Calculate the rolling angle with reference (ref position)
    roll_angle = bullet.readUserDebugParameter(debug_roll_angle)  # Clockwise is positive

    for leg, positions in motor_positions.items():

        t0, t1, t2 = positions
        l1 = a1.thigh_len
        l2 = a1.calf_len
        W  = a1.body_width / 2
        a  = a1.a
        δ  = roll_angle

        # The position of the foot relative to the hip joint
        x = -l1 * np.sin(t1) - l2 * np.sin(t1 + t2)
        h =  l1 * np.cos(t1) + l2 * np.cos(t1 + t2)
        match leg:
            case "fl" | "hl":
                y = -a * np.cos(t0) - h * np.sin(t0)
                z =  a * np.sin(t0) - h * np.cos(t0)
            case "fr" | "hr":
                y =  a * np.cos(t0) - h * np.sin(t0)
                z = -a * np.sin(t0) - h * np.cos(t0)

        # perform coordinate transformation
        match leg:
            case "fr" | "hr":
                roll = np.array([[ 1,  0,          0,          0                 ],
                                 [ 0,  np.cos(δ), -np.sin(δ),  W * np.cos(δ) - W ],
                                 [ 0,  np.sin(δ),  np.cos(δ),  W * np.sin(δ)     ],
                                 [ 0,  0,          0,          1                 ]])
            case "fl" | "hl":
                roll = np.array([[ 1,  0,          0,          0                 ],
                                 [ 0,  np.cos(δ), -np.sin(δ), -W * np.cos(δ) + W ],
                                 [ 0,  np.sin(δ),  np.cos(δ), -W *  np.sin(δ)    ],
                                 [ 0,  0,          0,          1                 ]])
        x, y, z, _ = roll.dot(np.array([x, y, z, 1]))
        h = np.sqrt(z**2 + y**2 - a**2)

        # inverse kinematics solution
        c2 = (-l1**2 - l2**2 + x**2 + h**2) / (2 * l1 * l2)
        s2 = -np.sqrt(1 - c2**2)
        positions[2] = np.arctan2(s2, c2)
        positions[1] = np.arccos((l1**2 + x**2 + h**2 - l2**2) / (2 * l1 * np.sqrt(x**2 + h**2))) - np.arctan2(x, h)
        match leg:
            case "fl" | "hl":
                positions[0] = np.arctan2(h, a) - np.arctan2(np.abs(z), -y)
            case "fr" | "hr":
                positions[0] = np.arctan2(np.abs(z), y) - np.arctan2(h, a)

    # set motor positions
    for positions, indices in zip(motor_positions.itertuples(index=False), a1.motor_indices.itertuples(index=False)):
        bullet.setJointMotorControlArray(
            physicsClientId = physics_server_id,
            bodyUniqueId = a1.id,
            jointIndices = indices,
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = positions,)
