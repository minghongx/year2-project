import pybullet as bullet
import numpy as np
from liba1 import A1
from time import sleep

# Initialisation
physics_server_id = bullet.connect(bullet.GUI)
bullet.setRealTimeSimulation(enableRealTimeSimulation=True, physicsClientId=physics_server_id)
bullet.setGravity(0, 0, -30, physics_server_id)
import pybullet_data; bullet.setAdditionalSearchPath(pybullet_data.getDataPath())
bullet.loadURDF("plane.urdf", physicsClientId=physics_server_id)
a1 = A1(physics_server_id)
bullet.resetDebugVisualizerCamera(
    physicsClientId = physics_server_id,
    cameraTargetPosition = [0, 0, 0.4],
    cameraDistance = 1.5,
    cameraYaw = 40,
    cameraPitch = -15)

debug_height = bullet.addUserDebugParameter(
    paramName = "height",
    rangeMin = 10,
    rangeMax = 370,
    startValue = 200,)

sleep(1)
ref_motor_positions = a1.motor_indices.applymap(lambda index: bullet.getJointState(a1.id, index, a1.in_physics_client)[0])
while True:
    motor_positions = ref_motor_positions.copy()

    for leg, positions in motor_positions.items():

        t0, t1, t2 = positions
        l1 = a1.thigh_len
        l2 = a1.calf_len
        W  = a1.body_width / 2
        o  = a1.hip_offset

        x = -l1 * np.sin(t1) - l2 * np.sin(t1 + t2)
        h =  l1 * np.cos(t1) + l2 * np.cos(t1 + t2)
        match leg:
            case "fl" | "hl":
                y = -o * np.cos(t0) - h * np.sin(t0)
            case "fr" | "hr":
                y =  o * np.cos(t0) - h * np.sin(t0)

        z = -bullet.readUserDebugParameter(debug_height)  # z is negative height
        h = np.sqrt(z**2 + y**2 - o**2)

        # inverse kinematics solution
        c2 = (-l1**2 - l2**2 + x**2 + h**2) / (2 * l1 * l2)
        s2 = -np.sqrt(1 - c2**2)
        positions[2] = np.arctan2(s2, c2)
        positions[1] = np.arccos((l1**2 + x**2 + h**2 - l2**2) / (2 * l1 * np.sqrt(x**2 + h**2))) - np.arctan2(x, h)
        match leg:
            case "fl" | "hl":
                positions[0] = np.arctan2(h, o) - np.arctan2(np.abs(z), -y)
            case "fr" | "hr":
                positions[0] = np.arctan2(np.abs(z), y) - np.arctan2(h, o)

    # set motor positions
    for positions, indices in zip(motor_positions.itertuples(index=False), a1.motor_indices.itertuples(index=False)):
        bullet.setJointMotorControlArray(
            physicsClientId = physics_server_id,
            bodyUniqueId = a1.id,
            jointIndices = indices,
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = positions,)
