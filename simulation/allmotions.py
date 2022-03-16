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

# Debug parameter silders to adjust the robot's motions
debug_pitch_angle = bullet.addUserDebugParameter(
    paramName = "pitch angle (rad)",
    rangeMin = -0.31,
    rangeMax =  0.31,
    startValue = 0,)
debug_roll_angle = bullet.addUserDebugParameter(
    paramName = "roll angle (rad)",
    rangeMin = -0.5,
    rangeMax =  0.5,
    startValue = 0,)
debug_yaw_angle = bullet.addUserDebugParameter(
    paramName = "yaw angle (rad)",
    rangeMin = -0.36,
    rangeMax =  0.36,
    startValue = 0)
debug_height = bullet.addUserDebugParameter(
    paramName = "height",
    rangeMin = 10,
    rangeMax = 370,
    startValue = 270,)

# Buttons to change the observing views
debug_initial_view = bullet.addUserDebugParameter(
    paramName="initial view",
    rangeMin = 1,
    rangeMax = 0,
    startValue = 0)
initial_view = bullet.readUserDebugParameter(debug_initial_view)
debug_front_view = bullet.addUserDebugParameter(
    paramName="front view",
    rangeMin = 1,
    rangeMax = 0,
    startValue = 0)
front_view = bullet.readUserDebugParameter(debug_front_view)
debug_top_view = bullet.addUserDebugParameter(
    paramName="top view",
    rangeMin = 1,
    rangeMax = 0,
    startValue = 0)
top_view = bullet.readUserDebugParameter(debug_top_view)

# Button to reset the robot position
reset = bullet.addUserDebugParameter(
    paramName="Reset Position",
    rangeMin = 1,
    rangeMax = 0,
    startValue = 0)
reset_value  = bullet.readUserDebugParameter(reset)

sleep(1)
ref_motor_positions = a1.motor_indices.applymap(lambda index: bullet.getJointState(a1.id, index, a1.in_physics_client)[0])
while True:
    motor_positions = ref_motor_positions.copy()
    yaw_angle = bullet.readUserDebugParameter(debug_yaw_angle)
    pitch_angle = bullet.readUserDebugParameter(debug_pitch_angle)
    roll_angle = bullet.readUserDebugParameter(debug_roll_angle)

    for leg, positions in motor_positions.items():
        θ0, θ1, θ2 = positions
        l1 = a1.thigh_len
        l2 = a1.calf_len
        L  = a1.body_len / 2
        W  = a1.body_width / 2
        o  = a1.hip_offset
        δ  = yaw_angle
        β  = pitch_angle
        λ  = roll_angle

        x = -l1 * np.sin(θ1) - l2 * np.sin(θ1 + θ2)
        h =  l1 * np.cos(θ1) + l2 * np.cos(θ1 + θ2)
        match leg:
            case "fl" | "hl":
                y = -o * np.cos(θ0) - h * np.sin(θ0)
                # z =  o * np.sin(θ0) - h * np.cos(θ0)
            case "fr" | "hr":
                y =  o * np.cos(θ0) - h * np.sin(θ0)
                # z = -o * np.sin(θ0) - h * np.cos(θ0)
        z = -bullet.readUserDebugParameter(debug_height)  # z is negative height

        match leg:
            case "fr":
                yaw = np.array([[ np.cos(δ), -np.sin(δ),   0,   L * np.cos(δ) - W * np.sin(δ) - L ],
                                [ np.sin(δ),  np.cos(δ),   0,   L * np.sin(δ) + W * np.cos(δ) - W ],
                                [ 0,          0,           1,   0                                 ],
                                [ 0,          0,           0,   1                                 ]])
            case "fl":
                yaw = np.array([[ np.cos(δ), -np.sin(δ),   0,   L * np.cos(δ) + W * np.sin(δ) - L ],
                                [ np.sin(δ),  np.cos(δ),   0,   L * np.sin(δ) - W * np.cos(δ) + W ],
                                [ 0,          0,           1,   0                                 ],
                                [ 0,          0,           0,   1                                 ]])
            case "hr":
                yaw = np.array([[ np.cos(δ), -np.sin(δ),   0,  -L * np.cos(δ) - W * np.sin(δ) + L ],
                                [ np.sin(δ),  np.cos(δ),   0,  -L * np.sin(δ) + W * np.cos(δ) - W ],
                                [ 0,          0,           1,   0                                 ],
                                [ 0,          0,           0,   1                                 ]])
            case "hl":
                yaw = np.array([[ np.cos(δ), -np.sin(δ),   0,  -L * np.cos(δ) + W * np.sin(δ) + L ],
                                [ np.sin(δ),  np.cos(δ),   0,  -L * np.sin(δ) - W * np.cos(δ) + W ],
                                [ 0,          0,           1,   0                                 ],
                                [ 0,          0,           0,   1                                 ]])
        x, y, z, _ = yaw.dot(np.array([x, y, z, 1]))

        match leg:
            case "fr" | "fl":
                pitch = np.array([[ np.cos(β), -np.sin(β),  L * np.cos(β) - L ],
                                  [ np.sin(β),  np.cos(β),  L * np.sin(β)     ],
                                  [ 0,          0,          1                 ]])
            case "hr" | "hl":
                pitch = np.array([[ np.cos(β), -np.sin(β), -L * np.cos(β) + L ],
                                  [ np.sin(β),  np.cos(β), -L * np.sin(β)     ],
                                  [ 0,          0,          1                 ]])
        x, z, _ = pitch.dot(np.array([x, z, 1]))

        match leg:
            case "fr" | "hr":
                roll = np.array([[ 1,  0,          0,          0                 ],
                                 [ 0,  np.cos(λ), -np.sin(λ),  W * np.cos(λ) - W ],
                                 [ 0,  np.sin(λ),  np.cos(λ),  W * np.sin(λ)     ],
                                 [ 0,  0,          0,          1                 ]])
            case "fl" | "hl":
                roll = np.array([[ 1,  0,          0,          0                 ],
                                 [ 0,  np.cos(λ), -np.sin(λ), -W * np.cos(λ) + W ],
                                 [ 0,  np.sin(λ),  np.cos(λ), -W * np.sin(λ)     ],
                                 [ 0,  0,          0,          1                 ]])
        x, y, z, _ = roll.dot(np.array([x, y, z, 1]))

        h = np.sqrt(z**2 + y**2 - o**2)

        c2 = (-l1**2 - l2**2 + x**2 + h**2) / (2 * l1 * l2)
        s2 = -np.sqrt(1 - c2**2)
        positions[2] = np.arctan2(s2, c2)
        positions[1] = np.arccos((l1**2 + x**2 + h**2 - l2**2) / (2 * l1 * np.sqrt(x**2 + h**2))) - np.arctan2(x, h)
        match leg:
            case "fl" | "hl":
                positions[0] = np.arctan2(h, o) - np.arctan2(np.abs(z), -y)
            case "fr" | "hr":
                positions[0] = np.arctan2(np.abs(z), y) - np.arctan2(h, o)

    for positions, indices in zip(motor_positions.itertuples(index=False), a1.motor_indices.itertuples(index=False)):
        bullet.setJointMotorControlArray(
            physicsClientId = physics_server_id,
            bodyUniqueId = a1.id,
            jointIndices = indices,
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = positions)

    if bullet.readUserDebugParameter(reset) != reset_value:
        # reset position
        bullet.resetBasePositionAndOrientation(a1.id, [0, 0, 0.43], [0, 0, 0, 1])
        reset_value = bullet.readUserDebugParameter(reset)

    if bullet.readUserDebugParameter(debug_initial_view) != initial_view:
        # set the camera to be initial view
        bullet.resetDebugVisualizerCamera(
            physicsClientId = physics_server_id,
            cameraTargetPosition = [0, 0, 0.4],
            cameraDistance = 1.5,
            cameraYaw = 40,
            cameraPitch = -15)
        initial_view = bullet.readUserDebugParameter(debug_initial_view)

    if bullet.readUserDebugParameter(debug_front_view) != front_view:
        # set the camera to be front view
        bullet.resetDebugVisualizerCamera(
            physicsClientId = physics_server_id,
            cameraTargetPosition = [0, 0, 0.4],
            cameraDistance = 1.5,
            cameraYaw = 90,
            cameraPitch = 0)
        front_view = bullet.readUserDebugParameter(debug_front_view)

    if bullet.readUserDebugParameter(debug_top_view) != top_view:
        # set the camera to be overlook view
        bullet.resetDebugVisualizerCamera(
            physicsClientId = physics_server_id,
            cameraTargetPosition = [0, 0, 0.4],
            cameraDistance = 1.5,
            cameraYaw = 90,
            cameraPitch = -89)
        top_view = bullet.readUserDebugParameter(debug_top_view)
