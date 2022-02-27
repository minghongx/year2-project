from cgitb import reset
import pybullet as bullet
import numpy as np
from liba1 import A1
from time import sleep

# Initialisation
physics_server_id = bullet.connect(bullet.GUI)
bullet.setRealTimeSimulation(enableRealTimeSimulation=True, physicsClientId=physics_server_id)
bullet.setGravity(0, 0, -30, physics_server_id)  # FIXME: The model is too light
import pybullet_data; bullet.setAdditionalSearchPath(pybullet_data.getDataPath())
bullet.loadURDF("plane.urdf", physicsClientId=physics_server_id)
a1 = A1(physics_server_id)
bullet.resetDebugVisualizerCamera(
    physicsClientId = physics_server_id,
    cameraTargetPosition = [0, 0, 0.4],
    cameraDistance = 1.5,
    cameraYaw = 40,
    cameraPitch = -15)

debug_pitch_angle = bullet.addUserDebugParameter(
    paramName = "pitch angle (rad)",
    rangeMin = -0.31,
    rangeMax =  0.31,
    startValue = 0,)

sleep(1)  # The initialisation is asynchronous. Before reading their position, wait one second to ensure that the motors reach their initial position
ref_motor_positions = a1.motor_indices.applymap(lambda index: bullet.getJointState(a1.id, index, a1.in_physics_client)[0])
while True:
    motor_positions = ref_motor_positions.copy()  # The following calculation is with reference to the initial position of the motor, not the current position
    pitch_angle = bullet.readUserDebugParameter(debug_pitch_angle)

    for leg, positions in motor_positions.items():
        # Abbreviating to increase readability
        θ0, θ1, θ2 = positions
        ℓ1 = a1.thigh_len
        ℓ2 = a1.calf_len
        L  = a1.body_len / 2
        o  = a1.hip_offset
        δ  = pitch_angle

        # θ0, θ1, θ2, ℓ1, ℓ2 ⇒ x, h, y, z
        x = -ℓ1 * np.sin(θ1) - ℓ2 * np.sin(θ1 + θ2)
        h =  ℓ1 * np.cos(θ1) + ℓ2 * np.cos(θ1 + θ2)
        match leg:
            case "fr" | "hr":
                y = -o * np.cos(θ0) - h * np.sin(θ0)
                z =  o * np.sin(θ0) - h * np.cos(θ0)
            case "fl" | "hl":
                y =  o * np.cos(θ0) - h * np.sin(θ0)
                z = -o * np.sin(θ0) - h * np.cos(θ0)

        # New x, z, h after pitching
        match leg:
            case "fr" | "fl":
                pitch = np.array([[ np.cos(δ), -np.sin(δ),  L * np.cos(δ) - L ],
                                  [ np.sin(δ),  np.cos(δ),  L * np.sin(δ)     ],
                                  [ 0,          0,          1                 ]])
            case "hr" | "hl":
                pitch = np.array([[ np.cos(δ), -np.sin(δ), -L * np.cos(δ) + L ],
                                  [ np.sin(δ),  np.cos(δ), -L * np.sin(δ)     ],
                                  [ 0,          0,          1                 ]])
        x, z, _ = pitch.dot(np.array([x, z, 1]))
        h = np.sqrt(z**2 + y**2 - o**2)

        # x, h, y, z ⇒ θ2, θ1
        cosθ2 = (-ℓ1**2 - ℓ2**2 + x**2 + h**2) / (2 * ℓ1 * ℓ2)
        sinθ2 = -np.sqrt(1 - cosθ2**2)  # takes negative suqare root, because the knee's position is specified as negative
        positions[2] = np.arctan2(sinθ2, cosθ2)
        positions[1] = np.arccos((ℓ1**2 + x**2 + h**2 - ℓ2**2) / (2 * ℓ1 * np.sqrt(x**2 + h**2))) - np.arctan2(x, h)

    # Set motor positions
    for positions, indices in zip(motor_positions.itertuples(index=False), a1.motor_indices.itertuples(index=False)):
        bullet.setJointMotorControlArray(
            physicsClientId = physics_server_id,
            bodyUniqueId = a1.id,
            jointIndices = indices,
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = positions,)
