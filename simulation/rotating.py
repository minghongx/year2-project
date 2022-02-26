from cgitb import reset
import pybullet as bullet
import numpy as np
from liba1 import A1
from time import sleep

# Initialisation
physics_server_id = bullet.connect(bullet.GUI)
bullet.setRealTimeSimulation(enableRealTimeSimulation=True, physicsClientId=physics_server_id)
bullet.setGravity(0, 0, -30, physics_server_id)  # FIXME 模型质量太轻
import pybullet_data; bullet.setAdditionalSearchPath(pybullet_data.getDataPath())
bullet.loadURDF("plane.urdf")
a1 = A1(physics_server_id)
bullet.resetDebugVisualizerCamera(
    physicsClientId = physics_server_id,
    cameraTargetPosition = [0, 0, 0.3],
    cameraDistance = 1.5,
    cameraYaw =90,
    cameraPitch = 0)

debug_rotation_angle = bullet.addUserDebugParameter(
    paramName = "Yaw Angle",
    rangeMin = -0.7,
    rangeMax = 0.7,
    startValue = 0)

#Use the reset buttom to reset the robot
reset = bullet.addUserDebugParameter(
    paramName="Reset Position",
    rangeMin=1,
    rangeMax=0,
    startValue=0
)
previous_btn_value = bullet.readUserDebugParameter(reset)



sleep(1)
ref_motor_positions = a1.motor_indices.applymap(lambda index: bullet.getJointState(a1.id, index, a1.in_physics_client)[0])
while True:
    motor_positions = ref_motor_positions.copy()  # 以 ref pos 为参照计算俯仰角
    #The positive yaw angle means that when looking down at the robot, the robot rotates clockwise
    rotation_angle = bullet.readUserDebugParameter(debug_rotation_angle)

    for leg, positions in motor_positions.items():
    #use simple letters to make it readable
        t0, t1, t2 = positions
        l1 = a1.thigh_len
        l2 = a1.calf_len
        L = a1.body_len/2
        W = a1.body_width/4
        a = a1.a
        δ = rotation_angle

        x = l1 * np.sin(t1) + l2 * np.sin(t1 + t2)
        h0 = l1 * np.cos(t1) + l2 * np.cos(t1 + t2)
        y = a * np.cos(t0) + h0 * np.sin(t0)
        z = h0 * np.cos(t0) + a * np.sin(t0)

        #analysis the robot to get the transfor matrix
        match leg:
            case "fl"|"hl":#the front leg
                yaw = np.array([[ 1, 0,          0,          0              ],
                                [ 0, np.cos(δ), -np.sin(δ), -W*np.sin(δ)    ],
                                [ 0, np.sin(δ),  np.cos(δ), -W + W*np.cos(δ)],
                                [ 0,         0,  0,          1              ]])
            case "fr"|"hr":#the front left leg
                yaw = np.array([[ 1, 0,          0,           0              ],
                                [ 0, np.cos(δ), -np.sin(δ),   W*np.sin(δ)    ],
                                [ 0, np.sin(δ),  np.cos(δ),   W - W*np.cos(δ)],
                                [ 0, 0,          0,           1              ]])
        x, y, z, _ = yaw.dot(np.array([x, y, z, 1]))


        match leg:
            case "fr"|"hr":
                h1 = h0 + W * np.sin(δ)
                positions[0] = -np.arctan2(h1, a) + np.arctan2(np.abs(z),y)
                positions[1] = np.arccos(h1/(2*l1))*1.05
                positions[2] = (-np.arccos(h1/(2*l1))-np.arccos(h1/(2*l2)))*1.05
            case "fl"|"hl":
                h2 = h0 - W * np.sin(δ)
                positions[0] = -np.arctan2(h2, a) + np.arctan2(np.abs(z),y)
                positions[1] = np.arccos(h2/(2*l1))*1.05
                positions[2] = (-np.arccos(h2/(2*l1))-np.arccos(h2/(2*l2)))*1.05

        #match leg:
        #    case "fl"|"fr":
        #        positions[0] = np.arctan2(h, a) - np.arctan2(np.abs(z),y)
        #    case "hr"|"hl":
        #        positions[0] = np.arctan2(h, a) - np.arctan2(np.abs(z),y)





    #set joint position
    for positions, indices in zip(motor_positions.itertuples(index=False), a1.motor_indices.itertuples(index=False)):
        bullet.setJointMotorControlArray(
            physicsClientId = physics_server_id,
            bodyUniqueId = a1.id,
            jointIndices = indices,
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = positions)

    if bullet.readUserDebugParameter(reset) != previous_btn_value:
        #reset the base position
        bullet.resetBasePositionAndOrientation(a1.id, [0, 0, 0.43], [0, 0, 0, 1])
        previous_btn_value = bullet.readUserDebugParameter(reset)



