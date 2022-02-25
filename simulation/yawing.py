import pybullet as bullet
import numpy as np
from liba1 import A1
from time import sleep

# Initialisation
physics_server_id = bullet.connect(bullet.GUI)
bullet.setRealTimeSimulation(enableRealTimeSimulation=True, physicsClientId=physics_server_id)
bullet.setGravity(0, 0, -30, physics_server_id)#The model is to light
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
    paramName = "Yaw Angle",
    rangeMin = -25,
    rangeMax = 25,
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
    bullet.stepSimulation()
    motor_positions = ref_motor_positions.copy()  # 以 ref pos 为参照计算俯仰角
    #The positive yaw angle means that when looking down at the robot, the robot rotates clockwise
    yaw_angle = bullet.readUserDebugParameter(debug_yaw_angle)
    yaw_angle_rad = np.radians(yaw_angle)#convert into radian

    for leg, positions in motor_positions.items():
    #use simple letters to make it readable
        t0, t1, t2 = positions
        l1 = a1.thigh_len
        l2 = a1.calf_len
        L = a1.body_len/2
        W = a1.body_width/2
        a = a1.a
        δ = yaw_angle_rad

        x = l1 * np.sin(t1) + l2 * np.sin(t1 + t2)
        h = l1 * np.cos(t1) + l2 * np.cos(t1 + t2)
        y = a * np.cos(t0) + h * np.sin(t0)
        z = -h * np.cos(t0) + a * np.sin(t0)

        #analysis the robot to get the transfor matrix
        match leg:
            case "fr":#the front right leg
                Yaw = np.array([[  np.cos(δ), np.sin(δ), 0, -L+L*np.cos(δ)+W*np.sin(δ)],
                                [ -np.sin(δ), np.cos(δ), 0, -W+W*np.cos(δ)-L*np.sin(δ)],
                                [  0,         0,         1,  0                        ],
                                [  0,         0,         0,  1                        ]])
            case "fl":#the front left leg
                Yaw = np.array([[  np.cos(δ), np.sin(δ), 0, -L+L*np.cos(δ)-W*np.sin(δ)],
                                [ -np.sin(δ), np.cos(δ), 0,  W-W*np.cos(δ)-L*np.sin(δ)],
                                [  0,         0,         1,  0                        ],
                                [  0,         0,         0,  1                        ]])
            case "hr":#the hind right leg
                Yaw = np.array([[  np.cos(δ), np.sin(δ), 0,  L-L*np.cos(δ)+W*np.sin(δ)],
                                [ -np.sin(δ), np.cos(δ), 0, -W+W*np.cos(δ)+L*np.sin(δ)],
                                [  0,         0,         1,  0                        ],
                                [  0,         0,         0,  1                        ]])
            case "hl":#the hind left leg
                Yaw = np.array([[  np.cos(δ), np.sin(δ), 0,  L-L*np.cos(δ)-W*np.sin(δ)],
                                [ -np.sin(δ), np.cos(δ), 0,  W-W*np.cos(δ)+L*np.sin(δ)],
                                [  0,         0,         1,  0                        ],
                                [  0,         0,         0,  1                        ]])
        x, y, z, _ = Yaw.dot(np.array([x, y, z, 1]))
        h = np.sqrt(z**2 + y**2 - a**2)

        #make inverse calculatino about the joint
        c2 = (-l1**2 - l2**2 + x**2 + h**2) / (2 * l1 * l2)
        s2 = -np.sqrt(1 - c2**2)  # sin is negative because the knee position is negative
        positions[2] = np.arctan2(s2, c2)
        positions[1] = np.arccos((l1**2 + x**2 + h**2 - l2**2) / (2 * l1 * np.sqrt(x**2 + h**2))) - np.arctan2(x, h)
        match leg:
            case "fr"|"hl":
                positions[0] =  np.pi - np.arccos(z/(z**2 + y**2)) - np.arccos(a/(z**2 + y**2))
            case "fl"|"hr":
                positions[0] = -np.pi + np.arccos(z/(z**2 + y**2)) + np.arccos(a/(z**2 + y**2))



    #set joint position
    for positions, indices in zip(motor_positions.itertuples(index=False), a1.motor_indices.itertuples(index=False)):
        bullet.setJointMotorControlArray(
            physicsClientId = physics_server_id,
            bodyUniqueId = a1.id,
            jointIndices = indices,
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = positions,)

    if bullet.readUserDebugParameter(reset) != previous_btn_value:
        #reset the base position
        bullet.resetBasePositionAndOrientation(a1.id, [0, 0, 0.43], [0, 0, 0, 1])
        previous_btn_value = bullet.readUserDebugParameter(reset)



