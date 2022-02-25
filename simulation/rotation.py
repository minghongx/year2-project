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

debug_yaw_angle = bullet.addUserDebugParameter(
    paramName = "Yaw_Angle",
    rangeMin = -22,
    rangeMax = 22,
    startValue = 0)

#The positive yaw angle means that when looking down at the robot, the robot rotates clockwise
yaw_angle = bullet.readUserDebugParameter(debug_yaw_angle)
yaw_angle_rad = np.radians(yaw_angle)#convert into radian


for leg, positions in motor_positions.items():
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
            Yaw = np.array([[  np.cos(yaw_angle),  np.sin(yaw_angle), 0, -L+L*np.cos(yaw_angle)+W*np.sin(yaw_angle)],
                            [ -np.sin(yaw_angle),  np.cos(yaw_angle), 0, -W+W*np.cos(yaw_angle)-L*np.sin(yaw_angle)],
                            [  0,                  0,                 1, 0                                         ],
                            [  0,                  0,                 0, 1                                         ]])
        case "fl":#the front left leg
            Yaw = np.array([[  np.cos(yaw_angle),  np.sin(yaw_angle), 0, -L+L*np.cos(yaw_angle)-W*np.sin(yaw_angle)],
                            [ -np.sin(yaw_angle),  np.cos(yaw_angle), 0,  W-W*np.cos(yaw_angle)-L*np.sin(yaw_angle)],
                            [  0,                  0,                 1, 0                                         ],
                            [  0,                  0,                 0, 1                                         ]])
        case "hr":#the hind right leg
            Yaw = np.array([[ np.cos(yaw_angle), -np.sin(yaw_angle), 0, L*np.cos(yaw_angle)-L-W*np.sin(yaw_angle)  ],
                            [ np.sin(yaw_angle),  np.cos(yaw_angle), 0, L * np.sin(yaw_angle)+W*np.cos(yaw_angle)-W],
                            [ 0,                  0,                 1, 0                                          ],
                            [ 0,                  0,                 0, 1                                          ]])
        case "hl":#the hind left leg
            Yaw = np.array([[ np.cos(yaw_angle), -np.sin(yaw_angle), 0, L*np.cos(yaw_angle)-L-W*np.sin(yaw_angle)  ],
                            [ np.sin(yaw_angle),  np.cos(yaw_angle), 0, L * np.sin(yaw_angle)+W*np.cos(yaw_angle)-W],
                            [ 0,                  0,                 1, 0                                          ],
                            [ 0,                  0,                 0, 1                                          ]])
    x, y, z, _ = Yaw.dot(np.array([x, y, z, 1]))
    h = np.sqrt(z**2 + y**2 - a**2)

    #make inverse calculatino about the joint
    c2 = (-l1**2 - l2**2 + x**2 + h**2) / (2 * l1 * l2)
    s2 = -np.sqrt(1 - c2**2)  # sin is negative because the knee position is negative
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

print(motor_positions)
sleep(20)
