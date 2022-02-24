import pybullet as bullet
import numpy as np
from liba1 import A1

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

motor_positions = a1.motor_indices.applymap(lambda index: bullet.getJointState(a1.id, index, a1.in_physics_client)[0])

pitch_angle = 0.3

for leg, positions in motor_positions.items():
    t0, t1, t2 = positions
    l1 = a1.length_of_thigh
    l2 = a1.length_of_calf
    L = a1.L
    a = a1.a

    h = l1 * np.cos(t1) + l2 * np.cos(t1 + t2)
    x = l1 * np.sin(t1) + l2 * np.sin(t1 + t2)
    z = -h * np.cos(t0) + a * np.sin(t0)

    A = np.array([[ np.cos(pitch_angle), -np.sin(pitch_angle), L * np.cos(pitch_angle) - L ],
                  [ np.sin(pitch_angle),  np.cos(pitch_angle), L * np.sin(pitch_angle)     ],
                  [ 0,                    0,                   1                           ]])
    x, z, _ = A.dot(np.array([x, z, 1]))

    c2 = (-l1**2 - l2**2 + x**2 + z**2)/(2*l1*l2)
    s2 = -np.sqrt(1 - c2**2)
    positions[2] = np.arctan2(s2, c2)
    positions[1] = np.arctan2(z, x) - np.arctan2(l2*s2, l1 + l2*c2)
