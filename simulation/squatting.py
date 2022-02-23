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

debug_distance_from_hip_to_toe = bullet.addUserDebugParameter(
    paramName = "distance from hip to toe",
    rangeMin = 1,
    # The calfs can be extended up to its motor position at -0.916, which is recorded in the URDF file.
    rangeMax = np.sqrt(a1.length_of_thigh**2 + a1.length_of_calf**2 - np.cos(np.pi - 0.916) * 2 * a1.length_of_thigh * a1.length_of_calf),
    startValue = 300,)

while True:
    distance_from_hip_to_toe = bullet.readUserDebugParameter(debug_distance_from_hip_to_toe)

    alpha = np.arccos(np.true_divide(
                a1.length_of_thigh**2 + distance_from_hip_to_toe**2 - a1.length_of_calf**2,
                2 * a1.length_of_calf * distance_from_hip_to_toe))
    beta = np.pi - np.arccos(np.true_divide(
                        a1.length_of_thigh**2 + a1.length_of_calf**2 - distance_from_hip_to_toe**2,
                        2 * a1.length_of_thigh * a1.length_of_calf))

    bullet.setJointMotorControlArray(
            physicsClientId=physics_server_id,
            bodyUniqueId = a1.id,
            jointIndices = a1.motor_indices.loc["hip flexion/extension", :],
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, alpha),)
    bullet.setJointMotorControlArray(
            physicsClientId=physics_server_id,
            bodyUniqueId = a1.id,
            jointIndices = a1.motor_indices.loc["knee", :],
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, -beta),)
