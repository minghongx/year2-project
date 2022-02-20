import pybullet as bullet
import pybullet_data
from liba1 import a1_motor_indices
from dataclasses import asdict
import numpy as np

# Initialisation
physics_server = bullet.connect(bullet.GUI)
bullet.setRealTimeSimulation(enableRealTimeSimulation=1)
bullet.setGravity(0, 0, -9.8)
bullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # a1 model is in the pybullet_data
bullet.loadURDF("plane.urdf")
a1_id = bullet.loadURDF(
                "a1/a1.urdf",
                [0,0,0.48], [0,0,0,1],
                flags = bullet.URDF_USE_SELF_COLLISION,
                useFixedBase = False)
bullet.resetDebugVisualizerCamera(
    cameraTargetPosition = [0,0,0.4],
    cameraDistance = 1.5,
    cameraYaw = 40,
    cameraPitch = -15)
for motor_name, motor_index in asdict(a1_motor_indices()).items():
    match motor_name:
        case name if name.endswith("hip_abduction_or_adduction"):
            bullet.setJointMotorControl2(
                bodyUniqueId = a1_id,
                jointIndex = motor_index,
                controlMode = bullet.POSITION_CONTROL,
                targetPosition = 0,)
        case name if name.endswith("hip_flexion_or_extension"):
            bullet.setJointMotorControl2(
                bodyUniqueId = a1_id,
                jointIndex = motor_index,
                controlMode = bullet.POSITION_CONTROL,
                targetPosition = 0.7,)
        case name if name.endswith("knee"):
            bullet.setJointMotorControl2(
                bodyUniqueId = a1_id,
                jointIndex = motor_index,
                controlMode = bullet.POSITION_CONTROL,
                targetPosition = -0.7*2,)  # ensures that the toes are beneath the hips

length_of_thigh = 200
length_of_calf = 200
# Obtained by measuring the STL file of the thigh and calf.

debug_distance_from_hip_to_toe = bullet.addUserDebugParameter(
    paramName = "distance from hip to toe",
    rangeMin = 1,
    # The calfs can be extended up to its motor position at -0.916, which is recorded in the URDF file.
    rangeMax = np.sqrt(length_of_thigh**2 + length_of_calf**2 - np.cos(np.pi - 0.916) * 2 * length_of_thigh * length_of_calf),
    startValue = 300,)

while True:
    distance_from_hip_to_toe = bullet.readUserDebugParameter(debug_distance_from_hip_to_toe)

    alpha = np.arccos(np.true_divide(
                length_of_thigh**2 + distance_from_hip_to_toe**2 - length_of_calf**2,
                2 * length_of_calf * distance_from_hip_to_toe))
    beta = np.pi - np.arccos(np.true_divide(
                        length_of_thigh**2 + length_of_calf**2 - distance_from_hip_to_toe**2,
                        2 * length_of_thigh * length_of_calf))

    bullet.setJointMotorControlArray(
            bodyUniqueId = a1_id,
            jointIndices = a1_motor_indices().hip_flexion_or_extension(),
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, alpha),)
    bullet.setJointMotorControlArray(
            bodyUniqueId = a1_id,
            jointIndices = a1_motor_indices().knee(),
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, -beta),)
