import pybullet as bullet
import pybullet_data
from liba1 import a1_motor_indices
from dataclasses import asdict

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

motors = {motor_name : bullet.getJointInfo(a1_id, motor_index) for motor_name, motor_index in asdict(a1_motor_indices()).items()}

motor_pos_debug_params = list()
for motor_name, motor_info in motors.items():
    match motor_name:
        case name if name.endswith("hip_abduction_or_adduction"):
            motor_pos_debug_params.append((
                motor_info[0],  # motor index
                bullet.addUserDebugParameter(
                    paramName = motor_name,
                    rangeMin = motor_info[8],
                    rangeMax = motor_info[9],
                    startValue = 0,)))
        case name if name.endswith("hip_flexion_or_extension"):
            motor_pos_debug_params.append((
                motor_info[0],  # motor index
                bullet.addUserDebugParameter(
                    paramName = motor_name,
                    rangeMin = motor_info[8],
                    rangeMax = motor_info[9],
                    startValue = 0.7,)))
        case name if name.endswith("knee"):
            motor_pos_debug_params.append((
                motor_info[0],  # motor index
                bullet.addUserDebugParameter(
                    paramName = motor_name,
                    rangeMin = motor_info[8],
                    rangeMax = motor_info[9],
                    startValue = -0.7*2,)))  # ensures that the toes are beneath the hips

while True:
    bullet.setJointMotorControlArray(
        bodyUniqueId = a1_id,
        jointIndices = [index for index, _ in motor_pos_debug_params],
        controlMode = bullet.POSITION_CONTROL,
        targetPositions = [bullet.readUserDebugParameter(param) for _, param in motor_pos_debug_params],)
