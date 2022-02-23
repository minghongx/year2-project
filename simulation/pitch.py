from distutils.log import debug
import pybullet as p
import pybullet_data
from liba1 import a1_motor_indices
from dataclasses import asdict
import numpy as np

# Initialisation
physics_server = p.connect(p.GUI)
p.setRealTimeSimulation(enableRealTimeSimulation=1)
p.setGravity(0, 0, -9.8)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # a1 model is in the pybullet_data
p.loadURDF("plane.urdf")

a1_id = p.loadURDF(
                "a1/a1.urdf",
                [0,0,0.48], [0,0,0,1],
                flags = p.URDF_USE_SELF_COLLISION,
                useFixedBase = False)

#set the Camera
p.resetDebugVisualizerCamera(
    cameraTargetPosition = [0,0,0.4],
    cameraDistance = 1.5,
    cameraYaw = 40,
    cameraPitch = -15)
#set initial condition
for motor_name, motor_index in asdict(a1_motor_indices()).items():
    match motor_name:
        case name if name.endswith("hip_abduction_or_adduction"):
            p.setJointMotorControl2(
                bodyUniqueId = a1_id,
                jointIndex = motor_index,
                controlMode = p.POSITION_CONTROL,
                targetPosition = 0,)
        case name if name.endswith("hip_flexion_or_extension"):
            p.setJointMotorControl2(
                bodyUniqueId = a1_id,
                jointIndex = motor_index,
                controlMode = p.POSITION_CONTROL,
                targetPosition = 0.784,)#0.784 in rad is equal to 45 in degree
        case name if name.endswith("knee"):
            p.setJointMotorControl2(
                bodyUniqueId = a1_id,
                jointIndex = motor_index,
                controlMode = p.POSITION_CONTROL,
                targetPosition = -0.784*2,)  # ensures that the toes are beneath the hips

# Obtained by measuring the STL file of the thigh and calf.
length_of_thigh = 200
length_of_calf = 200
length_of_body = 390
#Other useful initial conditions
half_body = 195
#length**2 = half_body**2 + length_of_thigh**2 + length_of_calf**2
length = np.sqrt(half_body**2 + length_of_thigh**2 + length_of_calf**2)
#The angle AOB
angle_beta = np.arccos(half_body/length)

# FIXME
# The algorithm below assumes that the height is the distance from hip to toe.
# The algorithm fails when the motor postion for hip abduction or adduction is not equal to 0.

#The parameter for pitch_angle
debug_pitch_angle = p.addUserDebugParameter(
    paramName = "Pitch_Angle",
    #By calculation, the maximum pitch angle is 0.3989287 in rad or 22.804 in degree
    rangeMin = -22,
    rangeMax = 22,
    startValue = 0,)


while True:
    pitch_angle = p.readUserDebugParameter(debug_pitch_angle)
    pitch_angle_rad = np.radians(pitch_angle)#convert into radian

    #The front legs
    #the angle for the front legs
    alpha = np.sqrt(length**2 + length_of_body**2-(2*length*length_of_body*(np.cos(angle_beta+pitch_angle_rad))))
    #The hip angle of front legs
    front_hip = np.arccos(alpha/(2*length_of_thigh))
    #The hip angle of front legs
    front_knee = front_hip*(-2)

    #The hind leg
    #the angle for the hind legs
    beta = np.sqrt(length**2 + length_of_body**2-(2*length*length_of_body*(np.cos(angle_beta-pitch_angle_rad))))
    #The hip angle of hind legs
    hind_hip = np.arccos(beta/(2*length_of_thigh))
    #The hip angle of hind legs
    hind_knee = hind_hip*(-2)

    #Joint controlling
    #control the front leg
    #control the front hip joint
    p.setJointMotorControlArray(
            bodyUniqueId = a1_id,
            jointIndices = np.array([3,8]),
            controlMode = p.POSITION_CONTROL,
            targetPositions = np.full(2, front_hip))
    #control the front knee joint
    p.setJointMotorControlArray(
            bodyUniqueId = a1_id,
            jointIndices = np.array([4,9]),
            controlMode = p.POSITION_CONTROL,
            targetPositions = np.full(2, front_knee))
    #control the hind leg
    #control the hind hip joint
    p.setJointMotorControlArray(
            bodyUniqueId = a1_id,
            jointIndices = np.array([13,18]),
            controlMode = p.POSITION_CONTROL,
            targetPositions = np.full(2, hind_hip))
    #control the hind knee joint
    p.setJointMotorControlArray(
            bodyUniqueId = a1_id,
            jointIndices = np.array([14,19]),
            controlMode = p.POSITION_CONTROL,
            targetPositions = np.full(2, hind_knee))
