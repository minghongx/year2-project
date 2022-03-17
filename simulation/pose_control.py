import pybullet as bullet
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
debug_roll_angle = bullet.addUserDebugParameter(
    paramName = "roll angle (rad)",
    rangeMin = -0.5,
    rangeMax =  0.5,
    startValue = 0,)
debug_pitch_angle = bullet.addUserDebugParameter(
    paramName = "pitch angle (rad)",
    rangeMin = -0.31,
    rangeMax =  0.31,
    startValue = 0,)
debug_yaw_angle = bullet.addUserDebugParameter(
    paramName = "yaw angle (rad)",
    rangeMin = -0.36,
    rangeMax =  0.36,
    startValue = 0,)
debug_height = bullet.addUserDebugParameter(
    paramName = "Delta z",
    rangeMin = -200,
    rangeMax = 60,
    startValue = 0,)

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

sleep(1)  # Ugly so FIXME
# The initialisation is asynchronous. Wait one second to ensure that the motors reach their initial position before reading the position values.
ini_pose = a1.current_pose()
while True:
    a1.pose_control(
        roll_angle  = bullet.readUserDebugParameter(debug_roll_angle),
        pitch_angle = bullet.readUserDebugParameter(debug_pitch_angle),
        yaw_angle   = bullet.readUserDebugParameter(debug_yaw_angle),
        Δz          = bullet.readUserDebugParameter(debug_height),
        reference_pose=ini_pose)

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

    if bullet.readUserDebugParameter(reset) != reset_value:
        # reset position
        bullet.resetBasePositionAndOrientation(a1.id, [0, 0, 0.43], [0, 0, 0, 1])
        reset_value = bullet.readUserDebugParameter(reset)
