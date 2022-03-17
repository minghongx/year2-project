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

debug_yaw_angle = bullet.addUserDebugParameter(
    paramName = "yaw angle (rad)",
    rangeMin = -0.36,
    rangeMax =  0.36,
    startValue = 0)

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
debug_bottom_view = bullet.addUserDebugParameter(
    paramName="bottom view",
    rangeMin = 1,
    rangeMax = 0,
    startValue = 0)
bottom_view = bullet.readUserDebugParameter(debug_bottom_view)

reset = bullet.addUserDebugParameter(
    paramName="Reset Position",
    rangeMin = 1,
    rangeMax = 0,
    startValue = 0)
previous_btn_value = bullet.readUserDebugParameter(reset)

sleep(1)  # Ugly so FIXME
# The initialisation is asynchronous. Wait one second to ensure that the motors reach their initial position before reading the position values.
ini_pos = a1.current_motor_angular_positions()
while True:
    a1.adjust_posture(yaw_angle=bullet.readUserDebugParameter(debug_yaw_angle), ref_motor_angular_positions=ini_pos)

    if bullet.readUserDebugParameter(reset) != previous_btn_value:
        # reset position
        bullet.resetBasePositionAndOrientation(a1.id, [0, 0, 0.43], [0, 0, 0, 1])
        previous_btn_value = bullet.readUserDebugParameter(reset)

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
    if bullet.readUserDebugParameter(debug_bottom_view) != bottom_view:
        # set the camera to be overlook view
        bullet.resetDebugVisualizerCamera(
            physicsClientId = physics_server_id,
            cameraTargetPosition = [0, 0, 0.4],
            cameraDistance = 1.5,
            cameraYaw = 90,
            cameraPitch = 89)
        bottom_view = bullet.readUserDebugParameter(debug_bottom_view)
