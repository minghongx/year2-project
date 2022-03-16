import pybullet as bullet
import numpy as np
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

debug_height = bullet.addUserDebugParameter(
    paramName = "height",
    rangeMin = 10,
    rangeMax = 370,
    startValue = 306,)

sleep(1)  # Ugly so FIXME
# The initialisation is asynchronous. Wait one second to ensure that the motors reach their initial position before reading the position values.
ref_pos = a1.current_motor_angular_positions()
while True:
    a1.squatting(bullet.readUserDebugParameter(debug_height), ref_pos)
