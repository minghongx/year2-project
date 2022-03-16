import pybullet as bullet
import numpy as np
from liba1 import A1
from time import sleep

# Initialisation
physics_server_id = bullet.connect(bullet.GUI)
bullet.setRealTimeSimulation(enableRealTimeSimulation=True, physicsClientId=physics_server_id)
bullet.setGravity(0, 0, -30, physics_server_id)  # FIXME: The model is too light
import pybullet_data; bullet.setAdditionalSearchPath(pybullet_data.getDataPath())
bullet.loadURDF("plane.urdf", physicsClientId=physics_server_id)
a1 = A1(physics_server_id)
bullet.resetDebugVisualizerCamera(
    physicsClientId = physics_server_id,
    cameraTargetPosition = [0, 0, 0.4],
    cameraDistance = 1.5,
    cameraYaw = 40,
    cameraPitch = -15)

debug_pitch_angle = bullet.addUserDebugParameter(
    paramName = "pitch angle (rad)",
    rangeMin = -0.31,
    rangeMax =  0.31,
    startValue = 0,)

sleep(1)  # Ugly so FIXME
# The initialisation is asynchronous. Wait one second to ensure that the motors reach their initial position before reading the position values.
ref_pos = a1.current_motor_angular_positions()
while True:
    a1.pitching(bullet.readUserDebugParameter(debug_pitch_angle), ref_pos)
