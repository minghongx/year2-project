import pybullet as bullet
import pandas as pd
import numpy as np
from typing import Iterable

class A1:

    def __init__(
    self,
    physics_client_id: int,
    base_position: Iterable[float] = [0, 0, 0.43],
    base_orientation: Iterable[float] = [0, 0, 0, 1]):
        self.in_physics_client = physics_client_id
        # Load the model in the given PyBullet physics client.
        import pybullet_data; bullet.setAdditionalSearchPath(pybullet_data.getDataPath())  # The A1 model is in the pybullet_data
        self.id = bullet.loadURDF(
            "a1/a1.urdf",
            base_position, base_orientation,
            physicsClientId = self.in_physics_client,
            flags = bullet.URDF_USE_SELF_COLLISION,
            useFixedBase = False)
        # Motors
        self.motor_indices = pd.DataFrame(dtype=np.int8,
            columns =        ["fr", "fl", "hr", "hl"],
            data = np.array([[  1 ,   6 ,  11 ,  16 ],
                             [  3 ,   8 ,  13 ,  18 ],
                             [  4 ,   9 ,  14 ,  19 ],]),
            index = ["hip abd/add", "hip fle/ext", "knee"])
        # A matrix contains motor information. The info is structured as
        # Tuple(jointIndex, jointName, jointType, qIndex, uIndex, flags, jointDamping
        # , jointFriction, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity
        # , linkName, jointAxis, parentFramePos, parentFrameOrn, parentIndex)
        # See PyBullet document for getJointInfo().
        self.motor_info = self.motor_indices.applymap(lambda index: bullet.getJointInfo(self.id, index))
        # Obtained by measuring the STL file of the thigh and calf. Both are 0.2mm.
        self.thigh_len = 200
        self.calf_len  = 200
        # FIXME 髋关节距离主体的偏移
        self.a = 80
        # FIXME 身长
        self.body_len = 360
        # FIXME 身宽
        self.body_width = 200
        # Set initial postion of motors.
        bullet.setJointMotorControlArray(
            physicsClientId = self.in_physics_client,
            bodyUniqueId = self.id,
            jointIndices = self.motor_indices.loc["hip abd/add", :],
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, 0),)
        bullet.setJointMotorControlArray(
            physicsClientId = self.in_physics_client,
            bodyUniqueId = self.id,
            jointIndices = self.motor_indices.loc["hip fle/ext", :],
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, 0.7),)
        bullet.setJointMotorControlArray(
            physicsClientId = self.in_physics_client,
            bodyUniqueId = self.id,
            jointIndices = self.motor_indices.loc["knee", :],
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, -0.7*2),)  # ensures that the toes are beneath the hips
