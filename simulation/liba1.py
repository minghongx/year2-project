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
        # Tuple(jointIndex, jointName, jointType, qIndex, uIndex, flags, jointDamping,
        # jointFriction, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity,
        # linkName, jointAxis, parentFramePos, parentFrameOrn, parentIndex)
        # See PyBullet document for getJointInfo().
        self.motor_info = self.motor_indices.applymap(lambda index: bullet.getJointInfo(self.id, index))
        # Obtained by measuring the STL file of the thigh and calf. Both are 0.2mm.
        self.thigh_len = 200
        self.calf_len  = 200
        self.hip_offset = 80   # FIXME: Obtain more accurate value
        self.body_len = 360    # FIXME: Obtain more accurate value
        self.body_width = 200  # FIXME: Obtain more accurate value
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


    def current_motor_angular_positions(self):
        return self.motor_indices.applymap(lambda index: bullet.getJointState(self.id, index, self.in_physics_client)[0])


    def pitching(self, angle, ref_motor_angular_positions=None) -> None:

        if ref_motor_angular_positions is None:
            # FIXME: 取 angle 为 0.0 相对当前位置反复计算可以观察到, 误差会累加, 机器狗姿态慢慢畸形.
            # Possible Sol: 数值精度缩小到 3 decimal places
            angular_positions = self.current_motor_angular_positions()
        else:
            angular_positions = ref_motor_angular_positions.copy()

        for leg, θ in angular_positions.items():  #! 引用传递; 修改 θ 即修改 angular_positions 中的元素
            θ0, θ1, θ2 = θ  # θ is a tuple of three motor angular positions of one leg
            # Abbreviating others to increase readability further
            δ  = angle
            ℓ1 = self.thigh_len
            ℓ2 = self.calf_len
            L  = self.body_len / 2
            o  = self.hip_offset

            # θ0, θ1, θ2 ⇒ x, h, y, z
            x = -ℓ1 * np.sin(θ1) - ℓ2 * np.sin(θ1 + θ2)
            h =  ℓ1 * np.cos(θ1) + ℓ2 * np.cos(θ1 + θ2)
            match leg:
                case "fr" | "hr":
                    y = -o * np.cos(θ0) - h * np.sin(θ0)
                    z =  o * np.sin(θ0) - h * np.cos(θ0)
                case "fl" | "hl":
                    y =  o * np.cos(θ0) - h * np.sin(θ0)
                    z = -o * np.sin(θ0) - h * np.cos(θ0)

            # x, z, h after pitching
            match leg:
                case "fr" | "fl":
                    pitch = np.array([[ np.cos(δ), -np.sin(δ),  L * np.cos(δ) - L ],
                                      [ np.sin(δ),  np.cos(δ),  L * np.sin(δ)     ],
                                      [ 0,          0,          1                 ]])
                case "hr" | "hl":
                    pitch = np.array([[ np.cos(δ), -np.sin(δ), -L * np.cos(δ) + L ],
                                      [ np.sin(δ),  np.cos(δ), -L * np.sin(δ)     ],
                                      [ 0,          0,          1                 ]])
            x, z, _ = pitch.dot(np.array([x, z, 1]))
            h = np.sqrt(z**2 + y**2 - o**2)

            # x, h, y, z ⇒ θ1, θ2
            cosθ2 = (-ℓ1**2 - ℓ2**2 + x**2 + h**2) / (2 * ℓ1 * ℓ2)
            sinθ2 = -np.sqrt(1 - cosθ2**2)  # takes negative suqare root, because the knee's position is specified as negative
            θ[1] = np.arccos((ℓ1**2 + x**2 + h**2 - ℓ2**2) / (2 * ℓ1 * np.sqrt(x**2 + h**2))) - np.arctan2(x, h)
            θ[2] = np.arctan2(sinθ2, cosθ2)

        # Motor angular position control
        for positions_of_one_leg, indices_of_one_leg in zip(angular_positions.itertuples(index=False), self.motor_indices.itertuples(index=False)):
            bullet.setJointMotorControlArray(
                physicsClientId = self.in_physics_client,
                bodyUniqueId = self.id,
                controlMode = bullet.POSITION_CONTROL,
                jointIndices = indices_of_one_leg,
                targetPositions = positions_of_one_leg,)
