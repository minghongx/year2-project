import pybullet as bullet
import pandas as pd
import numpy as np
from enum import Enum
from typing import Iterable

class A1:

    class EnumWithListMethod(Enum):
        @classmethod
        def list(cls):
            return list(map(lambda c: c.value, cls))

    class Leg(EnumWithListMethod):  # FIXME: Use 3.11 enum.StrEnum to remove .value
        fr = 'front-right'
        fl = 'front-left'
        hr = 'hind-right'
        hl = 'hind-left'

    class Motor(EnumWithListMethod):  # FIXME: Use 3.11 enum.StrEnum to remove .value
        h_aa = 'hip abduction/adduction'
        h_fe = 'hip flexion/extension'
        knee = 'knee'

    motor_indices = pd.DataFrame(dtype=np.int8, index = Motor.list(), columns = Leg.list(),
                        # front-right, front-left, hind-right, hind-left
        data = np.array([[     1     ,      6    ,     11    ,     16   ],    # hip abduction/adduction
                         [     3     ,      8    ,     13    ,     18   ],    # hip flexion/extension
                         [     4     ,      9    ,     14    ,     19   ]]))  # knee
    # Obtained by measuring the STL file of the thigh and calf. Both are 0.2mm.
    thigh_len  = 200
    shank_len  = 200
    hip_offset = 80   # FIXME: Obtain more accurate value
    body_len   = 360  # FIXME: Obtain more accurate value
    body_width = 200  # FIXME: Obtain more accurate value


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
        # Set initial postion of motors.
        bullet.setJointMotorControlArray(
            physicsClientId = self.in_physics_client,
            bodyUniqueId = self.id,
            jointIndices = self.motor_indices.loc[A1.Motor.h_aa.value, :],  # FIXME: Use 3.11 enum.StrEnum to remove .value
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, 0),)
        bullet.setJointMotorControlArray(
            physicsClientId = self.in_physics_client,
            bodyUniqueId = self.id,
            jointIndices = self.motor_indices.loc[A1.Motor.h_fe.value, :],  # FIXME: Use 3.11 enum.StrEnum to remove .value
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, 0.7),)
        bullet.setJointMotorControlArray(
            physicsClientId = self.in_physics_client,
            bodyUniqueId = self.id,
            jointIndices = self.motor_indices.loc[A1.Motor.knee.value, :],  # FIXME: Use 3.11 enum.StrEnum to remove .value
            controlMode = bullet.POSITION_CONTROL,
            targetPositions = np.full(4, -0.7*2),)  # ensures that the toes are beneath the hips


    def motor_info(self):
        """
        A matrix contains motor information. The info is structured as
        Tuple(jointIndex, jointName, jointType, qIndex, uIndex, flags, jointDamping,
        jointFriction, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity,
        linkName, jointAxis, parentFramePos, parentFrameOrn, parentIndex)

        See PyBullet document for getJointInfo().
        """
        return A1.motor_indices.applymap(lambda index: bullet.getJointInfo(self.id, index))


    def current_motor_angular_positions(self):
        return A1.motor_indices.applymap(lambda index: bullet.getJointState(self.id, index, self.in_physics_client)[0])


    def adjust_posture(self, roll_angle=0., pitch_angle=0., yaw_angle=0., δz=0., ref_motor_angular_positions=None) -> None:

        if ref_motor_angular_positions is None:
            # FIXME: 取 angle 为 0.0 相对当前位置反复计算可以观察到, 误差会累加, 机器狗姿态慢慢畸形.
            # Possible Sol: 数值精度缩小到 3 decimal places
            angular_positions = self.current_motor_angular_positions()
        else:
            angular_positions = ref_motor_angular_positions.copy()

        for leg, θ in angular_positions.items():  #! 引用传递; 修改 θ 即修改 angular_positions 中的元素
            # θ is a tuple of three motor angular positions of one leg

            # Abbreviating to increase readability
            λ = roll_angle
            δ = pitch_angle
            β = yaw_angle
            L = A1.body_len / 2
            W = A1.body_width / 2

            x, y, z = A1._forward_kinematics(leg, *θ)

            # adjust height
            z -= δz

            # x, y, z after rolling
            match leg:
                case A1.Leg.fr.value | A1.Leg.hr.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                    roll = np.array([[ 1,  0,          0,          0                 ],
                                     [ 0,  np.cos(λ), -np.sin(λ),  W * np.cos(λ) - W ],
                                     [ 0,  np.sin(λ),  np.cos(λ),  W * np.sin(λ)     ],
                                     [ 0,  0,          0,          1                 ]])
                case A1.Leg.fl.value | A1.Leg.hl.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                    roll = np.array([[ 1,  0,          0,          0                 ],
                                     [ 0,  np.cos(λ), -np.sin(λ), -W * np.cos(λ) + W ],
                                     [ 0,  np.sin(λ),  np.cos(λ), -W * np.sin(λ)     ],
                                     [ 0,  0,          0,          1                 ]])
            x, y, z, _ = roll.dot(np.array([x, y, z, 1]))

            # x, z after pitching
            match leg:
                case A1.Leg.fr.value | A1.Leg.fl.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                    pitch = np.array([[ np.cos(δ), -np.sin(δ),  L * np.cos(δ) - L ],
                                      [ np.sin(δ),  np.cos(δ),  L * np.sin(δ)     ],
                                      [ 0,          0,          1                 ]])
                case A1.Leg.hr.value | A1.Leg.hl.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                    pitch = np.array([[ np.cos(δ), -np.sin(δ), -L * np.cos(δ) + L ],
                                      [ np.sin(δ),  np.cos(δ), -L * np.sin(δ)     ],
                                      [ 0,          0,          1                 ]])
            x, z, _ = pitch.dot(np.array([x, z, 1]))

            # x, y, z after yawing
            match leg:
                case A1.Leg.fr.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                    yaw = np.array([[ np.cos(β), -np.sin(β),   0,   L * np.cos(β) - W * np.sin(β) - L ],
                                    [ np.sin(β),  np.cos(β),   0,   L * np.sin(β) + W * np.cos(β) - W ],
                                    [ 0,          0,           1,   0                                 ],
                                    [ 0,          0,           0,   1                                 ]])
                case A1.Leg.fl.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                    yaw = np.array([[ np.cos(β), -np.sin(β),   0,   L * np.cos(β) + W * np.sin(β) - L ],
                                    [ np.sin(β),  np.cos(β),   0,   L * np.sin(β) - W * np.cos(β) + W ],
                                    [ 0,          0,           1,   0                                 ],
                                    [ 0,          0,           0,   1                                 ]])
                case A1.Leg.hr.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                    yaw = np.array([[ np.cos(β), -np.sin(β),   0,  -L * np.cos(β) - W * np.sin(β) + L ],
                                    [ np.sin(β),  np.cos(β),   0,  -L * np.sin(β) + W * np.cos(β) - W ],
                                    [ 0,          0,           1,   0                                 ],
                                    [ 0,          0,           0,   1                                 ]])
                case A1.Leg.hl.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                    yaw = np.array([[ np.cos(β), -np.sin(β),   0,  -L * np.cos(β) + W * np.sin(β) + L ],
                                    [ np.sin(β),  np.cos(β),   0,  -L * np.sin(β) - W * np.cos(β) + W ],
                                    [ 0,          0,           1,   0                                 ],
                                    [ 0,          0,           0,   1                                 ]])
            x, y, z, _ = yaw.dot(np.array([x, y, z, 1]))

            θ[0], θ[1], θ[2] = A1._inverse_kinematics(leg, x, y, z)

        # Motor angular position control
        for positions_of_one_leg, indices_of_one_leg in zip(angular_positions.itertuples(index=False), A1.motor_indices.itertuples(index=False)):
            bullet.setJointMotorControlArray(
                physicsClientId = self.in_physics_client,
                bodyUniqueId = self.id,
                controlMode = bullet.POSITION_CONTROL,
                jointIndices = indices_of_one_leg,
                targetPositions = positions_of_one_leg,)


    @classmethod
    def _forward_kinematics(cls, leg: Leg, θ0, θ1, θ2):
        """
        θ0, θ1, θ2 ⇒ x, y, z

        :param θ0: angular position of hip abd/add motor
        :param θ1: angular position of hip fle/ext motor
        :param θ2: angular position of knee motor
        :return  : a tuple, (x, y, z), which is coordinates of the toe relative to the hip
        """

        # Abbreviating to increase readability
        ℓ1 = cls.thigh_len
        ℓ2 = cls.shank_len
        o  = cls.hip_offset
        h =  ℓ1 * np.cos(θ1) + ℓ2 * np.cos(θ1 + θ2)  # distance from the toe to the top centre of the thigh rod

        x = -ℓ1 * np.sin(θ1) - ℓ2 * np.sin(θ1 + θ2)
        match leg:
            case A1.Leg.fr.value | A1.Leg.hr.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                y =  o * np.cos(θ0) - h * np.sin(θ0)
                z = -o * np.sin(θ0) - h * np.cos(θ0)
            case A1.Leg.fl.value | A1.Leg.hl.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                y = -o * np.cos(θ0) - h * np.sin(θ0)
                z =  o * np.sin(θ0) - h * np.cos(θ0)

        return (x, y, z)


    @classmethod
    def _inverse_kinematics(cls, leg: Leg, x, y, z):
        """
        x, y, z ⇒ θ0, θ1, θ2

        :param x, y, z: coordinates of the toe relative to the hip
        :return       : a tuple, (θ0, θ1, θ2), which is angular position of three motors on a leg
        """

        # Abbreviating to increase readability
        ℓ1 = cls.thigh_len
        ℓ2 = cls.shank_len
        o  = cls.hip_offset
        h = np.sqrt(z**2 + y**2 - o**2)

        cosθ2 = (-ℓ1**2 - ℓ2**2 + x**2 + h**2) / (2 * ℓ1 * ℓ2)
        sinθ2 = -np.sqrt(1 - cosθ2**2)  # takes negative suqare root, because the knee's position is specified as negative
        match leg:
            case A1.Leg.fr.value | A1.Leg.hr.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                θ0 = np.arctan2(np.abs(z), y) - np.arctan2(h, o)
            case A1.Leg.fl.value | A1.Leg.hl.value:  # FIXME: Use 3.11 enum.StrEnum to remove .value
                θ0 = np.arctan2(h, o) - np.arctan2(np.abs(z), -y)
        θ1 = np.arccos((ℓ1**2 + x**2 + h**2 - ℓ2**2) / (2 * ℓ1 * np.sqrt(x**2 + h**2))) - np.arctan2(x, h)
        θ2 = np.arctan2(sinθ2, cosθ2)

        return (θ0, θ1, θ2)
