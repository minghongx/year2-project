from dataclasses import dataclass
from typing import Tuple

@dataclass
class A1MotorIndices:
    # Front Right
    fr_hip_abduction_or_adduction: int
    fr_hip_flexion_or_extension: int
    fr_knee: int
    # Front Left
    fl_hip_abduction_or_adduction: int
    fl_hip_flexion_or_extension: int
    fl_knee: int
    # Hind Right
    hr_hip_abduction_or_adduction: int
    hr_hip_flexion_or_extension: int
    hr_knee: int
    # Hind Left
    hl_hip_abduction_or_adduction: int
    hl_hip_flexion_or_extension: int
    hl_knee: int

    def hip_abduction_or_adduction(self) -> Tuple[int, ...]:
        return (self.fr_hip_abduction_or_adduction,
                self.fl_hip_abduction_or_adduction,
                self.hr_hip_abduction_or_adduction,
                self.hl_hip_abduction_or_adduction,)

    def hip_flexion_or_extension(self) -> Tuple[int, ...]:
        return (self.fr_hip_flexion_or_extension,
                self.fl_hip_flexion_or_extension,
                self.hr_hip_flexion_or_extension,
                self.hl_hip_flexion_or_extension,)

    def knee(self) -> Tuple[int, ...]:
        return (self.fr_knee,
                self.fl_knee,
                self.hr_knee,
                self.hl_knee,)

def a1_motor_indices():
    return A1MotorIndices(
        fr_hip_abduction_or_adduction = 1,
        fr_hip_flexion_or_extension = 3,
        fr_knee = 4,
        fl_hip_abduction_or_adduction = 6,
        fl_hip_flexion_or_extension = 8,
        fl_knee = 9,
        hr_hip_abduction_or_adduction = 11,
        hr_hip_flexion_or_extension = 13,
        hr_knee = 14,
        hl_hip_abduction_or_adduction = 16,
        hl_hip_flexion_or_extension = 18,
        hl_knee = 19)
