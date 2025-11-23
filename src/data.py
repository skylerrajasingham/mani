from dataclasses import dataclass
from typing import List
from enum import IntEnum, auto

@dataclass
class State:
    """ 
    Feedback from ORCA core provides pos (rad), vel (idk unit), and
    current (mA) from each actuator 
    """
    pos_rad: List[float]
    # vel: List[float] #TODO: add when needed
    _torque_nm: List[float]
    # TODO: Verify torque -> controllable current value conversion
    _Kt_nm_milliamp = 1.177 * 1000 # ask Skyler about this value - its rough for now
    
    @property
    def armature_current_mA(self):
        """ the electric current flowing through the rotating part of an electric motor """
        return self._torque_nm * (self._Kt_nm_milliamp)

class ActuatorIndex(IntEnum):
    def _generate_next_value_(name, start, count, last_values):
        # start enum count from 0
        return start - 1 + count

    # Orca actuators
    WRIST = auto()
    THUMB_MCP = auto()
    THUMB_ABD = auto()
    THUMB_PIP = auto()
    THUMB_DIP = auto()
    INDEX_ABD = auto()
    INDEX_MCP = auto()
    INDEX_PIP = auto()
    MIDDLE_ABD = auto()
    MIDDLE_MCP = auto()
    MIDDLE_PIP = auto()
    RING_ABD = auto()
    RING_MCP = auto()
    RING_PIP = auto()
    PINKY_ABD = auto()
    PINKY_MCP = auto()
    PINKY_PIP = auto()
    T_INDEX = auto()
    T_MIDDLE = auto()
    # Franka actuators
    FR3_J1 = auto()
    FR3_J2 = auto()
    FR3_J3 = auto()
    FR3_J4 = auto()
    FR3_J5 = auto()
    FR3_J6 = auto()
    FR3_J7 = auto()
    FR3_J8 = auto()