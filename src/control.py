from src.data import State, ActuatorIndex 

class Controller:
    """
    Takes current state and defines desired actuation (u) based on control logic 
    to send to orca_core to execute on hardware

    The actuations list (u) acts as connecting point to orca core / franka arm 
    """
    
    def __init__(self, state):
        self.state: State = state
        self.u = [0] * len(ActuatorIndex) 
    