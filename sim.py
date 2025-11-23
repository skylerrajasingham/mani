import sys
import numpy as np

import mujoco
import mujoco.viewer
import argparse

from src.control import Controller
from src.data import State

parser = argparse.ArgumentParser()
parser.add_argument('-fr', '--frank-fr3', action="store_true", help="Include Franka FR3 arm in sim")
args = parser.parse_args(sys.argv[1:])

if args.frank_fr3:
    MODEL_XML = "simulation/scene_fr3_orca.xml"
    additional_actuators = []
else:
    MODEL_XML = "simulation/scene_orca.xml"
    # extra actuators that don't actually exist to control floating hand (x, y, z) in sim
    additional_actuators = [0] * 3

# Load model and create data
model = mujoco.MjModel.from_xml_path(MODEL_XML)
data = mujoco.MjData(model)

# Initialize mani
mani = Controller(
    state=State(
        pos_rad=data.qpos,
        _torque_nm=data.qfrc_actuator,
    )
)

# Print Actuators
for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(f"Actuator {i}: {name}")

# Start the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        
        # Control with mani
        data.ctrl = additional_actuators + mani.u[:model.nu - len(additional_actuators)]
        
        # Step simulation
        mujoco.mj_step(model, data)

        # Render
        viewer.sync()

 