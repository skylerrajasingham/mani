import sys

import mujoco
import mujoco.viewer
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-fr', '--frank-fr3', action="store_true", help="Include Franka FR3 arm in sim")
args = parser.parse_args(sys.argv[1:])

# Path to your MJCF model (pendulum is simple for demo)
if args.frank_fr3:
    MODEL_XML = "simulation/scene_fr3_orca.xml"  # replace with your model path if needed
else:
    MODEL_XML = "simulation/scene_orca.xml"

# Load model and create data
model = mujoco.MjModel.from_xml_path(MODEL_XML)
data = mujoco.MjData(model)

for i in range(model.nu):
    name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
    print(f"Actuator {i}: {name}")

# Start the viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        # Step simulation
        mujoco.mj_step(model, data)

        # Render
        viewer.sync()

 