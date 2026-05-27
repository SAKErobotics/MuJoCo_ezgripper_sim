#!/usr/bin/env python3
"""
Test script to verify EZGripper model loads with MuJoCo 3.0+
"""
import mujoco
import mujoco.viewer
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print(f"Testing EZGripper model: {model_path}")
print(f"MuJoCo version: {mujoco.__version__}")

try:
    # Load the model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print("✓ Model loaded successfully!")
    print(f"  - Number of bodies: {model.nbody}")
    print(f"  - Number of joints: {model.njnt}")
    print(f"  - Number of actuators: {model.nu}")
    print(f"  - Number of tendons: {model.ntendon}")
    
    # Print actuator info
    print("\nActuators:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  {i}: {actuator_name}")
    
    # Print tendon info
    print("\nTendons:")
    for i in range(model.ntendon):
        tendon_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, i)
        print(f"  {i}: {tendon_name}")
    
    # Launch interactive viewer
    print("\nLaunching interactive viewer...")
    print("Controls:")
    print("  - Double-click on gripper to select")
    print("  - Use sliders to control actuators")
    print("  - Press ESC to exit")
    
    mujoco.viewer.launch(model)
    
except Exception as e:
    print(f"✗ Error loading model: {e}")
    import traceback
    traceback.print_exc()
