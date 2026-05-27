#!/usr/bin/env python3
"""
Tennis Ball Grasp Demo for EZGripper

Demonstrates under-actuated grasping:
1. Gripper starts open above ball
2. Close gripper with moderate effort
3. L1 contacts ball first → stops
4. Tendon force overcomes L2 spring → L2 wraps
5. Adaptive conforming to spherical object
"""
import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# Get the directory of this script
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")

print("=" * 60)
print("EZGripper Tennis Ball Grasp Demo")
print("=" * 60)
print(f"Model: {model_path}")
print(f"MuJoCo version: {mujoco.__version__}")
print()

# Load the model
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("✓ Model loaded successfully!")
print(f"  - Bodies: {model.nbody}")
print(f"  - Joints: {model.njnt}")
print(f"  - Actuators: {model.nu}")
print(f"  - Tendons: {model.ntendon}")
print()

# Get actuator ID (single actuator controls both fingers via tendon coupling)
gripper_actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')

# Demo sequence
print("Demo Sequence:")
print("  1. Gripper starts OPEN (springs extended)")
print("  2. Wait 2 seconds")
print("  3. CLOSE gripper slowly (watch L1 and L2 behavior)")
print("  4. L1 contacts ball → stops")
print("  5. L2 wraps around ball (tendon overcomes spring)")
print("  6. Hold grasp for 3 seconds")
print("  7. OPEN gripper (springs return)")
print()
print("Controls:")
print("  - Space: Pause/Resume")
print("  - ESC: Exit")
print("  - Double-click: Select body")
print("  - Right-drag: Rotate view")
print()
print("Starting in 2 seconds...")
time.sleep(2)

# Simulation parameters
sim_time = 0.0
phase_info = {"phase": "open"}

def update_control(model, data, phase_info):
    """Update gripper control - cycle between open and close every 6 seconds"""
    current_time = data.time
    
    # Calculate cycle: 6 seconds per direction
    cycle_time = current_time % 12.0  # 12 second total cycle (6s close, 6s open)
    
    if cycle_time < 6.0:
        # Closing phase (0-6 seconds) - gradually increase closing force
        progress = cycle_time / 6.0
        control_force = 1.0 * progress  # 0 to 1.0 (positive = close)
        data.ctrl[gripper_actuator_id] = control_force
        if phase_info['phase'] != "closing":
            phase_info['phase'] = "closing"
            print(f"[{current_time:.2f}s] CLOSING (force: {control_force:.2f})")
    else:
        # Opening phase (6-12 seconds) - negative force to help springs open
        progress = (cycle_time - 6.0) / 6.0
        control_force = -0.5 * progress  # 0 to -0.5 (negative = assist opening)
        data.ctrl[gripper_actuator_id] = control_force
        if phase_info['phase'] != "opening":
            phase_info['phase'] = "opening"
            print(f"[{current_time:.2f}s] OPENING (force: {control_force:.2f})")

# Launch viewer with custom controller
print("\n" + "=" * 60)
print("Launching viewer...")
print("=" * 60 + "\n")

with mujoco.viewer.launch_passive(model, data) as viewer:
    # Initial setup
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Run simulation
    while viewer.is_running():
        step_start = time.time()
        
        # Update control
        update_control(model, data, phase_info)
        
        # Step simulation
        mujoco.mj_step(model, data)
        sim_time += model.opt.timestep
        
        # Update viewer
        viewer.sync()
        
        # Maintain real-time
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("\nDemo finished!")
