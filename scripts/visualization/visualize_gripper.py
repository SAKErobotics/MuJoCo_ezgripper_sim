#!/usr/bin/env python3
"""
Visualize the EZGripper with active tendon control.
"""

import mujoco
import mujoco.viewer
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("EZGRIPPER VISUALIZATION")
print("="*80)
print("Controls:")
print("  [1] Open gripper (tendon length = 0.200)")
print("  [2] Close gripper (tendon length = 0.100)")
print("  [3] Neutral position (tendon length = 0.150)")
print("  [4] Toggle spring-only mode (disable tendons)")
print("  [ESC] Exit")
print("="*80)

# Get actuator IDs
f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

# Get joint IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

# Control variables
target_length = 0.150  # Start at neutral
springs_only = False

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Set camera view
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    viewer.cam.distance = 0.5
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.1, 0, 0.1])

    while viewer.is_running():
        # Get current state
        f1_palm_angle = np.degrees(data.qpos[f1_palm_id])
        f1_tip_angle = np.degrees(data.qpos[f1_tip_id])
        f2_palm_angle = np.degrees(data.qpos[f2_palm_id])
        f2_tip_angle = np.degrees(data.qpos[f2_tip_id])
        
        f1_tendon_length = data.ten_length[0] if not springs_only else 0
        f2_tendon_length = data.ten_length[1] if not springs_only else 0
        f1_spring_torque = data.qfrc_passive[f1_palm_id]
        
        # Update status display (print every 100 steps)
        if data.time % 1.0 < 0.01:  # Print every second
            mode = "SPRINGS ONLY" if springs_only else "ACTIVE CONTROL"
            status = (f"{mode} | "
                    f"F1: Palm={f1_palm_angle:6.1f}°, Tip={f1_tip_angle:6.1f}° | "
                    f"F2: Palm={f2_palm_angle:6.1f}°, Tip={f2_tip_angle:6.1f}° | "
                    f"Tendon: {f1_tendon_length:.3f} | "
                    f"Spring: {f1_spring_torque:.4f} N·m | "
                    f"Contacts: {data.ncon}")
            print(f"[{data.time:6.2f}s] {status}")
        
        # Handle keyboard input (simplified - just run demo sequence)
        if data.time < 2.0:
            target_length = 0.150  # Neutral
        elif data.time < 4.0:
            target_length = 0.200  # Open
            if data.time > 2.0 and data.time < 2.1:
                print("Opening gripper...")
        elif data.time < 6.0:
            target_length = 0.100  # Close
            if data.time > 4.0 and data.time < 4.1:
                print("Closing gripper...")
        elif data.time < 8.0:
            target_length = 0.200  # Open again
            if data.time > 6.0 and data.time < 6.1:
                print("Opening gripper again...")
        else:
            target_length = 0.150  # Back to neutral
            if data.time > 8.0 and data.time < 8.1:
                print("Back to neutral...")
        
        # Apply control if not in springs-only mode
        if not springs_only:
            data.ctrl[f1_actuator_id] = target_length
            data.ctrl[f2_actuator_id] = target_length
        
        # Step simulation
        mujoco.mj_step(model, data)
        
        # Sync viewer
        viewer.sync()

print("\nVisualization ended.")
