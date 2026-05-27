#!/usr/bin/env python3
"""
Standard Test Bench 2: Active Tendon Closing
Tests gripper closing with tendon actuation.
Verifies underactuated behavior, joint sequencing, and limit enforcement.
"""
import mujoco
import mujoco.viewer
import numpy as np
import time
import os

# Load model
script_dir = os.path.dirname(os.path.abspath(__file__))
model_path = os.path.join(script_dir, "ezgripper.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("=" * 80)
print("TEST BENCH 2: Active Tendon Closing")
print("=" * 80)

# Get joint IDs
f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')

# Get tendon IDs
f1_tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
f2_tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')

# Get actuator IDs
gripper_f1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
gripper_f2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')

print("\nActuator Configuration:")
print(f"  Tendon range: {model.actuator_ctrlrange[gripper_f1_id,0]*1000:.1f}mm to {model.actuator_ctrlrange[gripper_f1_id,1]*1000:.1f}mm")
print(f"  Force range: {model.actuator_forcerange[gripper_f1_id]}")

print("\nJoint Ranges:")
print(f"  F1_palm_knuckle: {np.degrees(model.jnt_range[f1_palm_id,0]):.1f}° to {np.degrees(model.jnt_range[f1_palm_id,1]):.1f}°")
print(f"  F1_knuckle_tip:  {np.degrees(model.jnt_range[f1_tip_id,0]):.1f}° to {np.degrees(model.jnt_range[f1_tip_id,1]):.1f}°")

print("\n" + "=" * 80)
print("Test Sequence:")
print("  Phase 1 (0-3s):   Open position (tendon = 200mm)")
print("  Phase 2 (3-8s):   Gradual closing (200mm → 100mm)")
print("  Phase 3 (8-12s):  Hold closed (tendon = 100mm)")
print("=" * 80 + "\n")

last_print = 0
phase = 1
phase_start_time = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Start with open position
    tendon_length = 0.200
    data.ctrl[gripper_f1_id] = tendon_length
    data.ctrl[gripper_f2_id] = tendon_length
    
    while viewer.is_running():
        step_start = time.time()
        
        # Update phase and tendon control
        if phase == 1 and data.time >= 3.0:
            phase = 2
            phase_start_time = data.time
            print("\n" + "=" * 80)
            print("PHASE 2: Closing gripper...")
            print("=" * 80 + "\n")
        elif phase == 2 and data.time >= 8.0:
            phase = 3
            print("\n" + "=" * 80)
            print("PHASE 3: Holding closed position")
            print("=" * 80 + "\n")
        elif phase == 3 and data.time >= 12.0:
            print("\n" + "=" * 80)
            print("TEST COMPLETE")
            print("=" * 80)
            break
        
        # Control tendon length based on phase
        if phase == 1:
            tendon_length = 0.200  # Open
        elif phase == 2:
            # Gradually close over 5 seconds
            progress = (data.time - phase_start_time) / 5.0
            tendon_length = 0.200 - progress * 0.100  # 0.200 → 0.100
            tendon_length = max(0.100, tendon_length)
        else:  # phase == 3
            tendon_length = 0.100  # Closed
        
        data.ctrl[gripper_f1_id] = tendon_length
        data.ctrl[gripper_f2_id] = tendon_length
        
        # Print status every 0.5 seconds
        if data.time - last_print >= 0.5:
            f1_palm_angle = data.qpos[f1_palm_id]
            f1_tip_angle = data.qpos[f1_tip_id]
            f2_palm_angle = data.qpos[f2_palm_id]
            f2_tip_angle = data.qpos[f2_tip_id]
            
            f1_tendon_len = data.ten_length[f1_tendon_id]
            f2_tendon_len = data.ten_length[f2_tendon_id]
            
            # Get actuator forces
            f1_act_force = data.actuator_force[gripper_f1_id]
            f2_act_force = data.actuator_force[gripper_f2_id]
            
            print(f"[{data.time:5.2f}s] Phase {phase} | Tendon cmd: {tendon_length*1000:.1f}mm")
            print(f"  F1_palm: {np.degrees(f1_palm_angle):6.1f}° | F1_tip: {np.degrees(f1_tip_angle):6.1f}°")
            print(f"  F2_palm: {np.degrees(f2_palm_angle):6.1f}° | F2_tip: {np.degrees(f2_tip_angle):6.1f}°")
            print(f"  Tendon: F1={f1_tendon_len*1000:.2f}mm, F2={f2_tendon_len*1000:.2f}mm")
            print(f"  Forces: F1={f1_act_force:.1f}N, F2={f2_act_force:.1f}N")
            
            # Check for limit violations
            if f1_palm_angle < model.jnt_range[f1_palm_id,0] - 0.01 or f1_palm_angle > model.jnt_range[f1_palm_id,1] + 0.01:
                print(f"  ⚠️  WARNING: F1_palm outside limits!")
            if f1_tip_angle < model.jnt_range[f1_tip_id,0] - 0.01 or f1_tip_angle > model.jnt_range[f1_tip_id,1] + 0.01:
                print(f"  ⚠️  WARNING: F1_tip outside limits!")
            
            print()
            last_print = data.time
        
        mujoco.mj_step(model, data)
        viewer.sync()
        
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("\nTest finished!")
