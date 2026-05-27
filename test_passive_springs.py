#!/usr/bin/env python3
"""
Standard Test Bench 1: Passive Spring Behavior
Tests gripper settling to open position with NO tendon actuation.
Verifies spring preload, joint limits, and mechanical stops.
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
print("TEST BENCH 1: Passive Spring Behavior (No Tendon Actuation)")
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

print("\nSpring Configuration:")
print(f"  F1_palm_knuckle: stiffness={model.jnt_stiffness[f1_palm_id]:.6f}, springref={model.qpos_spring[f1_palm_id]:.4f} rad ({np.degrees(model.qpos_spring[f1_palm_id]):.1f}°)")
print(f"  F1_knuckle_tip:  stiffness={model.jnt_stiffness[f1_tip_id]:.6f}, springref={model.qpos_spring[f1_tip_id]:.4f} rad ({np.degrees(model.qpos_spring[f1_tip_id]):.1f}°)")

print("\nJoint Ranges:")
print(f"  F1_palm_knuckle: {model.jnt_range[f1_palm_id,0]:.4f} to {model.jnt_range[f1_palm_id,1]:.4f} rad ({np.degrees(model.jnt_range[f1_palm_id,0]):.1f}° to {np.degrees(model.jnt_range[f1_palm_id,1]):.1f}°)")
print(f"  F1_knuckle_tip:  {model.jnt_range[f1_tip_id,0]:.4f} to {model.jnt_range[f1_tip_id,1]:.4f} rad ({np.degrees(model.jnt_range[f1_tip_id,0]):.1f}° to {np.degrees(model.jnt_range[f1_tip_id,1]):.1f}°)")

print("\n" + "=" * 80)
print("Expected: Gripper should settle to open position at joint limits")
print("  Palm-L1: ~-90° (at lower limit)")
print("  L1-L2: ~0° (at lower limit)")
print("=" * 80 + "\n")

last_print = 0
settled = False

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.distance = 0.4
    viewer.cam.azimuth = 45
    viewer.cam.elevation = -20
    viewer.cam.lookat = np.array([0.14, 0.0, 0.08])
    
    # Set actuator controls to zero (no tendon force)
    data.ctrl[gripper_f1_id] = 0.180  # Set to long length (no pulling)
    data.ctrl[gripper_f2_id] = 0.180
    
    while viewer.is_running():
        step_start = time.time()
        
        # Print status every 0.5 seconds
        if data.time - last_print >= 0.5:
            f1_palm_angle = data.qpos[f1_palm_id]
            f1_tip_angle = data.qpos[f1_tip_id]
            f2_palm_angle = data.qpos[f2_palm_id]
            f2_tip_angle = data.qpos[f2_tip_id]
            
            f1_tendon_len = data.ten_length[f1_tendon_id]
            f2_tendon_len = data.ten_length[f2_tendon_id]
            
            # Calculate spring torques
            f1_palm_torque = model.jnt_stiffness[f1_palm_id] * (model.qpos_spring[f1_palm_id] - f1_palm_angle)
            f1_tip_torque = model.jnt_stiffness[f1_tip_id] * (model.qpos_spring[f1_tip_id] - f1_tip_angle)
            
            # Get velocities
            f1_palm_vel = data.qvel[f1_palm_id]
            f1_tip_vel = data.qvel[f1_tip_id]
            
            print(f"[{data.time:5.2f}s]")
            print(f"  F1_palm: {np.degrees(f1_palm_angle):6.1f}° | torque: {f1_palm_torque:+7.4f} N·m | vel: {f1_palm_vel:+7.4f} rad/s")
            print(f"  F1_tip:  {np.degrees(f1_tip_angle):6.1f}° | torque: {f1_tip_torque:+7.4f} N·m | vel: {f1_tip_vel:+7.4f} rad/s")
            print(f"  F2_palm: {np.degrees(f2_palm_angle):6.1f}° | torque: {model.jnt_stiffness[f2_palm_id] * (model.qpos_spring[f2_palm_id] - f2_palm_angle):+7.4f} N·m")
            print(f"  F2_tip:  {np.degrees(f2_tip_angle):6.1f}° | torque: {model.jnt_stiffness[f2_tip_id] * (model.qpos_spring[f2_tip_id] - f2_tip_angle):+7.4f} N·m")
            print(f"  Tendon: F1={f1_tendon_len*1000:.2f}mm, F2={f2_tendon_len*1000:.2f}mm")
            
            # Check if settled (velocities near zero)
            if not settled and abs(f1_palm_vel) < 0.001 and abs(f1_tip_vel) < 0.001 and data.time > 2.0:
                print("\n" + "=" * 80)
                print("STEADY STATE REACHED:")
                print(f"  F1_palm: {np.degrees(f1_palm_angle):.2f}° (range: -90° to 20°)")
                print(f"  F1_tip:  {np.degrees(f1_tip_angle):.2f}° (range: 0° to 97°)")
                print(f"  Spring torques: F1_palm={f1_palm_torque:.4f} N·m, F1_tip={f1_tip_torque:.4f} N·m")
                
                # Check for limit violations
                if f1_palm_angle < model.jnt_range[f1_palm_id,0] - 0.01:
                    print(f"  ⚠️  WARNING: F1_palm below lower limit!")
                if f1_tip_angle < model.jnt_range[f1_tip_id,0] - 0.01:
                    print(f"  ⚠️  WARNING: F1_tip below lower limit!")
                    
                print("=" * 80 + "\n")
                settled = True
            
            print()
            last_print = data.time
        
        mujoco.mj_step(model, data)
        viewer.sync()
        
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("\nTest finished!")
