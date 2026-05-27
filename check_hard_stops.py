#!/usr/bin/env python3
"""
Check for hard stops and collisions limiting gripper range
"""

import mujoco
import numpy as np
import time

def main():
    print("="*70)
    print("HARD STOPS & COLLISIONS ANALYSIS")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    
    # Load model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Get IDs
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
    
    print(f"Joint ranges from model:")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        jnt_range = model.jnt_range[i]
        print(f"  {name}: [{jnt_range[0]:.3f}, {jnt_range[1]:.3f}] radians")
        print(f"  {name}: [{np.rad2deg(jnt_range[0]):.1f}°, {np.rad2deg(jnt_range[1]):.1f}°]")
    
    print(f"\nTesting extreme control values...")
    
    # Test extreme control values
    test_controls = [-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0]
    
    for control in test_controls:
        print(f"\n--- Control: {control:.1f} ---")
        
        # Set control
        data.ctrl[act_id] = control
        
        # Run simulation for 2 seconds to settle
        for step in range(1000):
            mujoco.mj_step(model, data)
        
        # Get final positions
        f1_palm = np.rad2deg(data.qpos[f1_palm_id])
        f2_palm = np.rad2deg(data.qpos[f2_palm_id])
        f1_tip = np.rad2deg(data.qpos[f1_tip_id])
        f2_tip = np.rad2deg(data.qpos[f2_tip_id])
        delta = abs(f1_palm - f2_palm)
        
        # Check if at joint limits
        f1_palm_at_limit = abs(data.qpos[f1_palm_id] - model.jnt_range[f1_palm_id, 0]) < 0.001 or abs(data.qpos[f1_palm_id] - model.jnt_range[f1_palm_id, 1]) < 0.001
        f2_palm_at_limit = abs(data.qpos[f2_palm_id] - model.jnt_range[f2_palm_id, 0]) < 0.001 or abs(data.qpos[f2_palm_id] - model.jnt_range[f2_palm_id, 1]) < 0.001
        
        print(f"Joints: F1: {f1_palm:6.2f}° F2: {f2_palm:6.2f}° Δ: {delta:5.2f}°")
        print(f"At limits: F1: {'✅' if f1_palm_at_limit else '❌'} F2: {'✅' if f2_palm_at_limit else '❌'}")
        
        # Check contacts
        contacts = 0
        for j in range(data.ncon):
            contact = data.contact[j]
            contacts += 1
        
        print(f"Contacts: {contacts}")
        
        # Check forces
        if contacts > 0:
            print(f"Contact forces detected - likely hitting hard stops")
    
    print(f"\n" + "="*70)
    print("CHECKING FOR MECHANICAL STOPS")
    print("="*70)
    
    # Look for mechanical stop geoms in the XML
    with open(model_path, 'r') as f:
        xml_content = f.read()
    
    stop_geoms = []
    lines = xml_content.split('\n')
    for i, line in enumerate(lines):
        if 'stop_' in line and 'geom' in line:
            stop_geoms.append((i+1, line.strip()))
    
    if stop_geoms:
        print(f"Found {len(stop_geoms)} mechanical stop geoms:")
        for line_num, line in stop_geoms:
            print(f"  Line {line_num}: {line}")
    else:
        print("No mechanical stop geoms found")
    
    print(f"\n" + "="*70)
    print("HARD STOPS ANALYSIS COMPLETE")
    print("="*70)

if __name__ == "__main__":
    main()
