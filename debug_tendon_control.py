#!/usr/bin/env python3
"""
Debug: Test tendon control directly
"""

import mujoco
import numpy as np
import time

def main():
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    
    # Load model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Get IDs
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    
    print("="*70)
    print("TENDON CONTROL DEBUG")
    print("="*70)
    
    # Test different control values
    test_values = [0.0, -0.5, -1.0, 0.0]
    
    for i, ctrl_val in enumerate(test_values):
        print(f"\n--- Test {i+1}: Control = {ctrl_val} ---")
        
        # Set control
        data.ctrl[act_id] = ctrl_val
        
        # Run simulation for 2 seconds
        for step in range(1000):
            mujoco.mj_step(model, data)
            
            # Print status every 200 steps
            if step % 200 == 0:
                f1_palm = np.rad2deg(data.qpos[f1_palm_id])
                f2_palm = np.rad2deg(data.qpos[f2_palm_id])
                delta = abs(f1_palm - f2_palm)
                
                print(f"Step {step:3d}: Ctrl={ctrl_val:6.2f} | F1: {f1_palm:6.2f}° F2: {f2_palm:6.2f}° Δ: {delta:5.2f}°")
        
        time.sleep(0.5)  # Pause between tests
    
    print("\n" + "="*70)
    print("TENDON CONTROL TEST COMPLETE")
    print("="*70)

if __name__ == "__main__":
    main()
