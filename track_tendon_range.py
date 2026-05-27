#!/usr/bin/env python3
"""
Track tendon length from full open to full closed to find correct range
"""

import mujoco
import numpy as np
import time

def main():
    print("="*70)
    print("TENDON LENGTH TRACKING - NO CYLINDER")
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
    
    # Get tendon IDs
    try:
        t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
        t2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')
        print(f"✓ Found tendons: finger1_tendon (ID: {t1_id}), finger2_tendon (ID: {t2_id})")
    except:
        print("✗ No tendons found")
        return
    
    print("\nTest sequence:")
    print("1. Full open (control = 0.0)")
    print("2. Gradually close to full (control = 1.0)")
    print("3. Track tendon lengths and joint positions")
    print("="*70)
    
    # Test different control values
    control_values = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]
    
    for control in control_values:
        print(f"\n--- Control: {control:.1f} ---")
        
        # Set control
        data.ctrl[act_id] = control
        
        # Run simulation for 1 second to settle
        for step in range(500):
            mujoco.mj_step(model, data)
        
        # Get final positions
        f1_palm = np.rad2deg(data.qpos[f1_palm_id])
        f2_palm = np.rad2deg(data.qpos[f2_palm_id])
        f1_tip = np.rad2deg(data.qpos[f1_tip_id])
        f2_tip = np.rad2deg(data.qpos[f2_tip_id])
        delta = abs(f1_palm - f2_palm)
        
        # Get tendon lengths
        t1_length = data.ten_length[t1_id]
        t2_length = data.ten_length[t2_id]
        avg_length = (t1_length + t2_length) / 2
        
        print(f"Joints: F1: {f1_palm:6.2f}° F2: {f2_palm:6.2f}° Δ: {delta:5.2f}°")
        print(f"Tendons: T1: {t1_length:6.4f} T2: {t2_length:6.4f} Avg: {avg_length:6.4f}")
        print(f"Status: {'✅ OPEN' if control == 0.0 else '✅ CLOSING' if control < 1.0 else '✅ FULLY CLOSED'}")
    
    print("\n" + "="*70)
    print("TENDON RANGE ANALYSIS")
    print("="*70)
    
    # Find min and max tendon lengths
    print("\nRe-running to find exact range...")
    
    # Test full range
    for control in [0.0, 1.0]:
        print(f"\nSetting control to {control:.1f}")
        data.ctrl[act_id] = control
        
        # Let it settle
        for step in range(1000):
            mujoco.mj_step(model, data)
        
        t1_length = data.ten_length[t1_id]
        t2_length = data.ten_length[t2_id]
        avg_length = (t1_length + t2_length) / 2
        
        f1_palm = np.rad2deg(data.qpos[f1_palm_id])
        f2_palm = np.rad2deg(data.qpos[f2_palm_id])
        
        if control == 0.0:
            print(f"OPEN:  Control={control:.1f}, Tendon={avg_length:.6f}, Joints=({f1_palm:.1f}°, {f2_palm:.1f}°)")
        else:
            print(f"CLOSED: Control={control:.1f}, Tendon={avg_length:.6f}, Joints=({f1_palm:.1f}°, {f2_palm:.1f}°)")
    
    print("\n" + "="*70)
    print("TENDON TRACKING COMPLETE")
    print("="*70)

if __name__ == "__main__":
    main()
