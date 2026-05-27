#!/usr/bin/env python3
"""
Simple test to verify tendon actuator is working.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("SIMPLE TENDON ACTUATOR TEST")
    print("="*70)
    
    # Load model
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    print(f"\nLoading: {model_path}")
    
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print("✅ Model loaded")
    
    # Get IDs
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    # Get tendon IDs
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    t2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')
    
    print(f"\nJoint IDs: F1={f1_palm_id}, F2={f2_palm_id}")
    print(f"Actuator ID: {act_id}")
    print(f"Tendon IDs: T1={t1_id}, T2={t2_id}")
    print(f"Equality constraints: {model.neq}")
    
    # Reset
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    
    print(f"\nInitial state:")
    print(f"  F1 angle: {np.rad2deg(data.qpos[f1_palm_id]):.2f}°")
    print(f"  F2 angle: {np.rad2deg(data.qpos[f2_palm_id]):.2f}°")
    print(f"  T1 length: {data.ten_length[t1_id]:.6f}")
    print(f"  T2 length: {data.ten_length[t2_id]:.6f}")
    print(f"  Control: {data.ctrl[act_id]:.2f}")
    
    # Test 1: Apply positive control
    print("\n" + "="*70)
    print("TEST 1: Apply positive control (close)")
    print("="*70)
    
    data.ctrl[act_id] = 1.0
    
    for i in range(1000):
        mujoco.mj_step(model, data)
        
        if i % 200 == 0:
            print(f"\nStep {i}:")
            print(f"  F1: {np.rad2deg(data.qpos[f1_palm_id]):7.2f}°  "
                  f"F2: {np.rad2deg(data.qpos[f2_palm_id]):7.2f}°")
            print(f"  T1 len: {data.ten_length[t1_id]:.6f}  "
                  f"T2 len: {data.ten_length[t2_id]:.6f}")
            print(f"  Ctrl: {data.ctrl[act_id]:.2f}")
    
    # Test 2: Apply negative control  
    print("\n" + "="*70)
    print("TEST 2: Apply negative control (open)")
    print("="*70)
    
    data.ctrl[act_id] = -1.0
    
    for i in range(1000):
        mujoco.mj_step(model, data)
        
        if i % 200 == 0:
            print(f"\nStep {i}:")
            print(f"  F1: {np.rad2deg(data.qpos[f1_palm_id]):7.2f}°  "
                  f"F2: {np.rad2deg(data.qpos[f2_palm_id]):7.2f}°")
            print(f"  T1 len: {data.ten_length[t1_id]:.6f}  "
                  f"T2 len: {data.ten_length[t2_id]:.6f}")
            print(f"  Ctrl: {data.ctrl[act_id]:.2f}")
    
    # Interactive viewer
    print("\n" + "="*70)
    print("INTERACTIVE VIEWER - Manual control")
    print("="*70)
    print("Watch the gripper - it should oscillate open/close")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        step = 0
        while viewer.is_running():
            step_start = time.time()
            
            # Oscillate control
            t = step * model.opt.timestep
            data.ctrl[act_id] = 1.5 * np.sin(2 * np.pi * 0.2 * t)  # 0.2 Hz
            
            mujoco.mj_step(model, data)
            
            if step % 100 == 0:
                f1 = np.rad2deg(data.qpos[f1_palm_id])
                f2 = np.rad2deg(data.qpos[f2_palm_id])
                print(f"[{step:5d}] F1:{f1:7.2f}° F2:{f2:7.2f}° Ctrl:{data.ctrl[act_id]:6.2f}")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
