#!/usr/bin/env python3
"""
Full range of motion test:
1. Open to -90° limit (record tendon length)
2. Close until fingers collide
3. Show full range
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("FULL RANGE OF MOTION TEST")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    t2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')
    
    print("\nPhysics:")
    print("  -90° = OPEN (fully open, at limit)")
    print("  +25° = CLOSED (fingers together)")
    print("\nTest sequence:")
    print("  1. Open to -90° limit")
    print("  2. Record tendon length")
    print("  3. Reverse and close until collision")
    
    # Phase 1: Open to limit
    print("\n" + "="*70)
    print("PHASE 1: Opening to -90° limit")
    print("="*70)
    
    mujoco.mj_resetData(model, data)
    data.qpos[f1_palm_id] = 0.0  # Start neutral
    data.qpos[f2_palm_id] = 0.0
    data.ctrl[act_id] = -2.0  # Negative = open
    
    for i in range(3000):
        mujoco.mj_step(model, data)
        
        if i % 500 == 0:
            f1 = np.rad2deg(data.qpos[f1_palm_id])
            f2 = np.rad2deg(data.qpos[f2_palm_id])
            t1_len = data.ten_length[t1_id]
            t2_len = data.ten_length[t2_id]
            
            print(f"Step {i:4d}: F1={f1:7.2f}° F2={f2:7.2f}° | "
                  f"T1 len={t1_len:.6f} T2 len={t2_len:.6f} | "
                  f"Contacts={data.ncon}")
    
    # Record open position
    open_f1 = np.rad2deg(data.qpos[f1_palm_id])
    open_f2 = np.rad2deg(data.qpos[f2_palm_id])
    open_t1 = data.ten_length[t1_id]
    open_t2 = data.ten_length[t2_id]
    
    print(f"\n✅ OPEN POSITION REACHED:")
    print(f"   F1 angle: {open_f1:.2f}°")
    print(f"   F2 angle: {open_f2:.2f}°")
    print(f"   T1 length: {open_t1:.6f} m")
    print(f"   T2 length: {open_t2:.6f} m")
    
    # Phase 2: Close until collision
    print("\n" + "="*70)
    print("PHASE 2: Closing until finger collision")
    print("="*70)
    
    data.ctrl[act_id] = 2.0  # Positive = close
    
    prev_contacts = 0
    collision_detected = False
    
    for i in range(5000):
        mujoco.mj_step(model, data)
        
        # Check for finger tip collision
        tip_collision = False
        for j in range(data.ncon):
            contact = data.contact[j]
            g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            if (g1 == 'f1_tip' and g2 == 'f2_tip') or (g1 == 'f2_tip' and g2 == 'f1_tip'):
                tip_collision = True
                if not collision_detected:
                    collision_detected = True
                    print(f"\n🎯 FINGER COLLISION DETECTED at step {i}!")
        
        if i % 500 == 0 or (tip_collision and i % 100 == 0):
            f1 = np.rad2deg(data.qpos[f1_palm_id])
            f2 = np.rad2deg(data.qpos[f2_palm_id])
            f1_tip = np.rad2deg(data.qpos[f1_tip_id])
            f2_tip = np.rad2deg(data.qpos[f2_tip_id])
            t1_len = data.ten_length[t1_id]
            t2_len = data.ten_length[t2_id]
            
            marker = "🎯" if tip_collision else "  "
            print(f"{marker} Step {i:4d}: F1={f1:7.2f}° F2={f2:7.2f}° | "
                  f"F1tip={f1_tip:6.2f}° F2tip={f2_tip:6.2f}° | "
                  f"T1={t1_len:.6f} T2={t2_len:.6f} | "
                  f"Contacts={data.ncon}")
        
        # Stop if collision persists
        if collision_detected and i > 3000:
            break
    
    # Record closed position
    closed_f1 = np.rad2deg(data.qpos[f1_palm_id])
    closed_f2 = np.rad2deg(data.qpos[f2_palm_id])
    closed_t1 = data.ten_length[t1_id]
    closed_t2 = data.ten_length[t2_id]
    
    print(f"\n✅ CLOSED POSITION REACHED:")
    print(f"   F1 angle: {closed_f1:.2f}°")
    print(f"   F2 angle: {closed_f2:.2f}°")
    print(f"   T1 length: {closed_t1:.6f} m")
    print(f"   T2 length: {closed_t2:.6f} m")
    
    # Summary
    print("\n" + "="*70)
    print("RANGE OF MOTION SUMMARY")
    print("="*70)
    print(f"\nOPEN (-90° limit):")
    print(f"  Angles: F1={open_f1:.2f}° F2={open_f2:.2f}°")
    print(f"  Tendon lengths: T1={open_t1:.6f}m T2={open_t2:.6f}m")
    print(f"\nCLOSED (finger collision):")
    print(f"  Angles: F1={closed_f1:.2f}° F2={closed_f2:.2f}°")
    print(f"  Tendon lengths: T1={closed_t1:.6f}m T2={closed_t2:.6f}m")
    print(f"\nRANGE:")
    print(f"  Angular range: {closed_f1 - open_f1:.2f}° ({open_f1:.1f}° to {closed_f1:.1f}°)")
    print(f"  Tendon travel: {abs(closed_t1 - open_t1):.6f}m ({open_t1:.6f}m to {closed_t1:.6f}m)")
    print(f"  Symmetric: F1-F2 diff = {abs(closed_f1 - closed_f2):.3f}° ✅" if abs(closed_f1 - closed_f2) < 1.0 else f"  Asymmetric: F1-F2 diff = {abs(closed_f1 - closed_f2):.3f}° ❌")
    
    # Interactive visualization
    print("\n" + "="*70)
    print("INTERACTIVE VISUALIZATION")
    print("="*70)
    print("Cycling between OPEN and CLOSED positions...")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        step = 0
        phase_duration = 3000
        
        while viewer.is_running():
            step_start = time.time()
            
            phase = (step // phase_duration) % 2
            
            if phase == 0:
                # OPEN
                data.ctrl[act_id] = -2.0
                phase_name = "OPENING"
            else:
                # CLOSE
                data.ctrl[act_id] = 2.0
                phase_name = "CLOSING"
            
            mujoco.mj_step(model, data)
            
            if step % 200 == 0:
                f1 = np.rad2deg(data.qpos[f1_palm_id])
                f2 = np.rad2deg(data.qpos[f2_palm_id])
                
                print(f"[{step:5d}] {phase_name:8s} | "
                      f"F1:{f1:7.2f}° F2:{f2:7.2f}° | "
                      f"Diff:{abs(f1-f2):5.2f}° | "
                      f"Ctrl:{data.ctrl[act_id]:6.2f} | "
                      f"Contacts:{data.ncon}")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
