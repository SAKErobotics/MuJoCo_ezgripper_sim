#!/usr/bin/env python3
"""
Simple force ramp test:
1. Start at -90° (open, at limit)
2. Slowly increase tendon force
3. Detect when fingers start moving in positive direction
4. Continue until finger tips meet
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("FORCE RAMP TEST")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    print("\nTest: Slowly increase force until fingers move toward +25° (closed)")
    print("Goal: Find minimum force to overcome springs and close gripper")
    
    # Start at open position
    mujoco.mj_resetData(model, data)
    data.qpos[f1_palm_id] = np.deg2rad(-90)
    data.qpos[f2_palm_id] = np.deg2rad(-90)
    data.ctrl[act_id] = 0.0
    mujoco.mj_forward(model, data)
    
    # Let it settle
    for _ in range(500):
        mujoco.mj_step(model, data)
    
    initial_angle = np.rad2deg(data.qpos[f1_palm_id])
    print(f"\nInitial position: F1={initial_angle:.2f}°")
    
    print("\n" + "="*70)
    print("RAMPING FORCE")
    print("="*70)
    
    force = 0.0
    force_increment = 0.1  # Increase by 0.1 per 100 steps
    movement_detected = False
    movement_threshold = 0.5  # degrees
    collision_detected = False
    
    step = 0
    max_steps = 50000
    
    while step < max_steps:
        # Slowly ramp up force
        if step % 100 == 0:
            force += force_increment
            data.ctrl[act_id] = force
        
        mujoco.mj_step(model, data)
        
        f1 = np.rad2deg(data.qpos[f1_palm_id])
        f2 = np.rad2deg(data.qpos[f2_palm_id])
        
        # Check for movement in positive direction
        if not movement_detected and (f1 - initial_angle) > movement_threshold:
            movement_detected = True
            print(f"\n🎯 MOVEMENT DETECTED at step {step}!")
            print(f"   Force: {force:.2f}")
            print(f"   F1 angle: {f1:.2f}° (moved {f1 - initial_angle:.2f}° from {initial_angle:.2f}°)")
            print(f"   F2 angle: {f2:.2f}°")
            print(f"\nContinuing to increase force until collision...")
        
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
                    print(f"\n🎯 FINGER TIP COLLISION at step {step}!")
                    print(f"   Force: {force:.2f}")
                    print(f"   F1 angle: {f1:.2f}°")
                    print(f"   F2 angle: {f2:.2f}°")
                    print(f"   Total travel: {f1 - initial_angle:.2f}°")
                    print(f"\nStopping force ramp.")
                    break
        
        if collision_detected:
            break
        
        # Print progress
        if step % 1000 == 0:
            status = "MOVING" if movement_detected else "WAITING"
            print(f"[{step:6d}] {status:8s} | Force:{force:6.2f} | "
                  f"F1:{f1:7.2f}° F2:{f2:7.2f}° | "
                  f"Travel:{f1-initial_angle:6.2f}° | Contacts:{data.ncon}")
        
        step += 1
    
    # Final summary
    final_f1 = np.rad2deg(data.qpos[f1_palm_id])
    final_f2 = np.rad2deg(data.qpos[f2_palm_id])
    
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print(f"\nInitial position: {initial_angle:.2f}°")
    print(f"Final position: F1={final_f1:.2f}° F2={final_f2:.2f}°")
    print(f"Total travel: {final_f1 - initial_angle:.2f}°")
    print(f"Final force: {force:.2f}")
    print(f"Movement detected: {'YES' if movement_detected else 'NO'}")
    print(f"Collision detected: {'YES' if collision_detected else 'NO'}")
    
    if not movement_detected:
        print(f"\n❌ No movement detected! Fingers stuck at {final_f1:.2f}°")
        print(f"   Maximum force applied: {force:.2f}")
        print(f"   This suggests the tendon force direction may be incorrect")
        print(f"   or the force is insufficient to overcome spring tension.")
    elif not collision_detected:
        print(f"\n⚠️  Movement detected but no collision!")
        print(f"   Fingers moved {final_f1 - initial_angle:.2f}° but didn't meet.")
    else:
        print(f"\n✅ SUCCESS! Fingers closed and collided!")
    
    # Interactive visualization
    print("\n" + "="*70)
    print("INTERACTIVE VISUALIZATION")
    print("="*70)
    print("Showing final closed position...")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        # Hold at final force
        data.ctrl[act_id] = force
        
        vstep = 0
        while viewer.is_running() and vstep < 5000:
            step_start = time.time()
            
            mujoco.mj_step(model, data)
            
            if vstep % 200 == 0:
                f1 = np.rad2deg(data.qpos[f1_palm_id])
                f2 = np.rad2deg(data.qpos[f2_palm_id])
                print(f"[{vstep:5d}] F1:{f1:7.2f}° F2:{f2:7.2f}° | "
                      f"Force:{force:6.2f} | Contacts:{data.ncon}")
            
            viewer.sync()
            vstep += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
