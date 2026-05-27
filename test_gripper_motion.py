#!/usr/bin/env python3
"""
Correct gripper motion test with proper control direction.

GRIPPER PHYSICS:
- Negative angles = CLOSED (fingers rotated inward)
- Positive angles = OPEN (fingers rotated outward)  
- Positive tendon force = PULL = CLOSE (more negative angles)
- Negative tendon force = RELEASE = OPEN (more positive angles)

Joint range: -90° (fully closed) to +25° (fully open)
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("EZGRIPPER MOTION TEST - Corrected Control Direction")
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
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    print(f"\nControl Direction:")
    print(f"  Positive control (+) = PULL tendon = CLOSE gripper (more negative angles)")
    print(f"  Negative control (-) = RELEASE tendon = OPEN gripper (more positive angles)")
    print(f"\nJoint Range: -90° (closed) to +25° (open)")
    
    # Reset to mid-position
    mujoco.mj_resetData(model, data)
    data.qpos[f1_palm_id] = np.deg2rad(-30)  # Start partially closed
    data.qpos[f2_palm_id] = np.deg2rad(-30)
    mujoco.mj_forward(model, data)
    
    print(f"\nStarting position: -30° (partially closed)")
    
    # Interactive viewer with clear motion
    print("\n" + "="*70)
    print("INTERACTIVE VIEWER")
    print("="*70)
    print("\nThe gripper will cycle through:")
    print("  1. OPEN (ctrl = -1.5)")
    print("  2. CLOSE (ctrl = +1.5)")
    print("  3. Oscillate smoothly")
    print("\nWatch for SYMMETRIC motion of both fingers!")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        step = 0
        mode_duration = 2000  # 2 seconds per mode at 1ms timestep
        
        while viewer.is_running():
            step_start = time.time()
            
            # Cycle through modes
            mode = (step // mode_duration) % 3
            
            if mode == 0:
                # OPEN - negative control
                data.ctrl[act_id] = -1.5
                mode_name = "OPENING"
                
            elif mode == 1:
                # CLOSE - positive control
                data.ctrl[act_id] = 1.5
                mode_name = "CLOSING"
                
            else:
                # OSCILLATE - smooth sine wave
                t = (step % mode_duration) * model.opt.timestep
                data.ctrl[act_id] = 1.5 * np.sin(2 * np.pi * 0.5 * t)  # 0.5 Hz
                mode_name = "OSCILLATING"
            
            mujoco.mj_step(model, data)
            
            if step % 100 == 0:
                f1_palm = np.rad2deg(data.qpos[f1_palm_id])
                f2_palm = np.rad2deg(data.qpos[f2_palm_id])
                f1_tip = np.rad2deg(data.qpos[f1_tip_id])
                f2_tip = np.rad2deg(data.qpos[f2_tip_id])
                diff = abs(f1_palm - f2_palm)
                
                print(f"[{step:5d}] {mode_name:12s} | "
                      f"F1 palm:{f1_palm:7.2f}° F2 palm:{f2_palm:7.2f}° | "
                      f"Diff:{diff:5.2f}° | Ctrl:{data.ctrl[act_id]:6.2f}")
                
                # Also show tip angles
                if step % 500 == 0:
                    print(f"         Tip angles    | "
                          f"F1 tip:{f1_tip:7.2f}° F2 tip:{f2_tip:7.2f}°")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
