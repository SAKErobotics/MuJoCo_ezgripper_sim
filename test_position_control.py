#!/usr/bin/env python3
"""
Test position control of tendon.
Control is now tendon LENGTH, not force.
Shorter tendon = fingers close toward +25°
Longer tendon = fingers open toward -90°
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("POSITION CONTROL TEST")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    
    print("\nPosition control: Control value = desired tendon length")
    print("  Shorter tendon → fingers close (toward +25°)")
    print("  Longer tendon → fingers open (toward -90°)")
    
    # Find natural tendon length
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    natural_len = data.ten_length[t1_id]
    
    print(f"\nNatural tendon length at 0°: {natural_len:.6f}m")
    
    # Test different positions
    print("\n" + "="*70)
    print("Testing different tendon lengths")
    print("="*70)
    
    test_lengths = [0.163, 0.160, 0.155, 0.150, 0.148, 0.146]
    
    for target_len in test_lengths:
        mujoco.mj_resetData(model, data)
        data.ctrl[act_id] = target_len
        
        # Simulate
        for _ in range(2000):
            mujoco.mj_step(model, data)
        
        f1 = np.rad2deg(data.qpos[f1_palm_id])
        f2 = np.rad2deg(data.qpos[f2_palm_id])
        actual_len = data.ten_length[t1_id]
        
        print(f"Target: {target_len:.6f}m → F1:{f1:7.2f}° F2:{f2:7.2f}° | "
              f"Actual len:{actual_len:.6f}m | Contacts:{data.ncon}")
    
    # Interactive visualization
    print("\n" + "="*70)
    print("INTERACTIVE VISUALIZATION")
    print("="*70)
    print("Cycling through tendon lengths (open ↔ closed)")
    
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
                # OPEN - longer tendon
                data.ctrl[act_id] = 0.163
                phase_name = "OPEN (long tendon)"
            else:
                # CLOSE - shorter tendon
                data.ctrl[act_id] = 0.146
                phase_name = "CLOSE (short tendon)"
            
            mujoco.mj_step(model, data)
            
            if step % 200 == 0:
                f1 = np.rad2deg(data.qpos[f1_palm_id])
                f2 = np.rad2deg(data.qpos[f2_palm_id])
                actual_len = data.ten_length[t1_id]
                
                print(f"[{step:5d}] {phase_name:22s} | "
                      f"F1:{f1:7.2f}° F2:{f2:7.2f}° | "
                      f"Len:{actual_len:.6f}m | "
                      f"Ctrl:{data.ctrl[act_id]:.6f} | "
                      f"Contacts:{data.ncon}")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
