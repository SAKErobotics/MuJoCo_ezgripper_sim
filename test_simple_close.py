#!/usr/bin/env python3
"""
Simple test: Command gripper to close and see if it moves.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("SIMPLE CLOSE TEST")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    
    print("\nCommand: Close gripper (tendon length = 0.146m)")
    print("Expected: Fingers move from ~-90° toward +25°")
    
    # Start from neutral
    mujoco.mj_resetData(model, data)
    data.ctrl[act_id] = 0.146  # CLOSE command
    
    print("\n" + "="*70)
    print("Simulating...")
    print("="*70)
    
    for i in range(5000):
        mujoco.mj_step(model, data)
        
        if i % 500 == 0:
            f1 = np.rad2deg(data.qpos[f1_palm_id])
            f2 = np.rad2deg(data.qpos[f2_palm_id])
            tendon_len = data.ten_length[t1_id]
            
            print(f"Step {i:4d}: F1={f1:7.2f}° F2={f2:7.2f}° | "
                  f"Tendon={tendon_len:.6f}m | Target=0.146m | "
                  f"Contacts={data.ncon}")
    
    final_f1 = np.rad2deg(data.qpos[f1_palm_id])
    final_f2 = np.rad2deg(data.qpos[f2_palm_id])
    
    print("\n" + "="*70)
    print("RESULT")
    print("="*70)
    print(f"\nFinal position: F1={final_f1:.2f}° F2={final_f2:.2f}°")
    
    if final_f1 > -80:
        print(f"✅ SUCCESS! Fingers moved toward closed position!")
        print(f"   Moved {final_f1 - (-90):.2f}° from open position")
    else:
        print(f"❌ FAILED! Fingers stuck near open position (-90°)")
        print(f"   Only moved {final_f1 - (-90):.2f}°")
    
    # Viewer
    print("\n" + "="*70)
    print("INTERACTIVE VIEWER")
    print("="*70)
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        step = 0
        while viewer.is_running() and step < 10000:
            step_start = time.time()
            
            # Hold close command
            data.ctrl[act_id] = 0.146
            
            mujoco.mj_step(model, data)
            
            if step % 500 == 0:
                f1 = np.rad2deg(data.qpos[f1_palm_id])
                f2 = np.rad2deg(data.qpos[f2_palm_id])
                print(f"[{step:5d}] F1:{f1:7.2f}° F2:{f2:7.2f}° | Ctrl:0.146m")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
