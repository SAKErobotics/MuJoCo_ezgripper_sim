#!/usr/bin/env python3
"""
Test motor (velocity) control with equality constraint.
Motor control: positive = close, negative = open
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("MOTOR CONTROL TEST")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    t2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')
    
    print("\nMotor control: velocity-based")
    print("  Positive control → pull tendon → close gripper")
    print("  Negative control → release tendon → open gripper")
    print("  Equality constraint enforces symmetric motion")
    
    print("\n" + "="*70)
    print("INTERACTIVE VISUALIZATION")
    print("="*70)
    print("\nCycling through:")
    print("  1. CLOSE (ctrl = +1.0)")
    print("  2. OPEN (ctrl = -1.0)")
    print("  3. HOLD (ctrl = 0.0)")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        mujoco.mj_resetData(model, data)
        
        step = 0
        phase_duration = 3000
        
        while viewer.is_running():
            step_start = time.time()
            
            phase = (step // phase_duration) % 3
            
            if phase == 0:
                # CLOSE
                data.ctrl[act_id] = 1.0
                phase_name = "CLOSING"
            elif phase == 1:
                # OPEN
                data.ctrl[act_id] = -1.0
                phase_name = "OPENING"
            else:
                # HOLD
                data.ctrl[act_id] = 0.0
                phase_name = "HOLDING"
            
            mujoco.mj_step(model, data)
            
            if step % 200 == 0:
                f1 = np.rad2deg(data.qpos[f1_palm_id])
                f2 = np.rad2deg(data.qpos[f2_palm_id])
                diff = abs(f1 - f2)
                t1_len = data.ten_length[t1_id]
                t2_len = data.ten_length[t2_id]
                t_diff = abs(t1_len - t2_len)
                
                sym_status = "✅ SYM" if diff < 2.0 else "❌ ASYM"
                
                print(f"[{step:5d}] {phase_name:8s} | "
                      f"F1:{f1:7.2f}° F2:{f2:7.2f}° Δ:{diff:5.2f}° {sym_status} | "
                      f"T1:{t1_len:.6f} T2:{t2_len:.6f} ΔT:{t_diff:.6f} | "
                      f"Ctrl:{data.ctrl[act_id]:5.2f}")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
