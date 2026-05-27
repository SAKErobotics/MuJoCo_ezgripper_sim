#!/usr/bin/env python3
"""
Gradually increase control until fingers touch.
Start with small control and slowly ramp up.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("GRADUAL CLOSING TEST")
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
    
    print("\nFull range of motion test:")
    print("Phase 1: Open to -90° (negative control)")
    print("Phase 2: Close to contact (positive control)")
    print("Springs push OPEN (toward -90°), Tendon pulls CLOSE (toward +25°)")
    print("Monitoring for finger tip collision")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        mujoco.mj_resetData(model, data)
        
        step = 0
        phase = "OPENING"
        control = -0.5  # Start with negative control to help springs open
        collision_detected = False
        open_complete = False
        
        while viewer.is_running() and step < 30000:
            step_start = time.time()
            
            # Phase 1: Open to -90° (steps 0-5000)
            if phase == "OPENING" and step < 5000:
                # Hold negative control to let springs open fully
                data.ctrl[act_id] = -0.5
                if step == 4999:
                    phase = "CLOSING"
                    control = 0.0
                    print("\n" + "="*70)
                    print("PHASE 2: CLOSING - Gradually increasing control")
                    print("="*70)
            
            # Phase 2: Gradually close (steps 5000+)
            elif phase == "CLOSING":
                if step % 100 == 0 and control < 1.0:
                    control += 0.01
                data.ctrl[act_id] = control
            
            mujoco.mj_step(model, data)
            
            # Check for finger tip collision
            for j in range(data.ncon):
                contact = data.contact[j]
                g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                
                if (g1 == 'f1_tip' and g2 == 'f2_tip') or (g1 == 'f2_tip' and g2 == 'f1_tip'):
                    if not collision_detected:
                        collision_detected = True
                        f1_palm = np.rad2deg(data.qpos[f1_palm_id])
                        f2_palm = np.rad2deg(data.qpos[f2_palm_id])
                        print(f"\n🎯 FINGER TIPS TOUCHED at step {step}!")
                        print(f"   Control: {control:.3f}")
                        print(f"   F1 palm: {f1_palm:.2f}°")
                        print(f"   F2 palm: {f2_palm:.2f}°")
                        print(f"   Difference: {abs(f1_palm - f2_palm):.2f}°")
            
            if step % 200 == 0:
                f1_palm = np.rad2deg(data.qpos[f1_palm_id])
                f2_palm = np.rad2deg(data.qpos[f2_palm_id])
                f1_tip = np.rad2deg(data.qpos[f1_tip_id])
                f2_tip = np.rad2deg(data.qpos[f2_tip_id])
                diff = abs(f1_palm - f2_palm)
                tendon_len = data.ten_length[t1_id]
                
                sym_status = "✅" if diff < 2.0 else "❌"
                collision_marker = "🎯" if collision_detected else "  "
                phase_marker = "OPEN" if phase == "OPENING" else "CLOSE"
                
                print(f"{collision_marker} [{step:5d}] {phase_marker:5s} Ctrl:{control:6.3f} | "
                      f"PALM: F1:{f1_palm:7.2f}° F2:{f2_palm:7.2f}° Δ:{diff:5.2f}° {sym_status} | "
                      f"TIP: F1:{f1_tip:6.2f}° F2:{f2_tip:6.2f}° | "
                      f"Tendon:{tendon_len:.6f}m | Contacts:{data.ncon}")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)
        
        if not collision_detected:
            print(f"\n⚠️  No collision detected after {step} steps")
            print(f"   Final control: {control:.3f}")

if __name__ == '__main__':
    main()
