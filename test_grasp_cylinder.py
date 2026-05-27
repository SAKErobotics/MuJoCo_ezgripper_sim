#!/usr/bin/env python3
"""
Test grasping a movable cylinder.
Cylinder: 2.5" (0.0635m) diameter, free to move.
Gripper wraps around and dynamically adjusts the cylinder position.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("CYLINDER GRASPING TEST")
    print("="*70)
    print("Cylinder: 2\" diameter (0.0508m), 15cm tall")
    print("Standing upright on floor - free to move during grasp")
    
    # Load base model
    base_model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    
    # Read the XML and modify existing cylinder
    with open(base_model_path, 'r') as f:
        xml_content = f.read()
    
    # Modify the existing cylinder to be blue and free-moving
    xml_content = xml_content.replace(
        '        <body name="grasp_cylinder" pos="0.15 0 0.10">\n            <geom name="cylinder_geom" type="cylinder" size="0.0286 0.10" rgba="0.3 0.6 0.8 1" contype="1" conaffinity="1" friction="1.0 0.1 0.01"/>',
        '        <body name="grasp_cylinder" pos="0.12 0 0.075">\n            <freejoint/>\n            <geom name="cylinder_geom" type="cylinder" size="0.0254 0.075" rgba="0.1 0.3 0.8 1" mass="0.12" contype="1" conaffinity="1" friction="2.0 0.005 0.0001"/>'
    )
    
    # Load modified model
    model = mujoco.MjModel.from_xml_string(xml_content)
    data = mujoco.MjData(model)
    
    # Get IDs
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    cyl_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cylinder')
    
    print("\nTest sequence:")
    print("1. Open gripper fully (2 seconds)")
    print("2. Close and wrap around cylinder (10 seconds, force up to 2.0)")
    print("3. Hold grasp and observe cylinder position")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.4
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -15
        
        mujoco.mj_resetData(model, data)
        
        step = 0
        phase = "OPENING"
        control = -0.5
        grasp_detected = False
        
        while viewer.is_running() and step < 35000:
            step_start = time.time()
            
            # Phase 1: Open (0-4000 steps = 2 seconds)
            if phase == "OPENING" and step < 4000:
                data.ctrl[act_id] = -0.5
                if step == 3999:
                    phase = "CLOSING"
                    control = 0.0
                    print("\n" + "="*70)
                    print("PHASE 2: CLOSING - Grasping cylinder")
                    print("="*70)
            
            # Phase 2: Close (4000-24000 steps = 10 seconds, continue until max force)
            elif phase == "CLOSING" and step < 24000:
                if step % 50 == 0 and control < 2.0:
                    control += 0.01  # Continue increasing force
                data.ctrl[act_id] = control
                if step == 23999:
                    phase = "HOLDING"
                    print("\n" + "="*70)
                    print("PHASE 3: HOLDING - Maintaining grasp")
                    print(f"Final control: {control:.3f}")
                    print("="*70)
            
            # Phase 3: Hold grasp
            elif phase == "HOLDING":
                data.ctrl[act_id] = control
                if step == 29999:
                    phase = "REOPENING"
                    print("\n" + "="*70)
                    print("PHASE 4: REOPENING - Opening gripper")
                    print("="*70)
            
            # Phase 4: Reopen
            elif phase == "REOPENING" and step < 34000:
                if step % 50 == 0 and control > 0.0:
                    control -= 0.01  # Decrease force to open
                data.ctrl[act_id] = control
                if step == 33999:
                    phase = "SECOND_CLOSE"
                    print("\n" + "="*70)
                    print("PHASE 5: SECOND CLOSE - Grasping again")
                    print("="*70)
            
            # Phase 5: Second close
            elif phase == "SECOND_CLOSE" and step < 44000:
                if step % 50 == 0 and control < 2.0:
                    control += 0.01  # Increase force to close
                data.ctrl[act_id] = control
                if step == 43999:
                    phase = "FINAL_HOLD"
                    print("\n" + "="*70)
                    print("PHASE 6: FINAL HOLD - Maintaining second grasp")
                    print("="*70)
            
            # Phase 6: Final hold
            elif phase == "FINAL_HOLD":
                data.ctrl[act_id] = control
                if step == 47999:
                    phase = "FINAL_OPEN"
                    print("\n" + "="*70)
                    print("PHASE 7: FINAL OPEN - Opening completely")
                    print("="*70)
            
            # Phase 7: Final open
            elif phase == "FINAL_OPEN" and step < 52000:
                if step % 50 == 0 and control > 0.0:
                    control -= 0.01  # Decrease force to open
                data.ctrl[act_id] = control
            
            mujoco.mj_step(model, data)
            
            # Check for cylinder contact
            cylinder_contacts = 0
            for j in range(data.ncon):
                contact = data.contact[j]
                g1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                g2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                
                if g1 == 'cylinder_body' or g2 == 'cylinder_body':
                    cylinder_contacts += 1
                    if not grasp_detected and phase == "CLOSING":
                        grasp_detected = True
                        f1_palm = np.rad2deg(data.qpos[f1_palm_id])
                        f2_palm = np.rad2deg(data.qpos[f2_palm_id])
                        print(f"\n🎯 CYLINDER CONTACT at step {step}!")
                        print(f"   Control: {control:.3f}")
                        print(f"   F1 palm: {f1_palm:.2f}°")
                        print(f"   F2 palm: {f2_palm:.2f}°")
            
            if step % 400 == 0:
                f1_palm = np.rad2deg(data.qpos[f1_palm_id])
                f2_palm = np.rad2deg(data.qpos[f2_palm_id])
                f1_tip = np.rad2deg(data.qpos[f1_tip_id])
                f2_tip = np.rad2deg(data.qpos[f2_tip_id])
                diff = abs(f1_palm - f2_palm)
                tendon_len = data.ten_length[t1_id]
                
                # Get cylinder position
                cyl_pos = data.xpos[cyl_body_id]
                
                sym_status = "✅" if diff < 2.0 else "❌"
                grasp_marker = "🎯" if grasp_detected else "  "
                phase_marker = phase[:5].ljust(5)
                
                print(f"{grasp_marker} [{step:5d}] {phase_marker} Ctrl:{control:6.3f} | "
                      f"PALM: F1:{f1_palm:7.2f}° F2:{f2_palm:7.2f}° Δ:{diff:5.2f}° {sym_status} | "
                      f"CYL_POS: X:{cyl_pos[0]:.4f} Y:{cyl_pos[1]:.4f} Z:{cyl_pos[2]:.4f} | "
                      f"Contacts:{cylinder_contacts}")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)
        
        print("\n" + "="*70)
        print("TEST COMPLETE")
        print("="*70)

if __name__ == '__main__':
    main()
