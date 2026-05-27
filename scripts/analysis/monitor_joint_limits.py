#!/usr/bin/env python3
"""
Monitor all joint angles and flag limit violations during cylinder grasp test.
Helps debug model errors by showing when joints exceed their defined limits.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("JOINT LIMIT MONITORING - Cylinder Grasp Test")
    print("="*70)
    print("Cylinder: 2\" diameter (0.0508m), 15cm tall")
    print("Monitoring all joints for limit violations")
    print("")
    
    # Load base model
    base_model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    
    # Read the XML and add cylinder
    with open(base_model_path, 'r') as f:
        xml_content = f.read()
    
    # Insert cylinder before </worldbody>
    cylinder_xml = '''
        <!-- GRASPABLE CYLINDER: 2" diameter, 15cm tall, standing upright -->
        <body name="cylinder" pos="0.12 0 0.075">
            <freejoint/>
            <geom name="cylinder_body" type="cylinder" size="0.0254 0.075" 
                  rgba="0.8 0.3 0.1 1" mass="0.12"
                  contype="1" conaffinity="1" friction="2.0 0.005 0.0001"/>
        </body>
'''
    
    xml_content = xml_content.replace('</worldbody>', cylinder_xml + '\n    </worldbody>')
    
    # Load modified model
    model = mujoco.MjModel.from_xml_string(xml_content)
    data = mujoco.MjData(model)
    
    # Get joint IDs and their limits
    joints = {
        'F1_palm': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle'),
        'F2_palm': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle'),
        'F1_tip': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip'),
        'F2_tip': mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip'),
    }
    
    # Get joint limits from model
    joint_limits = {}
    for name, jid in joints.items():
        qpos_adr = model.jnt_qposadr[jid]
        limit_low = model.jnt_range[jid, 0]
        limit_high = model.jnt_range[jid, 1]
        joint_limits[name] = (limit_low, limit_high)
        print(f"{name:12s}: range [{np.rad2deg(limit_low):7.2f}° to {np.rad2deg(limit_high):7.2f}°]")
    
    print("")
    
    # Get other IDs
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    cyl_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cylinder')
    
    print("Test sequence:")
    print("1. Open gripper fully (2 seconds)")
    print("2. Close to grasp cylinder (10 seconds, force up to 2.0)")
    print("3. Hold grasp and observe")
    print("")
    print("⚠️  LIMIT VIOLATIONS will be flagged with >>> markers")
    print("")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.4
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -15
        
        mujoco.mj_resetData(model, data)
        
        step = 0
        phase = "OPENING"
        control = -0.5
        violation_count = 0
        
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
            
            mujoco.mj_step(model, data)
            
            # Check for cylinder contact
            cylinder_contacts = 0
            for i in range(data.ncon):
                contact = data.contact[i]
                geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                if geom1_name == 'cylinder_body' or geom2_name == 'cylinder_body':
                    cylinder_contacts += 1
            
            # Get cylinder position
            cyl_xpos = data.xpos[cyl_body_id]
            
            # Read joint angles and check limits
            joint_angles = {}
            violations = []
            for name, jid in joints.items():
                qpos_adr = model.jnt_qposadr[jid]
                angle = data.qpos[qpos_adr]
                joint_angles[name] = angle
                
                # Check if exceeds limits
                limit_low, limit_high = joint_limits[name]
                if angle < limit_low:
                    violations.append(f"{name} BELOW limit: {np.rad2deg(angle):.2f}° < {np.rad2deg(limit_low):.2f}°")
                elif angle > limit_high:
                    violations.append(f"{name} ABOVE limit: {np.rad2deg(angle):.2f}° > {np.rad2deg(limit_high):.2f}°")
            
            # Print every 400 steps (0.2 seconds)
            if step % 400 == 0:
                # Calculate angle difference
                palm_diff = abs(joint_angles['F1_palm'] - joint_angles['F2_palm'])
                tip_diff = abs(joint_angles['F1_tip'] - joint_angles['F2_tip'])
                
                # Format output
                phase_str = phase[:5]
                
                # Build status line
                status = f"[{step:5d}] {phase_str} Ctrl:{control:6.3f} | "
                status += f"PALM: F1:{np.rad2deg(joint_angles['F1_palm']):6.2f}° F2:{np.rad2deg(joint_angles['F2_palm']):6.2f}° Δ:{np.rad2deg(palm_diff):5.2f}° | "
                status += f"TIP: F1:{np.rad2deg(joint_angles['F1_tip']):6.2f}° F2:{np.rad2deg(joint_angles['F2_tip']):6.2f}° Δ:{np.rad2deg(tip_diff):5.2f}° | "
                status += f"CYL: X:{cyl_xpos[0]:.4f} Z:{cyl_xpos[2]:.4f} | Contacts:{cylinder_contacts}"
                
                # Check for violations
                if violations:
                    print(f">>> {status}")
                    for v in violations:
                        print(f"    ⚠️  {v}")
                    
                    # Show forces/torques at violation points and identify driver
                    print(f"    Forces & Drivers:")
                    for name, jid in joints.items():
                        qpos_adr = model.jnt_qposadr[jid]
                        angle = joint_angles[name]
                        limit_low, limit_high = joint_limits[name]
                        
                        # Get forces
                        qfrc_act = data.qfrc_actuator[qpos_adr] if qpos_adr < len(data.qfrc_actuator) else 0
                        qfrc_spring = data.qfrc_passive[qpos_adr] if qpos_adr < len(data.qfrc_passive) else 0
                        qfrc_constraint = data.qfrc_constraint[qpos_adr] if qpos_adr < len(data.qfrc_constraint) else 0
                        
                        # Check if this joint is violating
                        if angle < limit_low or angle > limit_high:
                            # Determine which force is driving the violation
                            if angle < limit_low:
                                # Below limit (opening too much)
                                # Negative torque opens, positive closes
                                driver = "SPRING" if qfrc_spring < 0 and abs(qfrc_spring) > abs(qfrc_act) else "TENDON"
                            else:
                                # Above limit (closing too much)
                                driver = "SPRING" if qfrc_spring > 0 and abs(qfrc_spring) > abs(qfrc_act) else "TENDON"
                            
                            # Check for physical stop contact (constraint force should be non-zero)
                            stop_status = "STOP ACTIVE" if abs(qfrc_constraint) > 0.001 else "⚠️ NO STOP!"
                            
                            print(f"      {name}: act={qfrc_act:7.3f} spr={qfrc_spring:7.3f} constraint={qfrc_constraint:7.3f} | Driver: {driver} | {stop_status}")
                    
                    # Show tendon forces (via actuator)
                    tendon_force = data.actuator_force[act_id] if act_id < len(data.actuator_force) else 0
                    print(f"    Tendon: actuator_force={tendon_force:.3f} N")
                    
                    # Check for mechanical stop collisions
                    stop_contacts = 0
                    for i in range(data.ncon):
                        contact = data.contact[i]
                        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                        if geom1_name and geom2_name:
                            if 'stop' in geom1_name.lower() or 'stop' in geom2_name.lower():
                                stop_contacts += 1
                    
                    if stop_contacts > 0:
                        print(f"    Physical stops: {stop_contacts} stop contacts detected ✓")
                    else:
                        print(f"    Physical stops: ⚠️  NO STOP CONTACTS - stops not working!")
                    
                    violation_count += len(violations)
                else:
                    print(status)
            
            # Sync viewer
            viewer.sync()
            
            # Maintain real-time
            elapsed = time.time() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)
            
            step += 1
    
    print("\n" + "="*70)
    print("SIMULATION COMPLETE")
    print("="*70)
    print(f"Total limit violations detected: {violation_count}")
    if violation_count > 0:
        print("⚠️  Model errors detected - joints exceeded defined limits!")
        print("    Review forces at violation points to debug the model.")
    else:
        print("✅ All joints stayed within defined limits")
    print("="*70)

if __name__ == '__main__':
    main()
