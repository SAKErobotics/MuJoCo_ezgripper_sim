#!/usr/bin/env python3
"""
Test grasping a movable cylinder - TENDON VIEW ONLY.
Shows only tendons, sites, and collision geoms - no visual meshes.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("CYLINDER GRASPING TEST - TENDON VIEW")
    print("="*70)
    print("Cylinder: 2\" diameter (0.0508m), 15cm tall")
    print("Visualization: Tendons and collision geoms only (no STL meshes)")
    
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
    
    # Get IDs
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    print("\nTest sequence:")
    print("1. Open gripper fully (2 seconds)")
    print("2. Close and wrap around cylinder (10 seconds, force up to 2.0)")
    print("3. Hold grasp and observe cylinder position")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Camera setup
        viewer.cam.distance = 0.4
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -15
        
        # HIDE VISUAL MESHES - Show only tendons and collision geoms
        viewer._opt.geomgroup[0] = False  # Hide visual geoms (group 0)
        viewer._opt.geomgroup[1] = True   # Show sites/tendons (group 1)
        viewer._opt.geomgroup[2] = True   # Show collision geoms
        viewer._opt.geomgroup[3] = True   # Show all other geoms
        
        mujoco.mj_resetData(model, data)
        
        step = 0
        phase = "OPENING"
        control = -0.5
        
        while viewer.is_running() and step < 30000:
            # Phase 1: Open (0-4000 steps = 2 seconds)
            if phase == "OPENING" and step < 4000:
                data.ctrl[act_id] = control
                if step == 3999:
                    phase = "CLOSING"
                    control = 0.0
                    print("\n" + "="*70)
                    print("PHASE 2: CLOSING - Grasping cylinder")
                    print("="*70)
            
            # Phase 2: Close (4000-24000 steps = 10 seconds)
            elif phase == "CLOSING" and step < 24000:
                if step % 50 == 0 and control < 2.0:
                    control += 0.01
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
            
            # Print status every 400 steps
            if step % 400 == 0:
                f1_palm_deg = np.rad2deg(data.qpos[f1_palm_id])
                f2_palm_deg = np.rad2deg(data.qpos[f2_palm_id])
                cyl_pos = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cylinder')]
                
                print(f"   [{step:5d}] {phase[:5]:5s} Ctrl:{control:6.3f} | "
                      f"PALM: F1:{f1_palm_deg:6.2f}° F2:{f2_palm_deg:6.2f}° | "
                      f"CYL: X:{cyl_pos[0]:.4f} Z:{cyl_pos[2]:.4f}")
            
            viewer.sync()
            step += 1
    
    print("\n" + "="*70)
    print("Test complete!")
    print("="*70)

if __name__ == '__main__':
    main()
