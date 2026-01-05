#!/usr/bin/env python3
"""
Interactive MuJoCo simulation: EZGripper closing WITH cylinder object.
Shows dynamic wrapping motion around object.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def run_simulation():
    """Run interactive simulation of gripper wrapping around cylinder"""
    
    # First, enable cylinder in XML
    with open('ezgripper.xml', 'r') as f:
        xml_content = f.read()
    
    # Check if cylinder is commented
    cylinder_commented = '<!-- <body name="grasp_cylinder"' in xml_content
    
    if cylinder_commented:
        # Uncomment cylinder
        xml_content = xml_content.replace(
            '<!-- <body name="grasp_cylinder" pos="0.10 0 0.03">',
            '<body name="grasp_cylinder" pos="0.10 0 0.03">'
        )
        # Find the closing </body> for grasp_cylinder and uncomment it
        lines = xml_content.split('\n')
        in_cylinder = False
        for i, line in enumerate(lines):
            if 'grasp_cylinder' in line and 'pos="0.10 0 0.03"' in line:
                in_cylinder = True
            if in_cylinder and '</body> -->' in line:
                lines[i] = line.replace('</body> -->', '</body>')
                break
        xml_content = '\n'.join(lines)
        
        with open('ezgripper.xml', 'w') as f:
            f.write(xml_content)
        
        print("Enabled cylinder object in XML")
    
    # Load model with cylinder
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    
    print("=" * 70)
    print("EZGripper Dynamic Simulation - WITH CYLINDER OBJECT")
    print("=" * 70)
    print("Watch the gripper wrap around the cylinder dynamically")
    print("Fingers should conform to cylindrical shape")
    print()
    print("Controls:")
    print("  - Simulation runs automatically")
    print("  - Drag mouse to rotate view")
    print("  - Scroll to zoom")
    print("  - Press Ctrl+C or close window to exit")
    print("=" * 70)
    
    # Simulation state
    start_time = None
    closing_duration = 3.0  # seconds to complete closing
    
    def controller(model, data):
        """Control gripper closing"""
        nonlocal start_time
        
        if start_time is None:
            start_time = data.time
        
        elapsed = data.time - start_time
        
        # Progressive closing over closing_duration seconds
        if elapsed < closing_duration:
            progress = elapsed / closing_duration
            tendon_force = 100.0 * progress  # 0 to 100
        else:
            tendon_force = 100.0  # Fully closed
        
        # Apply force
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
        data.ctrl[actuator_id] = tendon_force
        
        # Print status every 0.5 seconds
        if int(elapsed * 2) != int((elapsed - model.opt.timestep) * 2):
            l1_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
            l2_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
            l1_angle = np.degrees(data.qpos[l1_joint_id])
            l2_angle = np.degrees(data.qpos[l2_joint_id])
            
            # Count contacts with cylinder
            num_contacts = 0
            try:
                cylinder_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cylinder_geom')
                for i in range(data.ncon):
                    contact = data.contact[i]
                    if contact.geom1 == cylinder_geom_id or contact.geom2 == cylinder_geom_id:
                        num_contacts += 1
            except:
                pass
            
            print(f"t={elapsed:.1f}s | Force={tendon_force:.0f}% | L1={l1_angle:.1f}° L2={l2_angle:.1f}° | Contacts={num_contacts}")
    
    # Launch interactive viewer
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            # Set up camera
            viewer.cam.distance = 0.25
            viewer.cam.azimuth = 45
            viewer.cam.elevation = -20
            viewer.cam.lookat[:] = [0.10, 0, 0.03]
            
            print("\nSimulation started - watch the gripper wrap around cylinder!")
            
            # Run simulation
            while viewer.is_running():
                step_start = time.time()
                
                # Apply control
                controller(model, data)
                
                # Step physics
                mujoco.mj_step(model, data)
                
                # Update viewer
                viewer.sync()
                
                # Control frame rate
                time_until_next = model.opt.timestep - (time.time() - step_start)
                if time_until_next > 0:
                    time.sleep(time_until_next)
    finally:
        # Restore XML (re-comment cylinder)
        if cylinder_commented:
            with open('ezgripper.xml', 'r') as f:
                xml_content = f.read()
            
            xml_content = xml_content.replace(
                '<body name="grasp_cylinder" pos="0.10 0 0.03">',
                '<!-- <body name="grasp_cylinder" pos="0.10 0 0.03">'
            )
            # Find the closing </body> for grasp_cylinder and comment it
            lines = xml_content.split('\n')
            in_cylinder = False
            for i, line in enumerate(lines):
                if 'grasp_cylinder' in line and '<!--' in line:
                    in_cylinder = True
                if in_cylinder and '</body>' in line and '-->' not in line:
                    lines[i] = line.replace('</body>', '</body> -->')
                    break
            xml_content = '\n'.join(lines)
            
            with open('ezgripper.xml', 'w') as f:
                f.write(xml_content)
            
            print("\nRestored XML (cylinder commented)")

if __name__ == '__main__':
    try:
        run_simulation()
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
