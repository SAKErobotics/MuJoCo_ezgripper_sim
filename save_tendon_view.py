#!/usr/bin/env python3
"""
Save images of tendon path at different stages of grasping.
"""

import mujoco
import numpy as np
from PIL import Image

def main():
    print("="*70)
    print("SAVING TENDON VIEW IMAGES")
    print("="*70)
    
    # Load base model
    base_model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    
    with open(base_model_path, 'r') as f:
        xml_content = f.read()
    
    # Add cylinder
    cylinder_xml = '''
        <body name="cylinder" pos="0.12 0 0.075">
            <freejoint/>
            <geom name="cylinder_body" type="cylinder" size="0.0254 0.075" 
                  rgba="0.8 0.3 0.1 1" mass="0.12"
                  contype="1" conaffinity="1" friction="2.0 0.005 0.0001"/>
        </body>
'''
    xml_content = xml_content.replace('</worldbody>', cylinder_xml + '\n    </worldbody>')
    
    model = mujoco.MjModel.from_xml_string(xml_content)
    data = mujoco.MjData(model)
    
    # Setup renderer
    renderer = mujoco.Renderer(model, height=480, width=640)
    
    # Configure visualization - hide visual meshes
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[0] = False  # Hide visual geoms
    scene_option.geomgroup[1] = True   # Show sites/tendons
    scene_option.geomgroup[2] = True   # Show collision geoms
    
    # Camera setup
    camera = mujoco.MjvCamera()
    camera.distance = 0.4
    camera.azimuth = 90
    camera.elevation = -15
    
    # Get IDs
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    # Save images at different stages
    stages = [
        (0, -0.5, "01_fully_open"),
        (2000, -0.5, "02_open_stable"),
        (4000, 0.0, "03_start_closing"),
        (6000, 0.4, "04_wrapping"),
        (10000, 1.0, "05_grasping"),
        (15000, 1.5, "06_tight_grasp"),
    ]
    
    for target_step, control, name in stages:
        mujoco.mj_resetData(model, data)
        
        # Simulate to target step
        for step in range(target_step + 1):
            if step < 4000:
                data.ctrl[act_id] = -0.5
            else:
                data.ctrl[act_id] = min(control, -0.5 + (step - 4000) * 0.01 / 50)
            mujoco.mj_step(model, data)
        
        # Render and save
        renderer.update_scene(data, camera, scene_option)
        pixels = renderer.render()
        
        img = Image.fromarray(pixels)
        filename = f"tendon_view_{name}.png"
        img.save(filename)
        print(f"Saved: {filename} (step {target_step}, control {control:.2f})")
    
    print("\n" + "="*70)
    print("All images saved!")
    print("="*70)

if __name__ == '__main__':
    main()
