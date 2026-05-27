#!/usr/bin/env python3
"""
Visualize EZGripper range of motion with and without object.
Shows gripper closing sequence from open to fully closed.
"""

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import os

def render_gripper_state(model, data, camera_config, title):
    """Render gripper at current state"""
    renderer = mujoco.Renderer(model, height=480, width=640)
    
    scene_option = mujoco.MjvOption()
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    
    camera = mujoco.MjvCamera()
    camera.distance = camera_config['distance']
    camera.azimuth = camera_config['azimuth']
    camera.elevation = camera_config['elevation']
    camera.lookat[:] = camera_config['lookat']
    
    renderer.update_scene(data, camera, scene_option)
    pixels = renderer.render()
    
    # Add title
    img = Image.fromarray(pixels)
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 18)
    except:
        font = ImageFont.load_default()
    
    # Draw title with background
    text_bbox = draw.textbbox((0, 0), title, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    draw.rectangle([(5, 5), (text_width + 15, text_height + 15)], fill=(0, 0, 0, 200))
    draw.text((10, 10), title, fill=(255, 255, 0), font=font)
    
    renderer.close()
    return img

def simulate_closing_sequence(has_object=False):
    """Simulate gripper closing from open to closed"""
    
    # Load model
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    
    # Get actuator ID
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    # Get joint IDs for monitoring
    l1_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    l2_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    
    # Define closing sequence: tendon force levels
    force_levels = [0, 25, 50, 75, 100]  # Progressive closing
    
    images_top = []
    images_side = []
    
    object_str = "with_object" if has_object else "no_object"
    
    for i, force in enumerate(force_levels):
        # Set control
        data.ctrl[actuator_id] = force
        
        # Let physics settle
        for _ in range(1000):
            mujoco.mj_step(model, data)
        
        # Get joint angles
        l1_angle = np.degrees(data.qpos[l1_joint_id])
        l2_angle = np.degrees(data.qpos[l2_joint_id])
        
        # Render top view
        camera_top = {
            'distance': 0.2,
            'azimuth': 90,
            'elevation': -85,
            'lookat': [0.12, 0, 0.02]
        }
        
        title_top = f"Force={force}% | L1={l1_angle:.1f}° L2={l2_angle:.1f}°"
        img_top = render_gripper_state(model, data, camera_top, title_top)
        images_top.append(img_top)
        
        # Render side view
        camera_side = {
            'distance': 0.25,
            'azimuth': 90,
            'elevation': -10,
            'lookat': [0.10, 0, 0.03]
        }
        
        title_side = f"Force={force}% | L1={l1_angle:.1f}° L2={l2_angle:.1f}°"
        img_side = render_gripper_state(model, data, camera_side, title_side)
        images_side.append(img_side)
        
        print(f"  Force {force}%: L1={l1_angle:.1f}°, L2={l2_angle:.1f}°")
    
    # Create composite images
    # Top view sequence
    width = images_top[0].width
    height = images_top[0].height
    composite_top = Image.new('RGB', (width * len(images_top), height))
    for i, img in enumerate(images_top):
        composite_top.paste(img, (i * width, 0))
    
    # Side view sequence
    composite_side = Image.new('RGB', (width * len(images_side), height))
    for i, img in enumerate(images_side):
        composite_side.paste(img, (i * width, 0))
    
    # Save
    filename_top = f'range_of_motion_{object_str}_top.png'
    filename_side = f'range_of_motion_{object_str}_side.png'
    
    composite_top.save(filename_top)
    composite_side.save(filename_side)
    
    print(f"  Saved: {filename_top}")
    print(f"  Saved: {filename_side}")
    
    return filename_top, filename_side

def main():
    print("=" * 70)
    print("EZGripper Range of Motion Visualization")
    print("=" * 70)
    
    # Test 1: Without object
    print("\n1. Gripper closing WITHOUT object (tips should meet)")
    print("-" * 70)
    simulate_closing_sequence(has_object=False)
    
    # Test 2: With object - need to enable cylinder in XML
    print("\n2. Gripper closing WITH cylinder object (should wrap around)")
    print("-" * 70)
    
    # Temporarily enable cylinder
    import xml.etree.ElementTree as ET
    tree = ET.parse('ezgripper.xml')
    root = tree.getroot()
    
    # Find and uncomment cylinder
    for body in root.iter('body'):
        if body.get('name') == 'grasp_cylinder':
            # Already exists but commented - need to enable it
            break
    
    # Check if cylinder is commented
    with open('ezgripper.xml', 'r') as f:
        xml_content = f.read()
    
    if '<!-- <body name="grasp_cylinder"' in xml_content:
        # Uncomment cylinder
        xml_content = xml_content.replace(
            '<!-- <body name="grasp_cylinder" pos="0.10 0 0.03">',
            '<body name="grasp_cylinder" pos="0.10 0 0.03">'
        )
        xml_content = xml_content.replace(
            '</body> -->',
            '</body>',
            1  # Only replace first occurrence
        )
        
        with open('ezgripper.xml', 'w') as f:
            f.write(xml_content)
        
        print("  Enabled cylinder object in XML")
        
        # Run simulation with object
        simulate_closing_sequence(has_object=True)
        
        # Re-comment cylinder
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
        
        print("  Restored XML (cylinder commented)")
    else:
        print("  Cylinder already enabled, running simulation...")
        simulate_closing_sequence(has_object=True)
    
    print("\n" + "=" * 70)
    print("VISUALIZATION COMPLETE")
    print("=" * 70)
    print("\nGenerated files:")
    print("  - range_of_motion_no_object_top.png")
    print("  - range_of_motion_no_object_side.png")
    print("  - range_of_motion_with_object_top.png")
    print("  - range_of_motion_with_object_side.png")
    print("\nEach image shows 5 stages of closing from 0% to 100% tendon force")

if __name__ == '__main__':
    main()
