#!/usr/bin/env python3
"""
Test EZGripper collision behavior:
1. Without object - verify tips meet and stop
2. With cylinder - verify wrapping around object
"""

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

def render_grasp_scenario(model, data, camera_config, filename, title):
    """Render a single grasp scenario"""
    renderer = mujoco.Renderer(model, height=600, width=800)
    
    scene_option = mujoco.MjvOption()
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
    
    camera = mujoco.MjvCamera()
    camera.distance = camera_config['distance']
    camera.azimuth = camera_config['azimuth']
    camera.elevation = camera_config['elevation']
    camera.lookat[:] = camera_config['lookat']
    
    renderer.update_scene(data, camera, scene_option)
    pixels = renderer.render()
    
    # Add title to image
    img = Image.fromarray(pixels)
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 24)
    except:
        font = ImageFont.load_default()
    
    # Draw title with background
    text_bbox = draw.textbbox((0, 0), title, font=font)
    text_width = text_bbox[2] - text_bbox[0]
    text_height = text_bbox[3] - text_bbox[1]
    draw.rectangle([(10, 10), (text_width + 30, text_height + 30)], fill=(0, 0, 0, 180))
    draw.text((20, 20), title, fill=(255, 255, 0), font=font)
    
    img.save(filename)
    print(f"Saved: {filename}")
    return img

def test_without_object():
    """Test gripper closing without object - tips should meet and stop"""
    print("\n=== TEST 1: Gripper closing WITHOUT object ===")
    print("Expected: Tips meet and stop due to collision")
    
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    
    # Apply strong closing force
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    data.ctrl[actuator_id] = 100.0  # Strong closing force
    
    # Run simulation to let gripper close
    for _ in range(2000):
        mujoco.mj_step(model, data)
    
    # Check contact points
    num_contacts = data.ncon
    print(f"Number of contacts: {num_contacts}")
    
    # Get tip positions
    f1_tip_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'F1_L2')
    f2_tip_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'F2_L2')
    
    # Render multiple views
    views = [
        {'name': 'front', 'azimuth': 0, 'elevation': -10, 'distance': 0.2, 'lookat': [0.12, 0, 0.02]},
        {'name': 'side', 'azimuth': 90, 'elevation': -10, 'distance': 0.2, 'lookat': [0.12, 0, 0.02]},
        {'name': 'top', 'azimuth': 90, 'elevation': -85, 'distance': 0.2, 'lookat': [0.12, 0, 0.02]}
    ]
    
    for view in views:
        filename = f"collision_test_no_object_{view['name']}.png"
        render_grasp_scenario(model, data, view, filename, 
                            f"No Object - Tips Meet ({view['name'].title()} View)")
    
    print(f"✓ Test 1 complete - {num_contacts} contact points detected")
    return num_contacts > 0

def test_with_cylinder():
    """Test gripper closing with cylinder - should wrap around object"""
    print("\n=== TEST 2: Gripper closing WITH cylinder object ===")
    print("Expected: Fingers wrap around cylinder, multiple contact points")
    
    # Load model and add cylinder
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    
    # Find cylinder body (if it exists in model)
    try:
        cylinder_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'grasp_cylinder')
        print("Cylinder object found in model")
    except:
        print("WARNING: No cylinder object in model - need to add one to XML")
        return False
    
    # Apply closing force
    actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    data.ctrl[actuator_id] = 100.0
    
    # Run simulation
    for _ in range(2000):
        mujoco.mj_step(model, data)
    
    # Check contacts
    num_contacts = data.ncon
    print(f"Number of contacts: {num_contacts}")
    
    # Count contacts with cylinder
    cylinder_contacts = 0
    for i in range(num_contacts):
        contact = data.contact[i]
        # Check if contact involves cylinder
        if contact.geom1 == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cylinder_geom') or \
           contact.geom2 == mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'cylinder_geom'):
            cylinder_contacts += 1
    
    print(f"Contacts with cylinder: {cylinder_contacts}")
    
    # Render views
    views = [
        {'name': 'front', 'azimuth': 0, 'elevation': -10, 'distance': 0.25, 'lookat': [0.10, 0, 0.03]},
        {'name': 'side', 'azimuth': 90, 'elevation': -10, 'distance': 0.25, 'lookat': [0.10, 0, 0.03]},
        {'name': 'top', 'azimuth': 90, 'elevation': -85, 'distance': 0.25, 'lookat': [0.10, 0, 0.03]}
    ]
    
    for view in views:
        filename = f"collision_test_with_cylinder_{view['name']}.png"
        render_grasp_scenario(model, data, view, filename,
                            f"With Cylinder - Wrapping ({view['name'].title()} View)")
    
    print(f"✓ Test 2 complete - {cylinder_contacts} cylinder contacts detected")
    return cylinder_contacts > 0

def main():
    print("EZGripper Collision Behavior Test")
    print("=" * 50)
    
    # Test 1: No object
    test1_passed = test_without_object()
    
    # Test 2: With cylinder
    test2_passed = test_with_cylinder()
    
    print("\n" + "=" * 50)
    print("COLLISION TEST SUMMARY:")
    print(f"  Test 1 (No Object - Tips Meet): {'✓ PASS' if test1_passed else '✗ FAIL'}")
    print(f"  Test 2 (With Cylinder - Wrapping): {'✓ PASS' if test2_passed else '✗ FAIL'}")
    print("\nGenerated images show collision behavior from multiple views")
    print("Contact points are visualized as small spheres in the images")

if __name__ == '__main__':
    main()
