#!/usr/bin/env python3
"""
Test L2 joint equilibrium position with zero tendon force.
This shows whether the L2 spring preload keeps the joint straight.
"""

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

def render_equilibrium(springref_value, filename):
    """Render gripper at equilibrium with given springref value"""
    
    # Load model
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    
    # Zero control - no tendon force
    data.ctrl[:] = 0
    
    # Let system settle to equilibrium
    for _ in range(2000):
        mujoco.mj_step(model, data)
    
    # Get joint angles
    l1_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    l2_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    
    l1_angle = data.qpos[l1_joint_id]
    l2_angle = data.qpos[l2_joint_id]
    
    print(f"\nSpringref: {springref_value} rad ({np.degrees(springref_value):.1f}°)")
    print(f"L1 equilibrium: {np.degrees(l1_angle):.2f}°")
    print(f"L2 equilibrium: {np.degrees(l2_angle):.2f}°")
    
    # Render top view
    renderer = mujoco.Renderer(model, height=480, width=640)
    
    scene_option = mujoco.MjvOption()
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
    
    camera = mujoco.MjvCamera()
    camera.distance = 0.2
    camera.azimuth = 90
    camera.elevation = -85
    camera.lookat[:] = [0.12, 0, 0.02]
    
    renderer.update_scene(data, camera, scene_option)
    pixels = renderer.render()
    
    # Add annotations
    img = Image.fromarray(pixels)
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 20)
        font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
    except:
        font = ImageFont.load_default()
        font_small = font
    
    # Title
    title = f"L2 Equilibrium Test - springref={np.degrees(springref_value):.1f}°"
    draw.rectangle([(10, 10), (600, 80)], fill=(0, 0, 0, 180))
    draw.text((20, 20), title, fill=(255, 255, 0), font=font)
    draw.text((20, 50), f"L2 angle: {np.degrees(l2_angle):.2f}° (should be ~0° if straight)", 
              fill=(255, 255, 255), font=font_small)
    
    img.save(filename)
    print(f"Saved: {filename}")
    
    return l2_angle

# Test current springref value
print("=" * 60)
print("L2 SPRING EQUILIBRIUM TEST")
print("=" * 60)
print("\nTesting with ZERO tendon force to see L2 equilibrium position")

model = mujoco.MjModel.from_xml_path('ezgripper.xml')
l2_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')

# Read current springref from XML
import xml.etree.ElementTree as ET
tree = ET.parse('ezgripper.xml')
root = tree.getroot()
for joint in root.iter('joint'):
    if joint.get('name') == 'F1_knuckle_tip':
        current_springref = float(joint.get('springref'))
        break

l2_angle = render_equilibrium(current_springref, 'l2_equilibrium_test.png')

print("\n" + "=" * 60)
print("RESULT:")
if abs(np.degrees(l2_angle)) < 1.0:
    print(f"✓ L2 is STRAIGHT (angle = {np.degrees(l2_angle):.2f}°)")
    print("  Spring preload is sufficient!")
else:
    print(f"✗ L2 is BENT (angle = {np.degrees(l2_angle):.2f}°)")
    print(f"  Need to increase spring preload by ~{abs(np.degrees(l2_angle)):.1f}° more")
print("=" * 60)
