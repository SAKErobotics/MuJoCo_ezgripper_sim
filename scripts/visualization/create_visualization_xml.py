#!/usr/bin/env python3
"""
Create a modified XML with visualization markers for mechanical stops.
"""

import mujoco
import numpy as np
import os
import xml.etree.ElementTree as ET

# Load original XML
xml_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
tree = ET.parse(xml_path)
root = tree.getroot()

print("="*80)
print("CREATING VISUALIZATION XML WITH STOP MARKERS")
print("="*80)

# Load model to calculate positions
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Calculate correct positions
f1_palm_id = model.joint('F1_palm_knuckle').id
palm_lower_stop_pos = model.geom('palm_stop_f1_lower').pos
palm_upper_stop_pos = model.geom('palm_stop_f1_upper').pos

data.qpos[f1_palm_id] = np.radians(-90)
mujoco.mj_kinematics(model, data)
finger_transform_lower = data.xpos[model.body('F1_L1').id]
correct_lower_relative = palm_lower_stop_pos - finger_transform_lower

data.qpos[f1_palm_id] = np.radians(25)
mujoco.mj_kinematics(model, data)
finger_transform_upper = data.xpos[model.body('F1_L1').id]
correct_upper_relative = palm_upper_stop_pos - finger_transform_upper

# Find the worldbody element
worldbody = root.find('worldbody')
mount_body = None
for body in worldbody.findall('body'):
    if body.get('name') == 'mount':
        mount_body = body
        break

if mount_body is not None:
    # Add visualization markers to mount body (so they're fixed in world coordinates)
    
    # Palm stop markers (BLUE)
    palm_lower_marker = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_palm_lower',
        'type': 'sphere',
        'size': '0.003',
        'pos': f"{palm_lower_stop_pos[0]} {palm_lower_stop_pos[1]} {palm_lower_stop_pos[2]}",
        'contype': '0',
        'conaffinity': '0',
        'rgba': '0 0 1 1'  # Blue
    })
    
    palm_upper_marker = ET.SubElement(mount_body, 'geom', {
        'name': 'viz_palm_upper',
        'type': 'sphere',
        'size': '0.003',
        'pos': f"{palm_upper_stop_pos[0]} {palm_upper_stop_pos[1]} {palm_upper_stop_pos[2]}",
        'contype': '0',
        'conaffinity': '0',
        'rgba': '0 0 1 1'  # Blue
    })
    
    # Find F1_L1 body to add finger stop markers
    f1_l1_body = None
    for body in mount_body.findall('.//body'):
        if body.get('name') == 'F1_L1':
            f1_l1_body = body
            break
    
    if f1_l1_body is not None:
        # Current stop markers (RED)
        current_lower_marker = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_current_lower',
            'type': 'sphere',
            'size': '0.003',
            'pos': '0.01 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 1'  # Red
        })
        
        current_upper_marker = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_current_upper',
            'type': 'sphere',
            'size': '0.003',
            'pos': '-0.01 0 0',
            'contype': '0',
            'conaffinity': '0',
            'rgba': '1 0 0 1'  # Red
        })
        
        # Correct stop markers (GREEN)
        correct_lower_marker = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_correct_lower',
            'type': 'sphere',
            'size': '0.003',
            'pos': f"{correct_lower_relative[0]} {correct_lower_relative[1]} {correct_lower_relative[2]}",
            'contype': '0',
            'conaffinity': '0',
            'rgba': '0 1 0 1'  # Green
        })
        
        correct_upper_marker = ET.SubElement(f1_l1_body, 'geom', {
            'name': 'viz_correct_upper',
            'type': 'sphere',
            'size': '0.003',
            'pos': f"{correct_upper_relative[0]} {correct_upper_relative[1]} {correct_upper_relative[2]}",
            'contype': '0',
            'conaffinity': '0',
            'rgba': '0 1 0 1'  # Green
        })

# Save modified XML
viz_xml_path = os.path.join(os.path.dirname(__file__), 'ezgripper_visualization.xml')
tree.write(viz_xml_path, encoding='utf-8', xml_declaration=True)

print(f"\nVisualization XML created: {viz_xml_path}")
print("\nMarker colors:")
print("  BLUE spheres: Palm stop positions (fixed)")
print("  RED spheres: Current (wrong) finger stop positions")
print("  GREEN spheres: Correct finger stop positions")

print(f"\nPosition summary:")
print(f"  Palm lower: [{palm_lower_stop_pos[0]:.3f}, {palm_lower_stop_pos[1]:.3f}, {palm_lower_stop_pos[2]:.3f}]")
print(f"  Palm upper: [{palm_upper_stop_pos[0]:.3f}, {palm_upper_stop_pos[1]:.3f}, {palm_upper_stop_pos[2]:.3f}]")
print(f"  Current lower: [0.010, 0.000, 0.000] (relative to finger)")
print(f"  Current upper: [-0.010, 0.000, 0.000] (relative to finger)")
print(f"  Correct lower: [{correct_lower_relative[0]:.3f}, {correct_lower_relative[1]:.3f}, {correct_lower_relative[2]:.3f}]")
print(f"  Correct upper: [{correct_upper_relative[0]:.3f}, {correct_upper_relative[1]:.3f}, {correct_upper_relative[2]:.3f}]")

print(f"\nNow run: python3 visualize_with_markers.py")
