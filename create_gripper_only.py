#!/usr/bin/env python3
"""
Create gripper-only model without cylinder
"""

def main():
    # Read the original model
    with open('/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml', 'r') as f:
        xml_content = f.read()
    
    # Remove the cylinder section
    lines = xml_content.split('\n')
    new_lines = []
    
    for line in lines:
        # Skip cylinder-related lines
        if 'grasp_cylinder' in line or 'cylinder_geom' in line or 'CYLINDER OBJECT' in line:
            continue
        new_lines.append(line)
    
    # Write gripper-only model
    with open('/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper_only.xml', 'w') as f:
        f.write('\n'.join(new_lines))
    
    print("✅ Created ezgripper_only.xml without cylinder")

if __name__ == "__main__":
    main()
