#!/usr/bin/env python3
"""
Create proper gripper-only model without cylinder
"""

def main():
    # Read the original model
    with open('/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml', 'r') as f:
        xml_content = f.read()
    
    # Remove the cylinder section properly
    # Find the cylinder body and remove it completely
    import re
    
    # Remove the cylinder body section
    pattern = r'        <!-- CYLINDER OBJECT TO GRASP.*?</body>'
    xml_content = re.sub(pattern, '', xml_content, flags=re.DOTALL)
    
    # Write gripper-only model
    with open('/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper_only.xml', 'w') as f:
        f.write(xml_content)
    
    print("✅ Created ezgripper_only.xml without cylinder")

if __name__ == "__main__":
    main()
