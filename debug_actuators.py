#!/usr/bin/env python3
"""
Debug: Check what actuators exist in the model
"""

import mujoco
import numpy as np

def main():
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    
    # Load model
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    print("="*70)
    print("ACTUATOR DEBUG")
    print("="*70)
    
    print(f"Total actuators: {model.nu}")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"Actuator {i}: '{name}'")
    
    print(f"\nTotal joints: {model.njnt}")
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        print(f"Joint {i}: '{name}'")
    
    print(f"\nTotal tendons: {model.ntendon}")
    for i in range(model.ntendon):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_TENDON, i)
        print(f"Tendon {i}: '{name}'")
    
    # Try to get the actuator ID
    try:
        act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
        print(f"\n✓ Found 'gripper_actuator' at ID: {act_id}")
    except:
        print(f"\n✗ 'gripper_actuator' not found!")
        
        # Try common alternative names
        alternatives = ['flex_tendon', 'gripper_flex_tendon', 'tendon_motor']
        for alt in alternatives:
            try:
                act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, alt)
                print(f"✓ Found '{alt}' at ID: {act_id}")
            except:
                print(f"✗ '{alt}' not found")

if __name__ == "__main__":
    main()
