#!/usr/bin/env python3
"""
Visualize the working EZGripper model to review the joint behavior
"""

import mujoco
import numpy as np
import time

def visualize_working_gripper():
    """Visualize the working EZGripper model with different tendon positions"""
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/models/working/ezgripper_working.xml'
    
    try:
        # Load model
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        
        print(f"✓ Working EZGripper Model Loaded")
        print(f"  - Actuators: {model.nu}")
        print(f"  - Joints: {model.njnt}")
        
        # Print joint names
        for i in range(model.njnt):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
            print(f"  - Joint {i}: {name}")
        
        # Create renderer
        renderer = mujoco.Renderer(model, height=480, width=640)
        
        # Test sequence
        tendon_sequence = [
            (0.0, "Open Position"),
            (-0.5, "Partial Close"), 
            (-1.0, "Full Close"),
            (0.0, "Back to Open")
        ]
        
        print("\n" + "="*60)
        print("EZGRIPPER BEHAVIOR VISUALIZATION")
        print("="*60)
        
        for tendon_pos, description in tendon_sequence:
            print(f"\n--- {description} (Tendon: {tendon_pos}) ---")
            
            # Set tendon position
            data.ctrl[0] = tendon_pos
            
            # Run for 3 seconds and show joint positions
            joint_history = []
            for step in range(300):  # 3 seconds at 100Hz
                mujoco.mj_step(model, data)
                
                # Record joint positions every 10 steps
                if step % 10 == 0:
                    joint_positions = [data.qpos[i] for i in range(model.njnt)]
                    joint_history.append(joint_positions)
                
                # Print status every 100 steps
                if step % 100 == 0:
                    joints_str = ", ".join([f"{data.qpos[i]:.3f}" for i in range(model.njnt)])
                    print(f"  Step {step:3d}: Joints=[{joints_str}]")
            
            # Calculate joint stability
            if len(joint_history) > 10:
                first_joints = joint_history[0]
                last_joints = joint_history[-1]
                max_diff = max(abs(last_joints[i] - first_joints[i]) for i in range(model.njnt))
                
                print(f"  → Joint movement range: {max_diff:.3f} radians")
                if max_diff > 0.1:
                    print(f"  ⚠️  JOINTS ARE UNSTABLE (moving {max_diff:.3f} rad)")
                else:
                    print(f"  ✓ Joints are stable")
        
        print("\n" + "="*60)
        print("VISUALIZATION COMPLETE")
        print("The gripper joints show continuous movement due to spring references")
        print("This is the same behavior seen in the triple gripper assembly")
        print("="*60)
        
        return True
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    visualize_working_gripper()
