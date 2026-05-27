#!/usr/bin/env python3
"""
MuJoCo visualization of the working EZGripper model
"""

import mujoco
import numpy as np
import time

def visualize_gripper_in_mujoco():
    """Launch MuJoCo viewer to see the working gripper behavior"""
    
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
        
        # Launch MuJoCo viewer
        print("\n" + "="*60)
        print("LAUNCHING MUJOCO VIEWER")
        print("="*60)
        print("Controls:")
        print("  - Space: Pause/Resume simulation")
        print("  - Tab: Cycle through cameras")
        print("  - Esc: Exit viewer")
        print("  - The gripper will cycle through open/partial/closed positions")
        print("="*60)
        
        # Create viewer
        with mujoco.MjViewer(model, data) as viewer:
            # Set initial camera view
            viewer.cam.lookat[:] = [0.1, 0, 0.1]  # Look at gripper center
            viewer.cam.distance = 0.5  # Camera distance
            viewer.cam.azimuth = 90   # Side view
            viewer.cam.elevation = -20  # Slightly tilted down
            
            # Simulation loop
            step = 0
            tendon_positions = [0.0, -0.5, -1.0, 0.0]  # Open -> Partial -> Closed -> Open
            tendon_index = 0
            tendon_change_interval = 500  # Change tendon every 500 steps
            
            print("Starting simulation...")
            
            while True:
                # Change tendon position periodically
                if step % tendon_change_interval == 0:
                    tendon_pos = tendon_positions[tendon_index % len(tendon_positions)]
                    data.ctrl[0] = tendon_pos
                    
                    position_names = ["OPEN", "PARTIAL CLOSE", "FULL CLOSE", "BACK TO OPEN"]
                    pos_name = position_names[tendon_index % len(position_names)]
                    print(f"\n>>> Tendon Position: {tendon_pos} ({pos_name})")
                    
                    tendon_index += 1
                
                # Step simulation
                mujoco.mj_step(model, data)
                
                # Update viewer
                viewer.sync()
                
                # Print joint positions every 100 steps
                if step % 100 == 0:
                    joints_str = ", ".join([f"{data.qpos[i]:.3f}" for i in range(model.njnt)])
                    print(f"Step {step:4d}: Joints=[{joints_str}]")
                
                step += 1
                
                # Small delay for real-time visualization
                time.sleep(0.01)
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    visualize_gripper_in_mujoco()
