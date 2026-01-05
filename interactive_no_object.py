#!/usr/bin/env python3
"""
Interactive MuJoCo simulation: EZGripper closing WITHOUT object.
Shows dynamic closing motion where tips meet.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def run_simulation():
    """Run interactive simulation of gripper closing without object"""
    
    # Load model (cylinder should be commented out)
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    
    print("=" * 70)
    print("EZGripper Dynamic Simulation - NO OBJECT")
    print("=" * 70)
    print("Watch the gripper close dynamically - tips should meet")
    print("L2 spring keeps fingers straight until tendon force applied")
    print()
    print("Controls:")
    print("  - Simulation runs automatically")
    print("  - Drag mouse to rotate view")
    print("  - Scroll to zoom")
    print("  - Press Ctrl+C or close window to exit")
    print("=" * 70)
    
    # Simulation state
    start_time = None
    closing_duration = 3.0  # seconds to complete closing
    
    def controller(model, data):
        """Control gripper closing"""
        nonlocal start_time
        
        if start_time is None:
            start_time = data.time
        
        elapsed = data.time - start_time
        
        # Progressive closing over closing_duration seconds
        if elapsed < closing_duration:
            progress = elapsed / closing_duration
            tendon_force = 100.0 * progress  # 0 to 100
        else:
            tendon_force = 100.0  # Fully closed
        
        # Apply force
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
        data.ctrl[actuator_id] = tendon_force
        
        # Print status every 0.5 seconds
        if int(elapsed * 2) != int((elapsed - model.opt.timestep) * 2):
            l1_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
            l2_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
            l1_angle = np.degrees(data.qpos[l1_joint_id])
            l2_angle = np.degrees(data.qpos[l2_joint_id])
            print(f"t={elapsed:.1f}s | Force={tendon_force:.0f}% | L1={l1_angle:.1f}° L2={l2_angle:.1f}°")
    
    # Launch interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Set up camera
        viewer.cam.distance = 0.25
        viewer.cam.azimuth = 45
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.10, 0, 0.03]
        
        print("\nSimulation started - watch the gripper close!")
        
        # Run simulation
        while viewer.is_running():
            step_start = time.time()
            
            # Apply control
            controller(model, data)
            
            # Step physics
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()
            
            # Control frame rate
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    try:
        run_simulation()
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
