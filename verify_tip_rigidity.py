#!/usr/bin/env python3
"""
Verify L2/Finger Tip Rigidity
Monitors the relative rotation between L2 body and finger pad geom
to confirm they remain rigidly fixed (no relative rotation).
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def quaternion_to_rotation_matrix(quat):
    """Convert quaternion to rotation matrix"""
    w, x, y, z = quat
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
    ])

def rotation_matrix_to_euler(R):
    """Convert rotation matrix to Euler angles (XYZ)"""
    sy = np.sqrt(R[0,0]**2 + R[1,0]**2)
    singular = sy < 1e-6
    
    if not singular:
        x = np.arctan2(R[2,1], R[2,2])
        y = np.arctan2(-R[2,0], sy)
        z = np.arctan2(R[1,0], R[0,0])
    else:
        x = np.arctan2(-R[1,2], R[1,1])
        y = np.arctan2(-R[2,0], sy)
        z = 0
    
    return np.degrees([x, y, z])

def run_simulation():
    """Run simulation and monitor L2/tip rigidity"""
    
    # Load model with cylinder
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    
    # Get body IDs
    f1_l2_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'F1_L2')
    
    # Get geom ID for finger tip
    f1_tip_geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'f1_tip')
    
    # Get joint IDs for monitoring
    l1_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    l2_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    
    print("=" * 80)
    print("L2/FINGER TIP RIGIDITY VERIFICATION")
    print("=" * 80)
    print("Monitoring relative rotation between L2 body and finger pad geom")
    print("If rigidly attached: relative rotation should be ZERO at all times")
    print()
    print("Body IDs:")
    print(f"  F1_L2 body: {f1_l2_body_id}")
    print(f"  f1_tip geom: {f1_tip_geom_id}")
    print()
    
    # Simulation state
    start_time = None
    closing_duration = 3.0
    initial_relative_euler = None
    max_rotation_change = 0.0
    
    def controller(model, data):
        """Control gripper closing"""
        nonlocal start_time, initial_relative_euler, max_rotation_change
        
        if start_time is None:
            start_time = data.time
        
        elapsed = data.time - start_time
        
        # Progressive closing
        if elapsed < closing_duration:
            progress = elapsed / closing_duration
            tendon_force = 100.0 * progress
        else:
            tendon_force = 100.0
        
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
        data.ctrl[actuator_id] = tendon_force
        
        # Get L2 body pose
        l2_body_xpos = data.xpos[f1_l2_body_id].copy()
        l2_body_xquat = data.xquat[f1_l2_body_id].copy()
        l2_body_xmat = data.xmat[f1_l2_body_id].reshape(3, 3).copy()
        
        # Get finger tip geom pose
        tip_geom_xpos = data.geom_xpos[f1_tip_geom_id].copy()
        tip_geom_xmat = data.geom_xmat[f1_tip_geom_id].reshape(3, 3).copy()
        
        # Calculate relative rotation matrix
        # R_relative = R_tip * R_l2^T
        R_relative = tip_geom_xmat @ l2_body_xmat.T
        
        # Convert to Euler angles to see rotation
        relative_euler = rotation_matrix_to_euler(R_relative)
        
        # Calculate magnitude of relative rotation
        relative_rotation_mag = np.linalg.norm(relative_euler)
        max_relative_rotation = max(max_relative_rotation, relative_rotation_mag)
        
        # Get joint angles
        l1_angle = np.degrees(data.qpos[l1_joint_id])
        l2_angle = np.degrees(data.qpos[l2_joint_id])
        
        # Print status every 0.5 seconds
        if int(elapsed * 2) != int((elapsed - model.opt.timestep) * 2):
            print(f"t={elapsed:.1f}s | Force={tendon_force:.0f}% | "
                  f"L1={l1_angle:6.2f}° L2={l2_angle:6.2f}° | "
                  f"Rel.Rot: [{relative_euler[0]:6.3f}° {relative_euler[1]:6.3f}° {relative_euler[2]:6.3f}°] "
                  f"Mag={relative_rotation_mag:.4f}°")
    
    # Launch interactive viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.25
        viewer.cam.azimuth = 45
        viewer.cam.elevation = -20
        viewer.cam.lookat[:] = [0.12, 0, 0.08]
        
        print("\nSimulation started - monitoring L2/tip relative rotation...")
        print("Time | Force | L1 Angle | L2 Angle | Relative Rotation [X° Y° Z°] | Magnitude")
        print("-" * 80)
        
        # Run simulation
        while viewer.is_running():
            step_start = time.time()
            
            controller(model, data)
            mujoco.mj_step(model, data)
            viewer.sync()
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)
    
    print("\n" + "=" * 80)
    print("RIGIDITY VERIFICATION RESULTS")
    print("=" * 80)
    print(f"Maximum relative rotation observed: {max_relative_rotation:.6f}°")
    print()
    
    if max_relative_rotation < 0.001:
        print("✓ EXCELLENT: Finger tip is PERFECTLY RIGID with L2 body")
        print("  Relative rotation < 0.001° (essentially zero)")
    elif max_relative_rotation < 0.01:
        print("✓ GOOD: Finger tip is RIGIDLY attached to L2 body")
        print("  Relative rotation < 0.01° (negligible)")
    elif max_relative_rotation < 0.1:
        print("⚠ ACCEPTABLE: Finger tip has minimal flex relative to L2")
        print("  Relative rotation < 0.1° (very small)")
    else:
        print("✗ PROBLEM: Finger tip is NOT rigidly attached to L2")
        print(f"  Relative rotation = {max_relative_rotation:.4f}° (too large)")
        print("  This indicates either:")
        print("    - Incorrect body hierarchy in XML")
        print("    - Unwanted joint between L2 and tip")
        print("    - Numerical instability from collision forces")
    
    print("=" * 80)

if __name__ == '__main__':
    try:
        run_simulation()
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
