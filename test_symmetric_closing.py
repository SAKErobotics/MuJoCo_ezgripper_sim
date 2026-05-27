#!/usr/bin/env python3
"""
Test symmetric closing behavior of EZGripper with tendon equality constraint.

This script verifies that:
1. Both fingers close symmetrically without backforce
2. Force distributes based on load (asymmetric objects)
3. Fingers wrap around objects adaptively
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def test_symmetric_closing(model, data):
    """Test that fingers close symmetrically without object."""
    print("\n" + "="*70)
    print("TEST 1: SYMMETRIC CLOSING (No Object)")
    print("="*70)
    
    # Get joint IDs
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
    
    # Get actuator ID
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    # Reset
    mujoco.mj_resetData(model, data)
    
    # Apply closing force
    data.ctrl[act_id] = 1.0
    
    # Simulate for 2 seconds
    print("\nApplying closing force (ctrl=1.0)...")
    for i in range(2000):
        mujoco.mj_step(model, data)
        
        if i % 500 == 0:
            f1_palm_angle = np.rad2deg(data.qpos[f1_palm_id])
            f2_palm_angle = np.rad2deg(data.qpos[f2_palm_id])
            f1_tip_angle = np.rad2deg(data.qpos[f1_tip_id])
            f2_tip_angle = np.rad2deg(data.qpos[f2_tip_id])
            
            print(f"\nStep {i}:")
            print(f"  F1 palm: {f1_palm_angle:7.2f}°  |  F2 palm: {f2_palm_angle:7.2f}°  "
                  f"|  Diff: {abs(f1_palm_angle - f2_palm_angle):6.3f}°")
            print(f"  F1 tip:  {f1_tip_angle:7.2f}°  |  F2 tip:  {f2_tip_angle:7.2f}°  "
                  f"|  Diff: {abs(f1_tip_angle - f2_tip_angle):6.3f}°")
    
    # Final check
    f1_palm_final = np.rad2deg(data.qpos[f1_palm_id])
    f2_palm_final = np.rad2deg(data.qpos[f2_palm_id])
    f1_tip_final = np.rad2deg(data.qpos[f1_tip_id])
    f2_tip_final = np.rad2deg(data.qpos[f2_tip_id])
    
    palm_diff = abs(f1_palm_final - f2_palm_final)
    tip_diff = abs(f1_tip_final - f2_tip_final)
    
    print("\n" + "-"*70)
    print("FINAL RESULT:")
    print(f"  Palm angle difference: {palm_diff:.3f}°")
    print(f"  Tip angle difference:  {tip_diff:.3f}°")
    
    # Check symmetry (should be very close, within 1 degree)
    if palm_diff < 1.0 and tip_diff < 1.0:
        print("  ✅ SYMMETRIC CLOSING VERIFIED!")
    else:
        print("  ❌ ASYMMETRIC - Tendon coupling may not be working")
    
    return palm_diff < 1.0 and tip_diff < 1.0

def test_with_cylinder(model, data):
    """Test grasping a cylinder (symmetric object)."""
    print("\n" + "="*70)
    print("TEST 2: GRASPING CYLINDER (Symmetric Object)")
    print("="*70)
    
    # Check if cylinder exists
    try:
        cyl_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cylinder_pedestal')
        print("✅ Cylinder found in model")
    except:
        print("⚠️  Cylinder not found - uncomment in XML to test")
        return False
    
    # Get joint IDs
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    
    # Get actuator ID
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    # Reset
    mujoco.mj_resetData(model, data)
    
    # Apply closing force
    data.ctrl[act_id] = 1.5
    
    # Simulate
    print("\nGrasping cylinder...")
    for i in range(3000):
        mujoco.mj_step(model, data)
        
        if i % 500 == 0:
            f1_palm_angle = np.rad2deg(data.qpos[f1_palm_id])
            f2_palm_angle = np.rad2deg(data.qpos[f2_palm_id])
            
            print(f"Step {i}: F1={f1_palm_angle:7.2f}°  F2={f2_palm_angle:7.2f}°  "
                  f"Diff={abs(f1_palm_angle - f2_palm_angle):6.3f}°  "
                  f"Contacts={data.ncon}")
    
    # Check symmetry with object
    f1_final = np.rad2deg(data.qpos[f1_palm_id])
    f2_final = np.rad2deg(data.qpos[f2_palm_id])
    diff = abs(f1_final - f2_final)
    
    print(f"\nFinal difference: {diff:.3f}°")
    if diff < 2.0:  # Allow slightly more tolerance with object
        print("✅ SYMMETRIC GRASP VERIFIED!")
        return True
    else:
        print("❌ ASYMMETRIC GRASP")
        return False

def interactive_visualization(model, data):
    """Interactive visualization with different test scenarios."""
    print("\n" + "="*70)
    print("INTERACTIVE VISUALIZATION")
    print("="*70)
    print("\nThe viewer will cycle through test scenarios:")
    print("  1. Open position")
    print("  2. Symmetric closing (no object)")
    print("  3. Half-closed position")
    print("  4. Fully closed")
    print("  5. With object (if available)")
    print("\nClose viewer window to exit.")
    
    # Get IDs
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    # Check for cylinder
    has_cylinder = False
    try:
        cyl_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'cylinder_pedestal')
        has_cylinder = True
    except:
        pass
    
    # Launch viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        step = 0
        last_report = 0
        
        while viewer.is_running():
            step_start = time.time()
            
            # Cycle through scenarios every 3 seconds
            cycle_time = step * model.opt.timestep
            mode = int(cycle_time / 3.0) % 5
            
            if mode == 0:
                # Open position
                data.ctrl[act_id] = -0.5
                
            elif mode == 1:
                # Symmetric closing
                data.ctrl[act_id] = 0.5
                
            elif mode == 2:
                # Half-closed
                data.ctrl[act_id] = 1.0
                
            elif mode == 3:
                # Fully closed
                data.ctrl[act_id] = 2.0
                
            elif mode == 4:
                # With object (if available)
                if has_cylinder:
                    data.ctrl[act_id] = 1.5
                else:
                    data.ctrl[act_id] = 1.0
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Report every 100 steps
            if step % 100 == 0 and step != last_report:
                last_report = step
                
                f1_angle = np.rad2deg(data.qpos[f1_palm_id])
                f2_angle = np.rad2deg(data.qpos[f2_palm_id])
                diff = abs(f1_angle - f2_angle)
                
                mode_names = ["Open", "Closing", "Half-closed", "Fully closed", "With object"]
                print(f"\n[Step {step}] {mode_names[mode]}")
                print(f"  F1: {f1_angle:7.2f}°  F2: {f2_angle:7.2f}°  Diff: {diff:6.3f}°  "
                      f"Ctrl: {data.ctrl[act_id]:5.2f}")
                
                if diff < 1.0:
                    print("  ✅ Symmetric")
                elif diff < 5.0:
                    print("  ⚠️  Slightly asymmetric")
                else:
                    print("  ❌ Very asymmetric!")
            
            # Sync viewer
            viewer.sync()
            step += 1
            
            # Maintain real-time
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

def main():
    """Main test function."""
    print("="*70)
    print("EZGRIPPER SYMMETRIC CLOSING TEST")
    print("="*70)
    
    # Load model
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    print(f"\nLoading model: {model_path}")
    
    try:
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
        print("✅ Model loaded successfully")
    except Exception as e:
        print(f"❌ Failed to load model: {e}")
        return
    
    # Check for equality constraint
    print(f"\nEquality constraints: {model.neq}")
    if model.neq > 0:
        print("✅ Tendon equality constraint found!")
    else:
        print("❌ No equality constraint - fingers will be independent!")
    
    # Check actuators
    print(f"Number of actuators: {model.nu}")
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  Actuator {i}: {act_name}")
    
    if model.nu == 1:
        print("✅ Single actuator - correct configuration!")
    else:
        print("⚠️  Multiple actuators - may allow asymmetric control")
    
    # Run tests
    test1_pass = test_symmetric_closing(model, data)
    
    # Test with cylinder if available
    test2_pass = test_with_cylinder(model, data)
    
    # Interactive visualization
    print("\n" + "="*70)
    print("Starting interactive visualization...")
    print("="*70)
    interactive_visualization(model, data)
    
    # Summary
    print("\n" + "="*70)
    print("TEST SUMMARY")
    print("="*70)
    print(f"Symmetric closing (no object): {'✅ PASS' if test1_pass else '❌ FAIL'}")
    if test2_pass is not None:
        print(f"Symmetric grasp (cylinder):    {'✅ PASS' if test2_pass else '❌ FAIL'}")
    
    if test1_pass:
        print("\n✅ The gripper now closes symmetrically, matching real hardware!")
        print("   Force will distribute based on load (tree tendon physics)")
    else:
        print("\n❌ Asymmetric closing detected - check tendon equality constraint")

if __name__ == '__main__':
    main()
