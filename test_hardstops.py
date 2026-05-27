#!/usr/bin/env python3
"""
Test and visualize EZGripper hard stops in MuJoCo.

This script:
1. Loads the EZGripper model
2. Tests joint limits and hard stop collisions
3. Provides interactive visualization
4. Reports collision issues
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def analyze_collision_groups(model):
    """Analyze collision groups for all stop geoms."""
    print("\n" + "="*70)
    print("COLLISION GROUP ANALYSIS")
    print("="*70)
    
    stop_geoms = [
        'palm_stop_f1_lower', 'palm_stop_f2_lower',
        'palm_stop_f1_upper', 'palm_stop_f2_upper',
        'f1l1_stop_lower', 'f1l1_stop_upper',
        'f2l1_stop_lower', 'f2l1_stop_upper',
        'f1l1_l2stop', 'f2l1_l2stop'
    ]
    
    for geom_name in stop_geoms:
        try:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            contype = model.geom_contype[geom_id]
            conaffinity = model.geom_conaffinity[geom_id]
            print(f"{geom_name:25s} contype={contype:3d} (0b{contype:08b})  "
                  f"conaffinity={conaffinity:3d} (0b{conaffinity:08b})")
        except:
            print(f"{geom_name:25s} NOT FOUND")
    
    print("\nCOLLISION RULES:")
    print("Collision occurs if: (contype1 & conaffinity2) OR (contype2 & conaffinity1)")
    print("\nEXAMPLE:")
    print("  palm_stop (contype=4) vs f1l1_stop (contype=1):")
    print("    (4 & 1) = 0  AND  (1 & 4) = 0  →  NO COLLISION ❌")
    print("  stop (contype=8) vs stop (contype=8):")
    print("    (8 & 8) = 8  →  COLLISION ✅")

def analyze_contact_exclusions(model):
    """Analyze contact exclusions."""
    print("\n" + "="*70)
    print("CONTACT EXCLUSIONS")
    print("="*70)
    
    if model.nexclude > 0:
        print(f"Number of exclusions: {model.nexclude}")
        for i in range(model.nexclude):
            # MuJoCo stores exclusions as signature pairs
            # We'll just report the count
            print(f"  Exclusion {i} exists (body-pair signature)")
        
        print("\n⚠️  ISSUE: Body-level exclusions prevent ALL collisions,")
        print("    including hard stops!")
        print("    Check XML for <exclude body1=... body2=...> tags")
    else:
        print("No exclusions found")

def test_joint_at_limit(model, data, joint_name, angle_deg, description):
    """Test joint at a specific angle and check for contacts."""
    print(f"\n{description}")
    print("-" * 70)
    
    # Get joint ID
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    
    # Set joint angle
    angle_rad = np.deg2rad(angle_deg)
    data.qpos[joint_id] = angle_rad
    
    # Forward kinematics
    mujoco.mj_forward(model, data)
    
    # Check contacts
    print(f"Joint: {joint_name} = {angle_deg}° ({angle_rad:.4f} rad)")
    print(f"Number of contacts: {data.ncon}")
    
    if data.ncon > 0:
        for i in range(data.ncon):
            contact = data.contact[i]
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            
            # Check if this is a stop contact
            is_stop = ('stop' in geom1_name or 'stop' in geom2_name)
            marker = "✅ STOP" if is_stop else "  "
            
            print(f"  {marker} Contact {i}: {geom1_name:20s} <-> {geom2_name:20s}  "
                  f"dist={contact.dist:.6f}")
    else:
        print("  ❌ NO CONTACTS - Hard stops not engaging!")
    
    return data.ncon

def interactive_test(model, data):
    """Interactive visualization with joint control."""
    print("\n" + "="*70)
    print("INTERACTIVE VISUALIZATION")
    print("="*70)
    print("\nControls:")
    print("  1/2: Rotate F1_palm_knuckle joint")
    print("  3/4: Rotate F1_knuckle_tip joint")
    print("  5/6: Apply tendon force")
    print("  R: Reset to neutral")
    print("  Q: Quit")
    print("\nWatch for:")
    print("  - Green boxes: Lower limit stops")
    print("  - Yellow boxes: Upper limit stops")
    print("  - Red boxes: Finger stops")
    print("  - Cyan boxes: L1-L2 stops")
    
    # Get joint IDs
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    
    # Get actuator IDs
    act_f1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f1')
    act_f2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator_f2')
    
    # Reset
    mujoco.mj_resetData(model, data)
    
    # Launch viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        step = 0
        last_contact_report = 0
        
        while viewer.is_running():
            step_start = time.time()
            
            # Keyboard control (simplified - viewer doesn't expose keyboard directly)
            # Instead, we'll cycle through test positions automatically
            
            # Cycle through different test positions
            cycle_time = step * model.opt.timestep
            mode = int(cycle_time / 3.0) % 5
            
            if mode == 0:
                # Neutral position
                data.qpos[f1_palm_id] = 0.0
                data.qpos[f1_tip_id] = 0.0
                data.ctrl[act_f1_id] = 0.0
                data.ctrl[act_f2_id] = 0.0
                
            elif mode == 1:
                # Test lower limit (-90°)
                data.qpos[f1_palm_id] = -1.57
                data.qpos[f1_tip_id] = 0.0
                
            elif mode == 2:
                # Test upper limit (+25°)
                data.qpos[f1_palm_id] = 0.436
                data.qpos[f1_tip_id] = 0.0
                
            elif mode == 3:
                # Test L1-L2 limit
                data.qpos[f1_palm_id] = 0.0
                data.qpos[f1_tip_id] = 1.2
                
            elif mode == 4:
                # Test with tendon force
                data.ctrl[act_f1_id] = 1.0
                data.ctrl[act_f2_id] = 1.0
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Report contacts every 100 steps
            if step % 100 == 0 and step != last_contact_report:
                last_contact_report = step
                print(f"\n[Step {step}] Mode {mode}: ", end="")
                
                if mode == 0:
                    print("Neutral position")
                elif mode == 1:
                    print("Lower limit test (-90°)")
                elif mode == 2:
                    print("Upper limit test (+25°)")
                elif mode == 3:
                    print("L1-L2 limit test")
                elif mode == 4:
                    print("Tendon force test")
                
                print(f"  Contacts: {data.ncon}")
                
                if data.ncon > 0:
                    for i in range(min(data.ncon, 5)):  # Show first 5
                        contact = data.contact[i]
                        geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                        geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                        is_stop = ('stop' in geom1_name or 'stop' in geom2_name)
                        marker = "✅" if is_stop else "  "
                        print(f"    {marker} {geom1_name} <-> {geom2_name}")
            
            # Sync viewer
            viewer.sync()
            step += 1
            
            # Maintain real-time
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

def main():
    """Main test function."""
    print("="*70)
    print("EZGRIPPER HARD STOP TEST")
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
    
    # Analyze collision groups
    analyze_collision_groups(model)
    
    # Analyze contact exclusions
    analyze_contact_exclusions(model)
    
    # Test specific joint positions
    print("\n" + "="*70)
    print("TESTING JOINT LIMITS")
    print("="*70)
    
    test_joint_at_limit(model, data, 'F1_palm_knuckle', -90, 
                       "Test 1: F1 palm knuckle at LOWER limit (-90°)")
    
    test_joint_at_limit(model, data, 'F1_palm_knuckle', 25, 
                       "Test 2: F1 palm knuckle at UPPER limit (+25°)")
    
    test_joint_at_limit(model, data, 'F1_knuckle_tip', 69, 
                       "Test 3: F1 knuckle tip at UPPER limit (+69°)")
    
    # Interactive visualization
    print("\n" + "="*70)
    print("Starting interactive visualization...")
    print("The viewer will cycle through different test positions.")
    print("Close the viewer window to exit.")
    print("="*70)
    
    interactive_test(model, data)
    
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print("\nExpected issues:")
    print("  ❌ No contacts at joint limits (body exclusions)")
    print("  ❌ Collision group mismatches (4/6 vs 1)")
    print("\nTo fix:")
    print("  1. Remove body-level exclusions")
    print("  2. Change all stops to contype=8, conaffinity=8")
    print("  3. Re-run this test to verify")
    print("="*70)

if __name__ == '__main__':
    main()
