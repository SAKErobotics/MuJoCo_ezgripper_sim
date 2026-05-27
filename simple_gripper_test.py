#!/usr/bin/env python3
"""
Simple gripper simulation without cylinder - based on working tendon control test
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("SIMPLE GRIPPER TEST - NO CYLINDER")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper_only.xml'
    
    # Read and modify the XML for better stability
    with open(model_path, 'r') as f:
        xml_content = f.read()
    
    # Use absolute paths so model works from any directory
    xml_content = xml_content.replace('file="meshes/', 'file="/home/sake/linorobot2_ws/src/ezgripper_sim/meshes/')
    
    # Reduce damping for faster spring opening (still 2x original for stability)
    xml_content = xml_content.replace('damping="0.05"', 'damping="0.1"')
    
    # Load modified model
    model = mujoco.MjModel.from_xml_string(xml_content)
    data = mujoco.MjData(model)
    
    # Get IDs
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
    
    # Get tendon IDs
    try:
        t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
        t2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')
        print(f"✓ Found tendons: finger1_tendon (ID: {t1_id}), finger2_tendon (ID: {t2_id})")
    except:
        print("✗ No tendons found")
        return
    
    print("\nTest sequence:")
    print("1. Full open (control = 0.0)")
    print("2. Full close (control = 1.0)")
    print("3. Hold position (2 seconds)")
    print("4. Full open (control = 0.0)")
    print("5. Hold position (2 seconds)")
    print("6. Record full range and tendon lengths")
    print("="*70)
    
    # Test sequence
    test_sequence = [
        (0.0, "FULL_OPEN", 2.0),    # Open for 2 seconds
        (1.0, "FULL_CLOSE", 2.0),  # Close for 2 seconds
        (0.0, "OPEN_AGAIN", 2.0),   # Open again for 2 seconds
    ]
    
    results = []
    
    with mujoco.viewer.launch(model, data) as viewer:
        step = 0
        current_test = 0
        control, phase_name, duration = test_sequence[current_test]
        phase_start_step = step
        
        while step < 6000:  # 12 seconds total
            
            # Change phase when needed
            if step - phase_start_step >= int(duration / 0.002):  # duration / timestep
                current_test += 1
                if current_test >= len(test_sequence):
                    current_test = 0  # Loop back
                
                control, phase_name, duration = test_sequence[current_test]
                phase_start_step = step
                print(f"\n--- {phase_name} (control = {control:.1f}) ---")
            
            # Set control
            data.ctrl[act_id] = control
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Record data every 100 steps
            if step % 100 == 0:
                f1_palm = np.rad2deg(data.qpos[f1_palm_id])
                f2_palm = np.rad2deg(data.qpos[f2_palm_id])
                f1_tip = np.rad2deg(data.qpos[f1_tip_id])
                f2_tip = np.rad2deg(data.qpos[f2_tip_id])
                delta = abs(f1_palm - f2_palm)
                
                # Get tendon lengths
                t1_length = data.ten_length[t1_id]
                t2_length = data.ten_length[t2_id]
                avg_length = (t1_length + t2_length) / 2
                
                print(f"[{step:4d}] {phase_name:12s} Ctrl:{control:5.1f} | Joints: F1:{f1_palm:6.2f}° F2:{f2_palm:6.2f}° Δ:{delta:5.2f}°")
                print(f"        Tendons: T1:{t1_length:6.4f} T2:{t2_length:6.4f} Avg:{avg_length:6.4f}")
                
                # Store results
                results.append({
                    'step': step,
                    'phase': phase_name,
                    'control': control,
                    'f1_palm': f1_palm,
                    'f2_palm': f2_palm,
                    'delta': delta,
                    't1_length': t1_length,
                    't2_length': t2_length,
                    'avg_length': avg_length
                })
            
            # Update viewer
            viewer.sync()
            
            step += 1
            time.sleep(0.002)  # 500 Hz
    
    print("\n" + "="*70)
    print("FULL RANGE ANALYSIS")
    print("="*70)
    
    # Find min and max positions
    min_f1 = min(r['f1_palm'] for r in results)
    max_f1 = max(r['f1_palm'] for r in results)
    min_f2 = min(r['f2_palm'] for r in results)
    max_f2 = max(r['f2_palm'] for r in results)
    
    min_tendon = min(r['avg_length'] for r in results)
    max_tendon = max(r['avg_length'] for r in results)
    
    print(f"\nJoint Range:")
    print(f"  F1_palm: {min_f1:.2f}° → {max_f1:.2f}° (range: {max_f1 - min_f1:.2f}°)")
    print(f"  F2_palm: {min_f2:.2f}° → {max_f2:.2f}° (range: {max_f2 - min_f2:.2f}°)")
    
    print(f"\nTendon Range:")
    print(f"  Length: {min_tendon:.6f} → {max_tendon:.6f} (change: {max_tendon - min_tendon:.6f})")
    
    print(f"\nControl Mapping:")
    print(f"  Open:  Control = 0.0")
    print(f"  Close: Control = 1.0")
    
    print("\n" + "="*70)
    print("SIMPLE GRIPPER TEST COMPLETE")
    print("="*70)

if __name__ == "__main__":
    main()
