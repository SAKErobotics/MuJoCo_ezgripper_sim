#!/usr/bin/env python3
"""
Stabilized EZGripper simulation with improved settling
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def main():
    print("="*70)
    print("STABILIZED GRIPPER TEST")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    
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
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    
    print("\nFull range sequence (20 seconds total):")
    print("1. Full closed (0.5 seconds) - control = -2.0")
    print("2. Gradual open (5 seconds) - control -2.0 → 2.0")
    print("3. Hold position (2 seconds)")
    print("4. Gradual close (5 seconds) - control 2.0 → -2.0")
    print("5. Hold position (2 seconds)")
    print("6. Full open (5 seconds) - control = 2.0")
    print("="*70)
    
    # Simulation parameters
    dt = 0.002  # 500 Hz
    control = -2.0  # Start fully closed
    phase = "FULL_CLOSED"
    
    with mujoco.viewer.launch(model, data) as viewer:
        step = 0
        
        while step < 10000:  # 20 seconds total
            
            # Phase control - full range
            if phase == "FULL_CLOSED" and step < 250:  # 0.5 seconds
                control = -2.0  # Fully closed
                data.ctrl[act_id] = control
                if step == 249:
                    phase = "GRADUAL_OPEN"
                    print("\n" + "="*70)
                    print("PHASE 2: GRADUAL OPEN - Full range")
                    print("="*70)
            
            elif phase == "GRADUAL_OPEN" and step < 2750:  # 5 seconds
                if step % 50 == 0 and control < 2.0:
                    control += 0.08  # Gradual opening
                data.ctrl[act_id] = control
                if step == 2749:
                    phase = "HOLD1"
                    print("\n" + "="*70)
                    print("PHASE 3: HOLDING - Full open")
                    print("="*70)
            
            elif phase == "HOLD1" and step < 3750:  # 2 seconds
                data.ctrl[act_id] = control
                if step == 3749:
                    phase = "GRADUAL_CLOSE"
                    print("\n" + "="*70)
                    print("PHASE 4: GRADUAL CLOSE - Full range")
                    print("="*70)
            
            elif phase == "GRADUAL_CLOSE" and step < 6500:  # 5 seconds
                if step % 50 == 0 and control > -2.0:
                    control -= 0.08  # Gradual closing
                data.ctrl[act_id] = control
                if step == 6499:
                    phase = "HOLD2"
                    print("\n" + "="*70)
                    print("PHASE 5: HOLDING - Full closed")
                    print("="*70)
            
            elif phase == "HOLD2" and step < 7500:  # 2 seconds
                data.ctrl[act_id] = control
                if step == 7499:
                    phase = "FULL_OPEN"
                    print("\n" + "="*70)
                    print("PHASE 6: FULL OPEN - Maximum opening")
                    print("="*70)
            
            elif phase == "FULL_OPEN":  # 5 seconds
                control = 2.0  # Fully open
                data.ctrl[act_id] = control
            
            # Print status every 250 steps (more frequent for better visibility)
            if step % 250 == 0:
                f1_palm = np.rad2deg(data.qpos[f1_palm_id])
                f2_palm = np.rad2deg(data.qpos[f2_palm_id])
                f1_tip = np.rad2deg(data.qpos[f1_tip_id])
                f2_tip = np.rad2deg(data.qpos[f2_tip_id])
                delta = abs(f1_palm - f2_palm)
                
                print(f"[{step:4d}] {phase:12s} Ctrl:{control:6.2f} | PALM: F1: {f1_palm:6.2f}° F2: {f2_palm:6.2f}° Δ: {delta:5.2f}° {'✅' if delta < 1.0 else '❌'}")
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()
            
            step += 1
            time.sleep(dt)
    
    print("\n" + "="*70)
    print("STABILIZED TEST COMPLETE")
    print("="*70)

if __name__ == "__main__":
    main()
