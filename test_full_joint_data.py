#!/usr/bin/env python3
"""
Record ALL joint angles throughout simulation to diagnose L1/L2 joint issue.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time
import csv

def main():
    print("="*70)
    print("FULL JOINT DATA RECORDING")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Get ALL joint IDs
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_knuckle_tip')
    
    act_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    t2_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger2_tendon')
    
    print(f"\nJoint IDs:")
    print(f"  F1_palm_knuckle: {f1_palm_id}")
    print(f"  F2_palm_knuckle: {f2_palm_id}")
    print(f"  F1_knuckle_tip:  {f1_tip_id}")
    print(f"  F2_knuckle_tip:  {f2_tip_id}")
    
    # Print joint ranges
    print(f"\nJoint Ranges:")
    print(f"  F1_palm: {np.rad2deg(model.jnt_range[f1_palm_id][0]):.1f}° to {np.rad2deg(model.jnt_range[f1_palm_id][1]):.1f}°")
    print(f"  F2_palm: {np.rad2deg(model.jnt_range[f2_palm_id][0]):.1f}° to {np.rad2deg(model.jnt_range[f2_palm_id][1]):.1f}°")
    print(f"  F1_tip:  {np.rad2deg(model.jnt_range[f1_tip_id][0]):.1f}° to {np.rad2deg(model.jnt_range[f1_tip_id][1]):.1f}°")
    print(f"  F2_tip:  {np.rad2deg(model.jnt_range[f2_tip_id][0]):.1f}° to {np.rad2deg(model.jnt_range[f2_tip_id][1]):.1f}°")
    
    # Data recording
    data_log = []
    
    print("\n" + "="*70)
    print("SIMULATION WITH FULL DATA LOGGING")
    print("="*70)
    print("\nApplying CLOSING command (ctrl = +1.0)")
    
    mujoco.mj_resetData(model, data)
    data.ctrl[act_id] = 1.0  # Close
    
    # Run simulation and log data
    for step in range(5000):
        mujoco.mj_step(model, data)
        
        # Log every 10 steps
        if step % 10 == 0:
            f1_palm = np.rad2deg(data.qpos[f1_palm_id])
            f2_palm = np.rad2deg(data.qpos[f2_palm_id])
            f1_tip = np.rad2deg(data.qpos[f1_tip_id])
            f2_tip = np.rad2deg(data.qpos[f2_tip_id])
            t1_len = data.ten_length[t1_id]
            t2_len = data.ten_length[t2_id]
            
            data_log.append({
                'step': step,
                'time': step * model.opt.timestep,
                'control': data.ctrl[act_id],
                'f1_palm': f1_palm,
                'f2_palm': f2_palm,
                'f1_tip': f1_tip,
                'f2_tip': f2_tip,
                't1_length': t1_len,
                't2_length': t2_len,
                'palm_diff': abs(f1_palm - f2_palm),
                'tip_diff': abs(f1_tip - f2_tip),
                'tendon_diff': abs(t1_len - t2_len),
                'contacts': data.ncon
            })
        
        # Print progress
        if step % 500 == 0:
            f1_palm = np.rad2deg(data.qpos[f1_palm_id])
            f2_palm = np.rad2deg(data.qpos[f2_palm_id])
            f1_tip = np.rad2deg(data.qpos[f1_tip_id])
            f2_tip = np.rad2deg(data.qpos[f2_tip_id])
            
            print(f"\nStep {step:4d}:")
            print(f"  PALM: F1={f1_palm:7.2f}° F2={f2_palm:7.2f}° | Diff={abs(f1_palm-f2_palm):5.2f}°")
            print(f"  TIP:  F1={f1_tip:7.2f}° F2={f2_tip:7.2f}° | Diff={abs(f1_tip-f2_tip):5.2f}°")
            print(f"  TENDON: T1={data.ten_length[t1_id]:.6f}m T2={data.ten_length[t2_id]:.6f}m")
    
    # Save to CSV
    csv_file = '/home/sake/linorobot2_ws/src/ezgripper_sim/joint_data_log.csv'
    with open(csv_file, 'w', newline='') as f:
        if data_log:
            writer = csv.DictWriter(f, fieldnames=data_log[0].keys())
            writer.writeheader()
            writer.writerows(data_log)
    
    print(f"\n✅ Data saved to: {csv_file}")
    print(f"   Total records: {len(data_log)}")
    
    # Analysis
    print("\n" + "="*70)
    print("ANALYSIS")
    print("="*70)
    
    final = data_log[-1]
    print(f"\nFinal State (step {final['step']}):")
    print(f"  F1 palm: {final['f1_palm']:7.2f}°  |  F2 palm: {final['f2_palm']:7.2f}°  |  Diff: {final['palm_diff']:5.2f}°")
    print(f"  F1 tip:  {final['f1_tip']:7.2f}°  |  F2 tip:  {final['f2_tip']:7.2f}°  |  Diff: {final['tip_diff']:5.2f}°")
    print(f"  Tendon:  T1={final['t1_length']:.6f}m  T2={final['t2_length']:.6f}m  |  Diff: {final['tendon_diff']:.6f}m")
    
    # Check for L1/L2 joint issues
    print(f"\n🔍 L1/L2 Joint Analysis:")
    print(f"  F1 L1/L2 (tip) joint: {final['f1_tip']:.2f}° (range: -0.01° to 68.8°)")
    print(f"  F2 L1/L2 (tip) joint: {final['f2_tip']:.2f}° (range: -0.01° to 68.8°)")
    
    if final['f1_tip'] < 0 or final['f2_tip'] < 0:
        print(f"  ⚠️  WARNING: Tip joint(s) at negative angle (rotating backwards!)")
    
    if final['f1_tip'] > 60 or final['f2_tip'] > 60:
        print(f"  ⚠️  WARNING: Tip joint(s) near upper limit (over-rotating!)")
    
    # Symmetry check
    if final['palm_diff'] > 2.0:
        print(f"\n❌ ASYMMETRIC: Palm joints differ by {final['palm_diff']:.2f}°")
    else:
        print(f"\n✅ SYMMETRIC: Palm joints within {final['palm_diff']:.2f}°")
    
    if final['tendon_diff'] > 0.0001:
        print(f"❌ EQUALITY CONSTRAINT VIOLATED: Tendons differ by {final['tendon_diff']:.6f}m")
    else:
        print(f"✅ EQUALITY CONSTRAINT WORKING: Tendons equal within {final['tendon_diff']:.6f}m")
    
    # Interactive viewer
    print("\n" + "="*70)
    print("INTERACTIVE VIEWER")
    print("="*70)
    print("Holding at final position...")
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance = 0.3
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20
        
        step = 0
        while viewer.is_running() and step < 5000:
            step_start = time.time()
            
            # Hold position
            data.ctrl[act_id] = 1.0
            
            mujoco.mj_step(model, data)
            
            if step % 500 == 0:
                f1_palm = np.rad2deg(data.qpos[f1_palm_id])
                f2_palm = np.rad2deg(data.qpos[f2_palm_id])
                f1_tip = np.rad2deg(data.qpos[f1_tip_id])
                f2_tip = np.rad2deg(data.qpos[f2_tip_id])
                
                print(f"[{step:5d}] PALM: F1:{f1_palm:7.2f}° F2:{f2_palm:7.2f}° | "
                      f"TIP: F1:{f1_tip:7.2f}° F2:{f2_tip:7.2f}°")
            
            viewer.sync()
            step += 1
            
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    main()
