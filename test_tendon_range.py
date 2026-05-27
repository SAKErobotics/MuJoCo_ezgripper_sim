#!/usr/bin/env python3
"""
Find the actual tendon length range by manually setting joint angles.
"""

import mujoco
import numpy as np

def main():
    print("="*70)
    print("TENDON LENGTH RANGE TEST")
    print("="*70)
    
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    f2_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F2_palm_knuckle')
    t1_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    
    print("\nManually setting joint angles to find tendon length range:")
    print("Joint range: -90° to +25°")
    
    test_angles = [-90, -60, -30, 0, 10, 20, 25]
    
    print("\n" + "="*70)
    for angle_deg in test_angles:
        angle_rad = np.deg2rad(angle_deg)
        
        mujoco.mj_resetData(model, data)
        data.qpos[f1_palm_id] = angle_rad
        data.qpos[f2_palm_id] = angle_rad
        mujoco.mj_forward(model, data)
        
        tendon_len = data.ten_length[t1_id]
        
        print(f"Angle: {angle_deg:4.0f}° → Tendon length: {tendon_len:.6f}m")
    
    # Find range
    mujoco.mj_resetData(model, data)
    data.qpos[f1_palm_id] = np.deg2rad(-90)
    data.qpos[f2_palm_id] = np.deg2rad(-90)
    mujoco.mj_forward(model, data)
    len_open = data.ten_length[t1_id]
    
    mujoco.mj_resetData(model, data)
    data.qpos[f1_palm_id] = np.deg2rad(25)
    data.qpos[f2_palm_id] = np.deg2rad(25)
    mujoco.mj_forward(model, data)
    len_closed = data.ten_length[t1_id]
    
    print("\n" + "="*70)
    print("SUMMARY")
    print("="*70)
    print(f"\nAt -90° (OPEN):   Tendon length = {len_open:.6f}m")
    print(f"At +25° (CLOSED): Tendon length = {len_closed:.6f}m")
    print(f"\nTendon travel: {abs(len_closed - len_open):.6f}m")
    print(f"Direction: {'SHORTENS when closing' if len_closed < len_open else 'LENGTHENS when closing'}")
    
    print(f"\n✅ Correct control range should be:")
    print(f"   ctrlrange=\"{min(len_open, len_closed):.6f} {max(len_open, len_closed):.6f}\"")
    
    if len_closed < len_open:
        print(f"\n   Control {len_closed:.6f} = CLOSED (+25°)")
        print(f"   Control {len_open:.6f} = OPEN (-90°)")
    else:
        print(f"\n   Control {len_open:.6f} = CLOSED (+25°)")
        print(f"   Control {len_closed:.6f} = OPEN (-90°)")

if __name__ == '__main__':
    main()
