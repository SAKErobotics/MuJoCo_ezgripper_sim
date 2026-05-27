#!/usr/bin/env python3
"""
Analyze tendon torque at L1/L2 joint as a function of joint angle.
Detects if tendon crosses to wrong side of joint axis (sign flip).
"""

import mujoco
import numpy as np
import matplotlib.pyplot as plt

def main():
    print("="*70)
    print("TENDON TORQUE ANALYSIS vs L1/L2 JOINT ANGLE")
    print("="*70)
    
    # Load model
    model_path = '/home/sake/linorobot2_ws/src/ezgripper_sim/ezgripper.xml'
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    
    # Get joint and tendon IDs
    f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_knuckle_tip')
    f1_palm_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'F1_palm_knuckle')
    tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    
    # Get qpos addresses
    tip_qpos_adr = model.jnt_qposadr[f1_tip_id]
    palm_qpos_adr = model.jnt_qposadr[f1_palm_id]
    
    print(f"\nJoint limits:")
    print(f"  F1_knuckle_tip: {np.rad2deg(model.jnt_range[f1_tip_id, 0]):.2f}° to {np.rad2deg(model.jnt_range[f1_tip_id, 1]):.2f}°")
    print(f"  F1_palm_knuckle: {np.rad2deg(model.jnt_range[f1_palm_id, 0]):.2f}° to {np.rad2deg(model.jnt_range[f1_palm_id, 1]):.2f}°")
    
    # Test range: ONLY within the allowed joint limits
    limit_low = model.jnt_range[f1_tip_id, 0]
    limit_high = model.jnt_range[f1_tip_id, 1]
    angles = np.linspace(limit_low, limit_high, 100)
    
    torques = []
    moment_arms = []
    tendon_lengths = []
    
    print(f"\nScanning L1/L2 angles WITHIN ALLOWED RANGE: {np.rad2deg(limit_low):.1f}° to {np.rad2deg(limit_high):.1f}°...")
    print("\nAngle(°)  | Torque(N⋅m) | Moment Arm(mm) | Tendon Length(m) | Sign")
    print("-" * 75)
    
    for angle in angles:
        # Reset
        mujoco.mj_resetData(model, data)
        
        # Set palm to neutral position
        data.qpos[palm_qpos_adr] = 0.0
        
        # Set L1/L2 angle
        data.qpos[tip_qpos_adr] = angle
        
        # Forward kinematics
        mujoco.mj_forward(model, data)
        
        # Get tendon moment arm (derivative of tendon length w.r.t. joint angle)
        # moment_arm = -dL/dq (negative because tendon shortens when joint closes)
        # ten_J is shape (ntendon, nv), so index properly
        moment_arm = -data.ten_J[tendon_id, tip_qpos_adr] if hasattr(data.ten_J, 'shape') else -data.ten_J[tendon_id * model.nv + tip_qpos_adr]
        
        # Get actuator torque at this joint
        # This is the torque the tendon would create with unit force
        torque = data.qfrc_actuator[tip_qpos_adr]
        
        # Get tendon length
        tendon_length = data.ten_length[tendon_id]
        
        torques.append(torque)
        moment_arms.append(moment_arm * 1000)  # Convert to mm
        tendon_lengths.append(tendon_length)
        
        # Print every 10 degrees
        if abs(np.rad2deg(angle) % 10) < 1.0:
            sign = "CLOSING" if moment_arm > 0 else "OPENING" if moment_arm < 0 else "ZERO"
            print(f"{np.rad2deg(angle):7.1f}  | {torque:11.6f} | {moment_arm*1000:14.3f} | {tendon_length:16.6f} | {sign}")
    
    # Find zero crossings (where tendon crosses joint axis)
    zero_crossings = []
    for i in range(1, len(moment_arms)):
        if moment_arms[i-1] * moment_arms[i] < 0:  # Sign change
            # Interpolate to find exact crossing angle
            crossing_angle = np.interp(0, [moment_arms[i-1], moment_arms[i]], 
                                      [np.rad2deg(angles[i-1]), np.rad2deg(angles[i])])
            zero_crossings.append(crossing_angle)
    
    print("\n" + "="*70)
    print("ANALYSIS RESULTS")
    print("="*70)
    
    if zero_crossings:
        print(f"\n⚠️  TENDON CROSSES JOINT AXIS at angles:")
        for crossing in zero_crossings:
            print(f"   {crossing:.2f}°")
        print("\nThis means the tendon switches from CLOSING to OPENING torque!")
        print("A pulley is needed to keep the tendon on the correct side.")
    else:
        print("\n✓ Tendon does NOT cross joint axis in tested range")
        print("  Moment arm maintains consistent sign")
    
    # Check sign at key angles
    limit_low_idx = np.argmin(np.abs(angles - model.jnt_range[f1_tip_id, 0]))
    limit_high_idx = np.argmin(np.abs(angles - model.jnt_range[f1_tip_id, 1]))
    zero_idx = np.argmin(np.abs(angles))
    
    print(f"\nMoment arm at key angles:")
    print(f"  At lower limit ({np.rad2deg(model.jnt_range[f1_tip_id, 0]):.1f}°): {moment_arms[limit_low_idx]:.3f} mm")
    print(f"  At zero (0°): {moment_arms[zero_idx]:.3f} mm")
    print(f"  At upper limit ({np.rad2deg(model.jnt_range[f1_tip_id, 1]):.1f}°): {moment_arms[limit_high_idx]:.3f} mm")
    
    # Check if moment arm is ALWAYS positive (closing) in allowed range
    min_moment_arm = min(moment_arms)
    max_moment_arm = max(moment_arms)
    
    print(f"\nMoment arm range in allowed limits:")
    print(f"  Minimum: {min_moment_arm:.3f} mm")
    print(f"  Maximum: {max_moment_arm:.3f} mm")
    
    if min_moment_arm < 0:
        print(f"\n⚠️  CRITICAL PROBLEM: Moment arm goes NEGATIVE in allowed range!")
        print(f"     Tendon creates OPENING torque instead of CLOSING!")
        print(f"     This explains why L1/L2 rotates backward!")
    elif min_moment_arm < 1.0:
        print(f"\n⚠️  WARNING: Moment arm is very small ({min_moment_arm:.3f} mm)")
        print(f"     Tendon has weak leverage at some angles")
    else:
        print(f"\n✓ GOOD: Moment arm is always POSITIVE (closing direction)")
        print(f"  Tendon will always pull joint closed")
    
    # Plot results
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
    
    # Plot 1: Moment arm vs angle
    ax1.plot(np.rad2deg(angles), moment_arms, 'b-', linewidth=2)
    ax1.axhline(y=0, color='r', linestyle='--', label='Zero line')
    ax1.axvline(x=np.rad2deg(model.jnt_range[f1_tip_id, 0]), color='g', linestyle='--', label='Lower limit')
    ax1.axvline(x=np.rad2deg(model.jnt_range[f1_tip_id, 1]), color='orange', linestyle='--', label='Upper limit')
    ax1.set_xlabel('L1/L2 Joint Angle (degrees)')
    ax1.set_ylabel('Moment Arm (mm)')
    ax1.set_title('Tendon Moment Arm vs Joint Angle\n(Positive = Closing, Negative = Opening)')
    ax1.grid(True)
    ax1.legend()
    
    # Shade the valid range
    ax1.axvspan(np.rad2deg(model.jnt_range[f1_tip_id, 0]), 
                np.rad2deg(model.jnt_range[f1_tip_id, 1]), 
                alpha=0.2, color='green', label='Valid range')
    
    # Plot 2: Torque vs angle
    ax2.plot(np.rad2deg(angles), torques, 'r-', linewidth=2)
    ax2.axhline(y=0, color='k', linestyle='--')
    ax2.axvline(x=np.rad2deg(model.jnt_range[f1_tip_id, 0]), color='g', linestyle='--')
    ax2.axvline(x=np.rad2deg(model.jnt_range[f1_tip_id, 1]), color='orange', linestyle='--')
    ax2.set_xlabel('L1/L2 Joint Angle (degrees)')
    ax2.set_ylabel('Actuator Torque (N⋅m)')
    ax2.set_title('Actuator Torque vs Joint Angle')
    ax2.grid(True)
    
    # Plot 3: Tendon length vs angle
    ax3.plot(np.rad2deg(angles), tendon_lengths, 'g-', linewidth=2)
    ax3.axvline(x=np.rad2deg(model.jnt_range[f1_tip_id, 0]), color='g', linestyle='--')
    ax3.axvline(x=np.rad2deg(model.jnt_range[f1_tip_id, 1]), color='orange', linestyle='--')
    ax3.set_xlabel('L1/L2 Joint Angle (degrees)')
    ax3.set_ylabel('Tendon Length (m)')
    ax3.set_title('Tendon Length vs Joint Angle')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('/home/sake/linorobot2_ws/src/ezgripper_sim/tendon_torque_analysis.png', dpi=150)
    print(f"\n✓ Plot saved to: tendon_torque_analysis.png")
    
    print("\n" + "="*70)

if __name__ == '__main__':
    main()
