#!/usr/bin/env python3
"""
Diagnose why fingers are not synchronized despite identical commands.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("FINGER ASYMMETRY DIAGNOSIS")
print("="*80)

# Get IDs
f1_palm_id = model.joint('F1_palm_knuckle').id
f1_tip_id = model.joint('F1_knuckle_tip').id
f2_palm_id = model.joint('F2_palm_knuckle').id
f2_tip_id = model.joint('F2_knuckle_tip').id

f1_actuator_id = model.actuator('gripper_actuator_f1').id
f2_actuator_id = model.actuator('gripper_actuator_f2').id

print("\nJOINT PARAMETERS COMPARISON:")
print("-" * 50)

# Compare joint parameters
f1_palm_joint = model.joint(f1_palm_id)
f2_palm_joint = model.joint(f2_palm_id)

print(f"Palm Joint Range:")
print(f"  F1: [{np.degrees(f1_palm_joint.range[0]):6.1f}°, {np.degrees(f1_palm_joint.range[1]):6.1f}°]")
print(f"  F2: [{np.degrees(f2_palm_joint.range[0]):6.1f}°, {np.degrees(f2_palm_joint.range[1]):6.1f}°]")

print(f"\nPalm Joint Stiffness:")
print(f"  F1: {model.jnt_stiffness[f1_palm_id]:.6f}")
print(f"  F2: {model.jnt_stiffness[f2_palm_id]:.6f}")

print(f"\nPalm Joint Springref:")
print(f"  F1: {np.degrees(model.jnt_springref[f1_palm_id]):6.1f}°")
print(f"  F2: {np.degrees(model.jnt_springref[f2_palm_id]):6.1f}°")

print(f"\nPalm Joint Damping:")
print(f"  F1: {model.jnt_damping[f1_palm_id]:.6f}")
print(f"  F2: {model.jnt_damping[f2_palm_id]:.6f}")

print(f"\nPalm Joint Axis:")
print(f"  F1: [{f1_palm_joint.axis[0]:.1f}, {f1_palm_joint.axis[1]:.1f}, {f1_palm_joint.axis[2]:.1f}]")
print(f"  F2: [{f2_palm_joint.axis[0]:.1f}, {f2_palm_joint.axis[1]:.1f}, {f2_palm_joint.axis[2]:.1f}]")

# Compare L1-L2 joints
f1_tip_joint = model.joint(f1_tip_id)
f2_tip_joint = model.joint(f2_tip_id)

print(f"\nL1-L2 Joint Range:")
print(f"  F1: [{np.degrees(f1_tip_joint.range[0]):6.1f}°, {np.degrees(f1_tip_joint.range[1]):6.1f}°]")
print(f"  F2: [{np.degrees(f2_tip_joint.range[0]):6.1f}°, {np.degrees(f2_tip_joint.range[1]):6.1f}°]")

print(f"\nL1-L2 Joint Stiffness:")
print(f"  F1: {model.jnt_stiffness[f1_tip_id]:.6f}")
print(f"  F2: {model.jnt_stiffness[f2_tip_id]:.6f}")

print(f"\nL1-L2 Joint Springref:")
print(f"  F1: {np.degrees(model.jnt_springref[f1_tip_id]):6.1f}°")
print(f"  F2: {np.degrees(model.jnt_springref[f2_tip_id]):6.1f}°")

print(f"\nL1-L2 Joint Damping:")
print(f"  F1: {model.jnt_damping[f1_tip_id]:.6f}")
print(f"  F2: {model.jnt_damping[f2_tip_id]:.6f}")

print(f"\nL1-L2 Joint Axis:")
print(f"  F1: [{f1_tip_joint.axis[0]:.1f}, {f1_tip_joint.axis[1]:.1f}, {f1_tip_joint.axis[2]:.1f}]")
print(f"  F2: [{f2_tip_joint.axis[0]:.1f}, {f2_tip_joint.axis[1]:.1f}, {f2_tip_joint.axis[2]:.1f}]")

print("\n" + "="*50)
print("TENDON PARAMETERS COMPARISON")
print("="*50)

# Compare tendon parameters
f1_tendon = model.tendon(model.tendon('finger1_tendon').id)
f2_tendon = model.tendon(model.tendon('finger2_tendon').id)

print(f"\nTendon Range:")
print(f"  F1: [{f1_tendon.range[0]:.3f}, {f1_tendon.range[1]:.3f}]")
print(f"  F2: [{f2_tendon.range[0]:.3f}, {f2_tendon.range[1]:.3f}]")

print(f"\nTendon Limited:")
print(f"  F1: {f1_tendon.limited}")
print(f"  F2: {f2_tendon.limited}")

print(f"\nTendon Stiffness:")
print(f"  F1: {float(f1_tendon.stiffness):.6f}")
print(f"  F2: {float(f2_tendon.stiffness):.6f}")

print(f"\nTendon Damping:")
print(f"  F1: {float(f1_tendon.damping):.6f}")
print(f"  F2: {float(f2_tendon.damping):.6f}")

print(f"\nTendon Frictionloss:")
print(f"  F1: {float(f1_tendon.frictionloss):.6f}")
print(f"  F2: {float(f2_tendon.frictionloss):.6f}")

print(f"\nNumber of tendon sites:")
print(f"  F1: {f1_tendon.num}")
print(f"  F2: {f2_tendon.num}")

print("\n" + "="*50)
print("ACTUATOR PARAMETERS COMPARISON")
print("="*50)

# Compare actuator parameters
f1_actuator = model.actuator(f1_actuator_id)
f2_actuator = model.actuator(f2_actuator_id)

print(f"\nActuator Control Range:")
print(f"  F1: [{f1_actuator.ctrlrange[0]:.3f}, {f1_actuator.ctrlrange[1]:.3f}]")
print(f"  F2: [{f2_actuator.ctrlrange[0]:.3f}, {f2_actuator.ctrlrange[1]:.3f}]")

print(f"\nActuator Gains (kp, kv):")
print(f"  F1: kp={f1_actuator.gainprm[0]:.1f}, kv={f1_actuator.gainprm[1]:.1f}")
print(f"  F2: kp={f2_actuator.gainprm[0]:.1f}, kv={f2_actuator.gainprm[1]:.1f}")

print(f"\nActuator Force Range:")
print(f"  F1: [{f1_actuator.forcerange[0]:.1f}, {f1_actuator.forcerange[1]:.1f}]")
print(f"  F2: [{f2_actuator.forcerange[0]:.1f}, {f2_actuator.forcerange[1]:.1f}]")

print("\n" + "="*50)
print("DYNAMIC TEST - IDENTICAL COMMANDS")
print("="*50)

# Test with identical commands
data.ctrl[f1_actuator_id] = 0.150
data.ctrl[f2_actuator_id] = 0.150

# Reset and run for 100 steps
mujoco.mj_resetData(model, data)
data.ctrl[f1_actuator_id] = 0.150
data.ctrl[f2_actuator_id] = 0.150

for i in range(100):
    mujoco.mj_step(model, data)
    
    if i % 20 == 0:
        f1_palm = np.degrees(data.qpos[f1_palm_id])
        f1_tip = np.degrees(data.qpos[f1_tip_id])
        f2_palm = np.degrees(data.qpos[f2_palm_id])
        f2_tip = np.degrees(data.qpos[f2_tip_id])
        
        f1_tendon_len = data.ten_length[0]
        f2_tendon_len = data.ten_length[1]
        
        print(f"Step {i:3d}:")
        print(f"  F1: Palm={f1_palm:6.1f}°, Tip={f1_tip:6.1f}°, Tendon={f1_tendon_len:.3f}")
        print(f"  F2: Palm={f2_palm:6.1f}°, Tip={f2_tip:6.1f}°, Tendon={f2_tendon_len:.3f}")
        print(f"  Diff: Palm={f1_palm-f2_palm:6.1f}°, Tip={f1_tip-f2_tip:6.1f}°, Tendon={f1_tendon_len-f2_tendon_len:.3f}")

print("\n" + "="*80)
print("DIAGNOSIS COMPLETE")
print("="*80)
