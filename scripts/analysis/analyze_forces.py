#!/usr/bin/env python3
"""
Analyze forces and constraints preventing gripper from opening.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("FORCE ANALYSIS - Why isn't the gripper opening?")
print("="*80)

# Set tendon to maximum length to avoid constraint
data.ctrl[:] = 0.180

# Run for a few steps to settle
for _ in range(1000):
    mujoco.mj_step(model, data)

# Get joint states
f1_palm = data.qpos[0]
f1_tip = data.qpos[1]

print(f"\nCurrent Joint Positions:")
print(f"  F1_palm: {np.rad2deg(f1_palm):.2f}° (range: -90° to 25°)")
print(f"  F1_tip:  {np.rad2deg(f1_tip):.2f}° (range: 0° to 97°)")

# Calculate spring torques
spring_torque_palm = -model.jnt_stiffness[0] * (f1_palm - model.qpos_spring[0])
spring_torque_tip = -model.jnt_stiffness[1] * (f1_tip - model.qpos_spring[1])

print(f"\nSpring Configuration:")
print(f"  Palm-L1: stiffness={model.jnt_stiffness[0]:.6f} N·m/rad, springref={model.qpos_spring[0]:.4f} rad ({np.rad2deg(model.qpos_spring[0]):.1f}°)")
print(f"  L1-L2:   stiffness={model.jnt_stiffness[1]:.6f} N·m/rad, springref={model.qpos_spring[1]:.4f} rad ({np.rad2deg(model.qpos_spring[1]):.1f}°)")

print(f"\nSpring Torques (negative = opening direction):")
print(f"  Palm-L1: {spring_torque_palm:+.6f} N·m")
print(f"  L1-L2:   {spring_torque_tip:+.6f} N·m")

# Get tendon info
mujoco.mj_tendon(model, data)
tendon1_length = data.ten_length[0] * 1000
tendon1_force = data.actuator_force[0]

print(f"\nTendon State:")
print(f"  Commanded length: {data.ctrl[0]*1000:.2f} mm")
print(f"  Actual length:    {tendon1_length:.2f} mm")
print(f"  Actuator force:   {tendon1_force:.2f} N")
print(f"  Range: {model.actuator_ctrlrange[0,0]*1000:.0f} to {model.actuator_ctrlrange[0,1]*1000:.0f} mm")

# Calculate tendon moment arms (how much torque the tendon creates per unit force)
# This requires computing the Jacobian
mujoco.mj_forward(model, data)

print(f"\nTendon Moment Arms:")
print(f"  (Calculated by numerical differentiation)")

# Calculate moment arms numerically
epsilon = 0.001  # Small angle change
original_qpos = data.qpos.copy()

# Palm-L1 moment arm
data.qpos[0] = original_qpos[0] + epsilon
mujoco.mj_kinematics(model, data)
mujoco.mj_tendon(model, data)
length_plus = data.ten_length[0]

data.qpos[0] = original_qpos[0] - epsilon
mujoco.mj_kinematics(model, data)
mujoco.mj_tendon(model, data)
length_minus = data.ten_length[0]

moment_arm_palm = (length_plus - length_minus) / (2 * epsilon)
data.qpos[:] = original_qpos

# L1-L2 moment arm
data.qpos[1] = original_qpos[1] + epsilon
mujoco.mj_kinematics(model, data)
mujoco.mj_tendon(model, data)
length_plus = data.ten_length[0]

data.qpos[1] = original_qpos[1] - epsilon
mujoco.mj_kinematics(model, data)
mujoco.mj_tendon(model, data)
length_minus = data.ten_length[0]

moment_arm_tip = (length_plus - length_minus) / (2 * epsilon)
data.qpos[:] = original_qpos

print(f"  Tendon 1 w.r.t. Palm-L1: {moment_arm_palm:.6f} m/rad")
print(f"  Tendon 1 w.r.t. L1-L2:   {moment_arm_tip:.6f} m/rad")

# Calculate effective torques from tendon
tendon_torque_palm = -tendon1_force * moment_arm_palm  # Negative because tendon pulls
tendon_torque_tip = -tendon1_force * moment_arm_tip

print(f"\nTendon Torques (from actuator force):")
print(f"  Palm-L1: {tendon_torque_palm:+.6f} N·m")
print(f"  L1-L2:   {tendon_torque_tip:+.6f} N·m")

# Total torques
total_palm = spring_torque_palm + tendon_torque_palm
total_tip = spring_torque_tip + tendon_torque_tip

print(f"\nTotal Torques (spring + tendon):")
print(f"  Palm-L1: {total_palm:+.6f} N·m")
print(f"  L1-L2:   {total_tip:+.6f} N·m")

# Check constraint forces
print(f"\nConstraint Forces:")
print(f"  Number of active constraints: {data.nefc}")
if data.nefc > 0:
    print(f"  Constraint forces (first 10):")
    for i in range(min(10, data.nefc)):
        print(f"    [{i}]: {data.efc_force[i]:.4f} N")

# Analyze why gripper won't open
print("\n" + "="*80)
print("DIAGNOSIS:")
print("="*80)

if abs(tendon1_length - data.ctrl[0]*1000) > 10:
    print("❌ PROBLEM: Tendon length doesn't match command!")
    print(f"   Commanded: {data.ctrl[0]*1000:.2f} mm, Actual: {tendon1_length:.2f} mm")
    print(f"   Difference: {tendon1_length - data.ctrl[0]*1000:.2f} mm")
    print("   → Tendon is physically constrained and cannot reach commanded length")
    print("   → This prevents springs from opening the gripper")

if abs(spring_torque_palm) < 0.1:
    print("⚠️  Spring torque is weak (< 0.1 N·m)")
    print("   → May not be strong enough to overcome friction/damping")

if abs(tendon1_force) > 10:
    print(f"⚠️  High tendon force ({tendon1_force:.1f} N) even at max length")
    print("   → Tendon is under tension, resisting spring opening force")

# Check moment arms
if abs(moment_arm_palm) < 0.001:
    print("❌ PROBLEM: Palm-L1 moment arm is near zero!")
    print("   → Tendon has no mechanical advantage at this joint")

if abs(moment_arm_tip) < 0.001:
    print("❌ PROBLEM: L1-L2 moment arm is near zero!")
    print("   → Tendon has no mechanical advantage at this joint")

print("\n" + "="*80)
print("RECOMMENDATION:")
print("="*80)
print("The L2 pulley is essential for:")
print("  1. Providing correct tendon path length for full range of motion")
print("  2. Creating proper moment arm at L1-L2 joint")
print("  3. Allowing springs to push fingers to fully open position")
print("\nWithout the L2 pulley, the tendon path is too short and constrains")
print("the gripper from opening, even when commanded to maximum length.")
print("="*80)
