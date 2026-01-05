#!/usr/bin/env python3
"""
Debug finger tip collision - check if tips are actually colliding.
"""

import mujoco
import numpy as np

# Load model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Get geom IDs
f1_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'f1_tip')
f2_tip_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'f2_tip')

print("=" * 70)
print("Finger Tip Collision Debug")
print("=" * 70)

# Check geom properties
print(f"\nf1_tip (ID {f1_tip_id}):")
print(f"  contype: {model.geom_contype[f1_tip_id]}")
print(f"  conaffinity: {model.geom_conaffinity[f1_tip_id]}")

print(f"\nf2_tip (ID {f2_tip_id}):")
print(f"  contype: {model.geom_contype[f2_tip_id]}")
print(f"  conaffinity: {model.geom_conaffinity[f2_tip_id]}")

# Check if they can collide
can_collide = (model.geom_contype[f1_tip_id] & model.geom_conaffinity[f2_tip_id]) != 0
print(f"\nCan f1_tip and f2_tip collide? {can_collide}")
print(f"  f1_tip contype ({model.geom_contype[f1_tip_id]}) & f2_tip conaffinity ({model.geom_conaffinity[f2_tip_id]}) = {model.geom_contype[f1_tip_id] & model.geom_conaffinity[f2_tip_id]}")

# Simulate closing
actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
data.ctrl[actuator_id] = 100.0  # Full force

print("\nSimulating gripper closing...")
for i in range(2000):
    mujoco.mj_step(model, data)

# Check for contacts
print(f"\nTotal contacts: {data.ncon}")

tip_contacts = 0
for i in range(data.ncon):
    contact = data.contact[i]
    if (contact.geom1 == f1_tip_id and contact.geom2 == f2_tip_id) or \
       (contact.geom1 == f2_tip_id and contact.geom2 == f1_tip_id):
        tip_contacts += 1
        print(f"  ✓ Found tip-to-tip contact!")
        print(f"    Contact {i}: geom1={contact.geom1}, geom2={contact.geom2}")
        print(f"    Distance: {contact.dist:.6f}")

if tip_contacts == 0:
    print("  ✗ NO tip-to-tip contacts found!")
    print("\n  Other contacts:")
    for i in range(min(data.ncon, 10)):
        contact = data.contact[i]
        try:
            geom1_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            print(f"    Contact {i}: {geom1_name} <-> {geom2_name}")
        except:
            print(f"    Contact {i}: geom1={contact.geom1}, geom2={contact.geom2}")

print("\n" + "=" * 70)
