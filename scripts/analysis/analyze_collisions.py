#!/usr/bin/env python3
"""
Analyze all collision sources in the current EZGripper model.
"""

import mujoco
import numpy as np
import os

# Load model
model_path = os.path.join(os.path.dirname(__file__), 'ezgripper.xml')
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

print("="*80)
print("COLLISION ANALYSIS - CURRENT MODEL")
print("="*80)

print(f"\nTOTAL GEOMS: {model.ngeom}")
print(f"TOTAL BODIES: {model.nbody}")

print("\n" + "="*50)
print("ALL GEOMS WITH COLLISION PROPERTIES")
print("="*50)

for i in range(model.ngeom):
    name = model.geom(i).name if model.geom(i).name else f"geom_{i}"
    contype = model.geom_contype[i]
    conaffinity = model.geom_conaffinity[i]
    body = model.body(model.geom_bodyid[i]).name if model.body(model.geom_bodyid[i]).name else f"body_{model.geom_bodyid[i]}"
    
    print(f"\nGeom {i}: {name}")
    print(f"  Body: {body}")
    print(f"  Contype: {contype} (binary: {bin(contype)})")
    print(f"  Conaffinity: {conaffinity} (binary: {bin(conaffinity)})")
    
    if contype == 0 and conaffinity == 0:
        print(f"  → NO COLLISION (disabled)")
    else:
        print(f"  → COLLISION ENABLED")

print("\n" + "="*50)
print("COLLISION MATRIX SUMMARY")
print("="*50)

# Group by contype/conaffinity patterns
collision_groups = {}
for i in range(model.ngeom):
    name = model.geom(i).name if model.geom(i).name else f"geom_{i}"
    contype = model.geom_contype[i]
    conaffinity = model.geom_conaffinity[i]
    
    key = (contype, conaffinity)
    if key not in collision_groups:
        collision_groups[key] = []
    collision_groups[key].append((i, name))

for (contype, conaffinity), geoms in collision_groups.items():
    print(f"\nContype={contype}, Conaffinity={conaffinity}:")
    for geom_id, geom_name in geoms:
        print(f"  Geom {geom_id}: {geom_name}")

print("\n" + "="*50)
print("CURRENT CONTACTS")
print("="*50)

# Reset and step once to see contacts
mujoco.mj_resetData(model, data)
mujoco.mj_step(model, data)

print(f"Active contacts: {data.ncon}")
for i in range(data.ncon):
    contact = data.contact[i]
    geom1_name = model.geom(contact.geom1).name if model.geom(contact.geom1).name else f"geom_{contact.geom1}"
    geom2_name = model.geom(contact.geom2).name if model.geom(contact.geom2).name else f"geom_{contact.geom2}"
    print(f"  Contact {i}: {geom1_name} ↔ {geom2_name} (dist={contact.dist:.4f})")

print("\n" + "="*80)
print("COLLISION DESIGN ANALYSIS")
print("="*80)
print("Based on recent commits, the collision system uses:")
print("1. Mechanical stops (contype 4 & 5, 6)")
print("2. Finger mesh collisions (contype 3)")
print("3. Palm collision surfaces")
print("\nKey collision groups:")
print("- contype=3: Finger mesh collisions (fingers collide with each other)")
print("- contype=4: Palm-L1 mechanical stops")
print("- contype=5: L1-L2 mechanical stops") 
print("- contype=6: Palm upper limit stops")
print("="*80)
