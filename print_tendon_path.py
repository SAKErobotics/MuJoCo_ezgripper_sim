#!/usr/bin/env python3
"""
Print the tendon path coordinates to verify routing.
"""

import mujoco
import numpy as np

# Load the model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Initialize to neutral position
data.qpos[:] = 0
mujoco.mj_forward(model, data)

print("\n" + "="*80)
print("TENDON PATH COORDINATES (in meters)")
print("="*80)

# Find finger1_tendon
tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
print(f"\nTendon: finger1_tendon (id={tendon_id})")

# Get tendon wrap information
tendon_adr = model.tendon_adr[tendon_id]
num_wraps = model.tendon_num[tendon_id]

print(f"Number of wrap points: {num_wraps}\n")
print(f"{'#':<4} {'Type':<10} {'Name':<25} {'X (mm)':<10} {'Y (mm)':<10} {'Z (mm)':<10}")
print("-" * 80)

for i in range(num_wraps):
    wrap_id = tendon_adr + i
    wrap_type = model.wrap_type[wrap_id]
    obj_id = model.wrap_objid[wrap_id]
    
    if wrap_type == 0:  # Site
        site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, obj_id)
        site_pos = model.site_pos[obj_id]
        print(f"{i+1:<4} {'Site':<10} {site_name:<25} {site_pos[0]*1000:>9.3f} {site_pos[1]*1000:>9.3f} {site_pos[2]*1000:>9.3f}")
        
    elif wrap_type == 1:  # Geom (pulley)
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, obj_id)
        geom_pos = model.geom_pos[obj_id]
        
        # Get sidesite if it exists
        sidesite_id = model.wrap_sideid[wrap_id]
        if sidesite_id >= 0:
            sidesite_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, sidesite_id)
            sidesite_pos = model.site_pos[sidesite_id]
            print(f"{i+1:<4} {'Pulley':<10} {geom_name:<25} {geom_pos[0]*1000:>9.3f} {geom_pos[1]*1000:>9.3f} {geom_pos[2]*1000:>9.3f}")
            print(f"     {'└─side':<10} {sidesite_name:<25} {sidesite_pos[0]*1000:>9.3f} {sidesite_pos[1]*1000:>9.3f} {sidesite_pos[2]*1000:>9.3f}")
        else:
            print(f"{i+1:<4} {'Pulley':<10} {geom_name:<25} {geom_pos[0]*1000:>9.3f} {geom_pos[1]*1000:>9.3f} {geom_pos[2]*1000:>9.3f}")

print("="*80)

# Calculate tendon length
mujoco.mj_forward(model, data)
tendon_length = data.ten_length[tendon_id]
print(f"\nTendon length at neutral position: {tendon_length*1000:.2f} mm")
print("="*80 + "\n")
