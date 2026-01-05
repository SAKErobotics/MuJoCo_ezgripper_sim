#!/usr/bin/env python3
"""
Visualize the tendon path without the full model rendering.
Shows tendon sites and routing through pulleys.
"""

import mujoco
import mujoco.viewer
import numpy as np

# Load the model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

# Get tendon information
print("\n" + "="*70)
print("TENDON PATH VISUALIZATION")
print("="*70)

# Find finger1_tendon
tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
print(f"\nTendon: finger1_tendon (id={tendon_id})")

# Get tendon wrap information
tendon = model.tendon_adr[tendon_id]
num_wraps = model.tendon_num[tendon_id]

print(f"Number of wrap points: {num_wraps}")
print("\nTendon routing:")

for i in range(num_wraps):
    wrap_id = tendon + i
    wrap_type = model.wrap_type[wrap_id]
    obj_id = model.wrap_objid[wrap_id]
    
    if wrap_type == 0:  # Site
        site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, obj_id)
        site_pos = model.site_pos[obj_id]
        print(f"  {i+1}. Site: {site_name:20s} at ({site_pos[0]:7.4f}, {site_pos[1]:7.4f}, {site_pos[2]:7.4f})")
    elif wrap_type == 1:  # Geom (pulley)
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, obj_id)
        # Get sidesite if it exists
        sidesite_id = model.wrap_sideid[wrap_id]
        if sidesite_id >= 0:
            sidesite_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, sidesite_id)
            sidesite_pos = model.site_pos[sidesite_id]
            print(f"  {i+1}. Pulley: {geom_name:18s} (sidesite: {sidesite_name} at ({sidesite_pos[0]:7.4f}, {sidesite_pos[1]:7.4f}, {sidesite_pos[2]:7.4f}))")
        else:
            print(f"  {i+1}. Pulley: {geom_name}")

print("\n" + "="*70)
print("Opening interactive viewer...")
print("- Red spheres show tendon sites")
print("- Red cylinders show pulleys")
print("- Press ESC to exit")
print("="*70 + "\n")

# Launch viewer with visualization options
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Enable visualization
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
    
    # Set joint to neutral position to see routing clearly
    data.qpos[:] = 0
    
    # Step once to update visualization
    mujoco.mj_forward(model, data)
    
    # Keep viewer open
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
