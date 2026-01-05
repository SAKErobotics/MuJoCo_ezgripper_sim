#!/usr/bin/env python3
"""
Visualize tendon path with meshes hidden - shows sites and tendon routing clearly.
"""

import mujoco
import mujoco.viewer
import numpy as np

# Load the model
model = mujoco.MjModel.from_xml_path('ezgripper.xml')
data = mujoco.MjData(model)

print("\n" + "="*70)
print("TENDON PATH VISUALIZATION")
print("="*70)
print("Opening viewer with:")
print("- Meshes hidden (only collision geoms visible)")
print("- Sites visible (red spheres)")
print("- Tendon path visible")
print("\nControls:")
print("- Right-click drag: rotate view")
print("- Scroll: zoom")
print("- Press ESC to exit")
print("="*70 + "\n")

# Set joints to neutral position
data.qpos[:] = 0
mujoco.mj_forward(model, data)

# Launch viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    # Configure visualization
    viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
    
    # Hide visual geoms, show collision geoms
    viewer.vopt.geomgroup[0] = False  # Hide visual geoms (group 0)
    viewer.vopt.geomgroup[1] = True   # Show sites and tendons (group 1)
    viewer.vopt.geomgroup[2] = True   # Show collision geoms
    
    # Keep viewer open
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
