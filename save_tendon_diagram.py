#!/usr/bin/env python3
"""
Save a diagram showing the full tendon paths with pegs, pins, and pulleys.
"""

import mujoco
import numpy as np
from PIL import Image

def main():
    print("="*70)
    print("SAVING TENDON PATH DIAGRAM")
    print("="*70)
    
    # Load model
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    
    # Set gripper to neutral/open position for clear view
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    
    # Setup renderer
    renderer = mujoco.Renderer(model, height=480, width=640)
    
    # Configure visualization - show tendons, sites, and pulleys
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[0] = False  # Hide visual meshes
    scene_option.geomgroup[1] = True   # Show sites/tendons (red)
    scene_option.geomgroup[2] = True   # Show collision geoms
    
    # Multiple camera angles
    views = [
        {
            'name': 'top_view',
            'distance': 0.35,
            'azimuth': 90,
            'elevation': -90,
            'lookat': [0.08, 0, 0.1]
        },
        {
            'name': 'front_view',
            'distance': 0.4,
            'azimuth': 0,
            'elevation': 0,
            'lookat': [0.08, 0, 0.1]
        },
        {
            'name': 'side_view',
            'distance': 0.4,
            'azimuth': 90,
            'elevation': -15,
            'lookat': [0.08, 0, 0.1]
        },
        {
            'name': 'angled_view',
            'distance': 0.45,
            'azimuth': 45,
            'elevation': -30,
            'lookat': [0.08, 0, 0.1]
        }
    ]
    
    for view in views:
        camera = mujoco.MjvCamera()
        camera.distance = view['distance']
        camera.azimuth = view['azimuth']
        camera.elevation = view['elevation']
        camera.lookat[:] = view['lookat']
        
        # Render
        renderer.update_scene(data, camera, scene_option)
        pixels = renderer.render()
        
        # Save image
        img = Image.fromarray(pixels)
        filename = f"tendon_diagram_{view['name']}.png"
        img.save(filename)
        print(f"Saved: {filename}")
    
    print("\n" + "="*70)
    print("All tendon diagrams saved!")
    print("="*70)
    
    # Print tendon path information
    print("\nTENDON PATH SUMMARY:")
    print("-" * 70)
    
    tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    tendon_adr = model.tendon_adr[tendon_id]
    num_wraps = model.tendon_num[tendon_id]
    
    print(f"\nFinger 1 Tendon ({num_wraps} waypoints):")
    for i in range(num_wraps):
        wrap_id = tendon_adr + i
        wrap_type = model.wrap_type[wrap_id]
        obj_id = model.wrap_objid[wrap_id]
        
        if wrap_type == 0:  # Site
            site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, obj_id)
            site_pos = model.site_pos[obj_id]
            print(f"  {i+1}. Site: {site_name:20s} at ({site_pos[0]*1000:6.2f}, {site_pos[1]*1000:6.2f}, {site_pos[2]*1000:6.2f}) mm")
        elif wrap_type == 1:  # Geom (pulley)
            geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, obj_id)
            geom_pos = model.geom_pos[obj_id]
            sidesite_id = model.wrap_sideid[wrap_id]
            if sidesite_id >= 0:
                sidesite_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, sidesite_id)
                print(f"  {i+1}. Pulley: {geom_name:18s} at ({geom_pos[0]*1000:6.2f}, {geom_pos[1]*1000:6.2f}, {geom_pos[2]*1000:6.2f}) mm (sidesite: {sidesite_name})")
    
    print("-" * 70)

if __name__ == '__main__':
    main()
