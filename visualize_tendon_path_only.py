#!/usr/bin/env python3
"""
Visualize ONLY the tendon path - no meshes, just lines and sites.
"""

import mujoco
import numpy as np
from PIL import Image, ImageDraw

def main():
    print("="*70)
    print("TENDON PATH VISUALIZATION")
    print("="*70)
    
    # Load model
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    
    # Set gripper to neutral position
    data.qpos[:] = 0
    mujoco.mj_forward(model, data)
    
    # Print text-based tendon path
    print("\n" + "="*70)
    print("FINGER 1 TENDON PATH (from XML)")
    print("="*70)
    
    tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, 'finger1_tendon')
    tendon_adr = model.tendon_adr[tendon_id]
    num_wraps = model.tendon_num[tendon_id]
    
    print(f"\nTotal waypoints: {num_wraps}\n")
    
    for i in range(num_wraps):
        wrap_id = tendon_adr + i
        wrap_type = model.wrap_type[wrap_id]
        obj_id = model.wrap_objid[wrap_id]
        
        if wrap_type == 0:  # Site
            site_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, obj_id)
            site_pos = model.site_pos[obj_id]
            print(f"{i+1:2d}. SITE:   {site_name:20s}")
            print(f"    Position: X={site_pos[0]*1000:7.2f}mm  Y={site_pos[1]*1000:7.2f}mm  Z={site_pos[2]*1000:7.2f}mm")
            
        elif wrap_type == 1:  # Geom (pulley)
            geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, obj_id)
            geom_pos = model.geom_pos[obj_id]
            geom_size = model.geom_size[obj_id]
            
            sidesite_id = model.wrap_sideid[wrap_id]
            if sidesite_id >= 0:
                sidesite_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, sidesite_id)
                sidesite_pos = model.site_pos[sidesite_id]
                print(f"{i+1:2d}. PULLEY: {geom_name:20s} (radius={geom_size[0]*1000:.2f}mm)")
                print(f"    Position: X={geom_pos[0]*1000:7.2f}mm  Y={geom_pos[1]*1000:7.2f}mm  Z={geom_pos[2]*1000:7.2f}mm")
                print(f"    Sidesite: {sidesite_name:20s}")
                print(f"              X={sidesite_pos[0]*1000:7.2f}mm  Y={sidesite_pos[1]*1000:7.2f}mm  Z={sidesite_pos[2]*1000:7.2f}mm")
        print()
    
    print("="*70)
    print("\nTendon path flow:")
    print("  Servo → palm_peg0 → palm_peg1 → palm_pulley_f1 (wraps) →")
    print("  f1l1_peg0 → f1l1_pulley (wraps) → f1l1_peg1 →")
    print("  f1l2_pulley (wraps) → f1l2_peg → f1l2_pin (fingertip)")
    print("="*70)
    
    # Create visual diagram with renderer
    print("\nCreating visual diagrams...")
    renderer = mujoco.Renderer(model, height=480, width=640)
    
    # Configure to show ONLY tendons and sites
    scene_option = mujoco.MjvOption()
    scene_option.geomgroup[0] = False  # Hide visual meshes
    scene_option.geomgroup[1] = True   # Show sites/tendons
    scene_option.geomgroup[2] = False  # Hide collision geoms
    scene_option.geomgroup[3] = False  # Hide everything else
    
    # Enable tendon rendering and joint visualization
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = True
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = True
    
    views = [
        {'name': 'top', 'distance': 0.3, 'azimuth': 90, 'elevation': -89},
        {'name': 'side', 'distance': 0.35, 'azimuth': 90, 'elevation': -15},
        {'name': 'front', 'distance': 0.35, 'azimuth': 0, 'elevation': 0},
    ]
    
    # Get joint positions in world coordinates
    joint_names = ['F1_palm_knuckle', 'F2_palm_knuckle', 'F1_knuckle_tip', 'F2_knuckle_tip']
    joint_positions = []
    print("\nJoint positions:")
    for jname in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        # Get the actual joint anchor position in world coordinates
        jpos = data.xanchor[jid].copy()
        joint_positions.append(jpos)
        print(f"  {jname}: X={jpos[0]*1000:.2f}mm Y={jpos[1]*1000:.2f}mm Z={jpos[2]*1000:.2f}mm")
    
    # Also check L2 pulley positions for comparison
    print("\nL2 Pulley positions (should match L1-L2 joints):")
    for pulley_name in ['f1l2_pulley', 'f2l2_pulley']:
        geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, pulley_name)
        pulley_pos = data.geom_xpos[geom_id].copy()
        print(f"  {pulley_name}: X={pulley_pos[0]*1000:.2f}mm Y={pulley_pos[1]*1000:.2f}mm Z={pulley_pos[2]*1000:.2f}mm")
    
    for view in views:
        camera = mujoco.MjvCamera()
        camera.distance = view['distance']
        camera.azimuth = view['azimuth']
        camera.elevation = view['elevation']
        camera.lookat[:] = [0.08, 0, 0.1]
        
        renderer.update_scene(data, camera, scene_option)
        pixels = renderer.render()
        
        img = Image.fromarray(pixels)
        draw = ImageDraw.Draw(img)
        
        # Grid removed - calibration complete and verified!
        
        # Draw circles at joint locations (only for top view)
        if view['name'] == 'top':
            for i, jpos in enumerate(joint_positions):
                # Simple orthographic projection for top view
                # Map world X,Y to screen coordinates
                x_world = jpos[0]
                y_world = jpos[1]
                
                # Projection for top view (azimuth=90, elevation=-89)
                # Recalibrated with grid to match L2 red pulleys:
                # Palm joints: X=72.55mm at screen x=305
                # L2 pulleys (red circles): X=124.55mm at screen x=395
                # Difference: 52mm → 90 pixels → scale_x = 1730
                # Palm Y=+30mm at screen y=180, Y=-30mm at screen y=300
                # But L2 pulleys are at y=175 and y=305
                # Difference: 130 pixels for 60mm → scale_y = 2167
                
                scale_x = 1730  # horizontal scale (X-axis in world → screen X)
                scale_y = 2167  # vertical scale (Y-axis in world → screen Y)
                center_x = 305  # horizontal center (palm joint location)
                center_y = 240  # vertical center
                
                x_screen = center_x + (x_world - 0.07255) * scale_x  # X becomes horizontal
                y_screen = center_y - (y_world - 0.0) * scale_y  # Y becomes vertical (inverted)
                
                # Draw yellow circle with black outline
                radius = 10
                draw.ellipse([x_screen-radius, y_screen-radius, 
                             x_screen+radius, y_screen+radius], 
                            outline='yellow', width=3, fill=None)
                
                print(f"  Joint {joint_names[i]}: screen=({x_screen:.0f}, {y_screen:.0f})")
        
        filename = f"tendon_path_{view['name']}.png"
        img.save(filename)
        print(f"  Saved: {filename}")
    
    print("\n" + "="*70)
    print("Done!")
    print("="*70)

if __name__ == '__main__':
    main()
