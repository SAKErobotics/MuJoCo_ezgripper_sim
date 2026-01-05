#!/usr/bin/env python3
"""
Generate static images showing EZGripper grasping with visible L1 tendon structure.
Shows the internal tendon routing through L1_face.stl mesh.
"""

import mujoco
import numpy as np
from PIL import Image

def create_tendon_visibility_images():
    """Generate images showing tendon structure in L1 during grasping"""

    # Load the model with L1_face.stl
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=800, width=1200)

    # Visualization options
    scene_option = mujoco.MjvOption()
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = True  # Show tendons
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
    scene_option.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = False

    print("Generating images showing L1 tendon structure during grasping...")

    # Define grasping sequence
    grasp_sequence = [
        {'name': 'open', 'tendon_force': 0.0, 'description': 'Open position'},
        {'name': 'mid_grasp', 'tendon_force': 25.0, 'description': 'Mid-grasp position'},
        {'name': 'closed', 'tendon_force': 50.0, 'description': 'Fully closed position'}
    ]

    # Camera views to show tendon structure
    views = [
        {'name': 'side', 'azimuth': 90, 'elevation': -10, 'distance': 0.25, 'lookat': [0.08, 0, 0.02]},
        {'name': 'front', 'azimuth': 0, 'elevation': -5, 'distance': 0.25, 'lookat': [0.08, 0, 0.02]},
        {'name': 'top', 'azimuth': 90, 'elevation': -85, 'distance': 0.25, 'lookat': [0.08, 0, 0.02]},
        {'name': 'angled', 'azimuth': 45, 'elevation': -30, 'distance': 0.25, 'lookat': [0.08, 0, 0.02]}
    ]

    # Generate images for each grasp position and view
    for grasp in grasp_sequence:
        print(f"  Generating {grasp['name']} position: {grasp['description']}")

        # Set tendon force
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
        data.ctrl[actuator_id] = grasp['tendon_force']

        # Step physics to settle
        for _ in range(100):
            mujoco.mj_step(model, data)

        # Generate image for each view
        for view in views:
            camera = mujoco.MjvCamera()
            camera.distance = view['distance']
            camera.azimuth = view['azimuth']
            camera.elevation = view['elevation']
            camera.lookat[:] = view['lookat']

            renderer.update_scene(data, camera, scene_option)
            pixels = renderer.render()

            # Convert to PIL Image and add annotation
            img = Image.fromarray(pixels)
            filename = f"L1_tendon_{grasp['name']}_{view['name']}.png"
            img.save(filename)
            print(f"    Saved: {filename}")

    print("\nImage generation complete!")
    print("Images show:")
    print("- L1_face.stl revealing internal tendon structure")
    print("- Tendon routing through L1 segments")
    print("- Progressive grasping motion")
    print("\nOpen the PNG files to see the tendon structure in L1!")

if __name__ == '__main__':
    create_tendon_visibility_images()
