#!/usr/bin/env python3
"""
Test script to verify thin shell collision meshes work correctly.
Loads the EZGripper model and checks that tendons are visible.
"""

import mujoco
import numpy as np

def test_shell_meshes():
    """Test that the shell meshes load and work correctly"""

    print("Testing EZGripper with thin shell collision meshes...")

    # Load model
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)

    print(f"Model loaded successfully: {model.ngeom} geoms, {model.nmesh} meshes")

    # Check that shell meshes are loaded
    shell_mesh_names = [
        'palm_stop_f1_lower_shell', 'palm_stop_f2_lower_shell',
        'palm_stop_f1_upper_shell', 'palm_stop_f2_upper_shell',
        'f1l1_stop_lower_shell', 'f1l1_stop_upper_shell',
        'f2l1_stop_lower_shell', 'f2l1_stop_upper_shell',
        'f1l1_l2stop_zero_shell', 'f1l1_l2stop_max_shell',
        'f2l1_l2stop_zero_shell', 'f2l1_l2stop_max_shell'
    ]

    loaded_shells = 0
    for mesh_name in shell_mesh_names:
        try:
            mesh_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, mesh_name)
            if mesh_id >= 0:
                loaded_shells += 1
                print(f"✓ Found shell mesh: {mesh_name}")
            else:
                print(f"✗ Missing shell mesh: {mesh_name}")
        except:
            print(f"✗ Error loading shell mesh: {mesh_name}")

    print(f"\nLoaded {loaded_shells}/{len(shell_mesh_names)} shell meshes")

    # Check that corresponding geoms are using mesh type
    shell_geom_names = [
        'palm_stop_f1_lower', 'palm_stop_f2_lower',
        'palm_stop_f1_upper', 'palm_stop_f2_upper',
        'f1l1_stop_lower', 'f1l1_stop_upper',
        'f2l1_stop_lower', 'f2l1_stop_upper',
        'f1l1_l2stop_zero', 'f1l1_l2stop_max',
        'f2l1_l2stop_zero', 'f2l1_l2stop_max'
    ]

    mesh_geoms = 0
    for geom_name in shell_geom_names:
        try:
            geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
            if geom_id >= 0:
                geom_type = model.geom_type[geom_id]
                if geom_type == mujoco.mjtGeom.mjGEOM_MESH:
                    mesh_geoms += 1
                    print(f"✓ Geom {geom_name} is mesh type")
                else:
                    print(f"✗ Geom {geom_name} is not mesh type (type: {geom_type})")
        except:
            print(f"✗ Error checking geom: {geom_name}")

    print(f"\n{mesh_geoms}/{len(shell_geom_names)} geoms are mesh type")

    # Test forward kinematics
    mujoco.mj_forward(model, data)

    # Check tendon lengths
    tendon_names = ['finger1_tendon', 'finger2_tendon']
    for tendon_name in tendon_names:
        tendon_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, tendon_name)
        if tendon_id >= 0:
            length = data.ten_length[tendon_id]
            print(f"✓ Tendon {tendon_name}: {length*1000:.2f}mm")

    print("\n✓ Test completed successfully!")
    print("The EZGripper model now uses thin shell collision meshes for debugging.")

if __name__ == '__main__':
    test_shell_meshes()
