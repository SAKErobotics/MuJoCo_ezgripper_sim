#!/usr/bin/env python3
"""
Generate thin shell STL files for EZGripper collision geometries.
Outputs binary STL format directly for MuJoCo compatibility.
"""

import os
import struct

def write_binary_stl(filename, faces):
    """Write faces to binary STL file"""
    with open(filename, 'wb') as f:
        # 80-byte header
        header = b'Thin shell collision geometry for EZGripper' + b'\x00' * (80 - len(b'Thin shell collision geometry for EZGripper'))
        f.write(header)

        # 4-byte triangle count
        f.write(struct.pack('<I', len(faces)))

        # Write each triangle
        for face in faces:
            # Normal vector (compute from first 3 points)
            v1 = [face[1][i] - face[0][i] for i in range(3)]
            v2 = [face[2][i] - face[0][i] for i in range(3)]
            normal = [
                v1[1]*v2[2] - v1[2]*v2[1],
                v1[2]*v2[0] - v1[0]*v2[2],
                v1[0]*v2[1] - v1[1]*v2[0]
            ]
            # Normalize
            length = (normal[0]**2 + normal[1]**2 + normal[2]**2)**0.5
            if length > 0:
                normal = [n/length for n in normal]

            f.write(struct.pack('<fff', *normal))

            # Three vertices
            for vertex in face:
                f.write(struct.pack('<fff', *vertex))

            # Attribute byte count
            f.write(struct.pack('<H', 0))

def create_box_shell(size, thickness=0.001):
    """
    Create a thin shell box STL with only inside face having thickness.
    size: [x, y, z] dimensions
    thickness: shell thickness (1mm = 0.001)
    """
    import numpy as np

    # Create outer box vertices (full size)
    x, y, z = size
    outer = np.array([
        # Bottom face
        [-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],
        # Top face
        [-x, -y, z], [x, -y, z], [x, y, z], [-x, y, z]
    ])

    # Create inner box vertices (reduced by thickness on inside face)
    # Only the top face is reduced to create the inside collision surface
    inner = np.array([
        # Bottom face (unchanged - outside)
        [-x, -y, -z], [x, -y, -z], [x, y, -z], [-x, y, -z],
        # Top face (reduced inward by thickness)
        [-x+thickness, -y+thickness, z-thickness],
        [x-thickness, -y+thickness, z-thickness],
        [x-thickness, y-thickness, z-thickness],
        [-x+thickness, y-thickness, z-thickness]
    ])

    # Create faces for the shell (only the inner collision surface and connecting sides)
    faces = []

    # Inner top face (the collision surface we want) - this is the key collision surface
    faces.append([inner[4], inner[5], inner[6]])  # Triangle 1
    faces.append([inner[4], inner[6], inner[7]])  # Triangle 2

    # Side faces connecting inner and outer (thin walls)
    # Front face
    faces.append([inner[3], inner[7], outer[7]])
    faces.append([inner[3], outer[7], outer[3]])

    # Back face
    faces.append([inner[0], outer[0], outer[4]])
    faces.append([inner[0], outer[4], inner[4]])

    # Left face
    faces.append([inner[0], inner[4], outer[4]])
    faces.append([inner[0], outer[4], outer[0]])

    # Right face
    faces.append([inner[1], outer[1], outer[5]])
    faces.append([inner[1], outer[5], inner[5]])

    return faces

def main():
    """Generate shell STL files for all collision geometries"""

    # Create meshes directory if it doesn't exist
    os.makedirs('meshes', exist_ok=True)

    # Collision geometries from ezgripper.xml
    collision_geoms = {
        # Palm mechanical stops
        'palm_stop_f1_lower_shell.stl': [0.005, 0.015, 0.005],
        'palm_stop_f2_lower_shell.stl': [0.005, 0.015, 0.005],
        'palm_stop_f1_upper_shell.stl': [0.005, 0.015, 0.005],
        'palm_stop_f2_upper_shell.stl': [0.005, 0.015, 0.005],

        # Finger L1 mechanical stops
        'f1l1_stop_lower_shell.stl': [0.008, 0.020, 0.008],
        'f1l1_stop_upper_shell.stl': [0.008, 0.020, 0.008],
        'f2l1_stop_lower_shell.stl': [0.008, 0.020, 0.008],
        'f2l1_stop_upper_shell.stl': [0.008, 0.020, 0.008],

        # L1-L2 joint stops
        'f1l1_l2stop_zero_shell.stl': [0.008, 0.015, 0.008],
        'f1l1_l2stop_max_shell.stl': [0.008, 0.015, 0.008],
        'f2l1_l2stop_zero_shell.stl': [0.008, 0.015, 0.008],
        'f2l1_l2stop_max_shell.stl': [0.008, 0.015, 0.008],
    }

    print("Generating thin shell STL files in binary format...")

    for filename, size in collision_geoms.items():
        filepath = f'meshes/{filename}'
        faces = create_box_shell(size)
        write_binary_stl(filepath, faces)
        print(f"Saved: {filepath}")

    print("All shell STL files generated successfully!")

if __name__ == '__main__':
    main()
