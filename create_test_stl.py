#!/usr/bin/env python3
"""
Create a simple test binary STL file to verify MuJoCo compatibility
"""

import struct

def create_simple_binary_stl(filename):
    """Create a simple cube STL in binary format"""
    # Define a simple cube with 12 triangles (2 per face)
    # Cube from -0.005 to 0.005 (10mm cube)

    # Vertices of a cube
    vertices = [
        [-0.005, -0.005, -0.005],  # 0
        [0.005, -0.005, -0.005],   # 1
        [0.005, 0.005, -0.005],    # 2
        [-0.005, 0.005, -0.005],   # 3
        [-0.005, -0.005, 0.005],   # 4
        [0.005, -0.005, 0.005],    # 5
        [0.005, 0.005, 0.005],     # 6
        [-0.005, 0.005, 0.005],    # 7
    ]

    # Define triangles (12 faces, 2 per cube face)
    faces = [
        # Bottom face
        [vertices[0], vertices[1], vertices[2]],
        [vertices[0], vertices[2], vertices[3]],

        # Top face
        [vertices[4], vertices[6], vertices[5]],
        [vertices[4], vertices[7], vertices[6]],

        # Front face
        [vertices[0], vertices[3], vertices[7]],
        [vertices[0], vertices[7], vertices[4]],

        # Back face
        [vertices[1], vertices[5], vertices[6]],
        [vertices[1], vertices[6], vertices[2]],

        # Left face
        [vertices[0], vertices[4], vertices[5]],
        [vertices[0], vertices[5], vertices[1]],

        # Right face
        [vertices[3], vertices[2], vertices[6]],
        [vertices[3], vertices[6], vertices[7]],
    ]

    with open(filename, 'wb') as f:
        # 80-byte header
        header = b'Simple test cube for MuJoCo compatibility' + b'\x00' * (80 - len(b'Simple test cube for MuJoCo compatibility'))
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

    print(f"Created binary STL: {filename}")

def main():
    """Create a test STL file"""
    create_simple_binary_stl('meshes/test_cube.stl')
    print("Test STL file created for MuJoCo compatibility check")

if __name__ == '__main__':
    main()
