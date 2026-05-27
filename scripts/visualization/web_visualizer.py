#!/usr/bin/env python3
"""
Simple web-based visualizer for MuJoCo model positions.
"""

import mujoco
import numpy as np
import os
import json
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import webbrowser
import time

# Load model
viz_path = os.path.join(os.path.dirname(__file__), 'ezgripper_visualization.xml')
model = mujoco.MjModel.from_xml_path(viz_path)
data = mujoco.MjData(model)

print("="*60)
print("WEB-BASED MUJOCO VISUALIZER")
print("="*60)

# Get marker positions at different angles
angles = [-90, -45, 0, 25]
positions = {}

for angle in angles:
    data.qpos[model.joint('F1_palm_knuckle').id] = np.radians(angle)
    mujoco.mj_forward(model, data)
    
    # Get positions of key elements
    positions[angle] = {
        'palm_lower': data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_lower")].tolist(),
        'palm_upper': data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "palm_stop_f1_upper")].tolist(),
        'current_lower': data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_lower")].tolist(),
        'current_upper': data.geom_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "f1l1_stop_upper")].tolist(),
    }

# Create HTML content
html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <title>EZGripper Stop Position Visualizer</title>
    <style>
        body {{ font-family: Arial, sans-serif; margin: 20px; }}
        .container {{ max-width: 1200px; margin: 0 auto; }}
        .angle-section {{ margin: 30px 0; padding: 20px; border: 1px solid #ccc; border-radius: 8px; }}
        .position {{ margin: 10px 0; padding: 10px; border-radius: 4px; }}
        .palm {{ background-color: #e3f2fd; }}
        .current {{ background-color: #ffebee; }}
        .distance {{ background-color: #fff3e0; }}
        h1 {{ color: #333; }}
        h2 {{ color: #666; }}
        .coords {{ font-family: monospace; color: #666; }}
        .good {{ color: green; font-weight: bold; }}
        .bad {{ color: red; font-weight: bold; }}
    </style>
</head>
<body>
    <div class="container">
        <h1>EZGripper Mechanical Stop Position Visualizer</h1>
        <p><strong>Marker colors:</strong></p>
        <ul>
            <li><span style="color: blue;">●</span> Palm stops (reference positions)</li>
            <li><span style="color: red;">●</span> Current finger stops (wrong positions)</li>
        </ul>
        
        <h2>Position Analysis</h2>
"""

for angle in angles:
    pos = positions[angle]
    
    # Calculate distances
    palm_lower = np.array(pos['palm_lower'])
    current_lower = np.array(pos['current_lower'])
    dist_current = np.linalg.norm(current_lower - palm_lower)
    
    html_content += f"""
        <div class="angle-section">
            <h2>Angle: {angle}°</h2>
            
            <div class="position palm">
                <strong>Palm Lower Stop:</strong>
                <span class="coords">[{pos['palm_lower'][0]:.3f}, {pos['palm_lower'][1]:.3f}, {pos['palm_lower'][2]:.3f}]</span>
            </div>
            
            <div class="position current">
                <strong>Current Finger Stop:</strong>
                <span class="coords">[{pos['current_lower'][0]:.3f}, {pos['current_lower'][1]:.3f}, {pos['current_lower'][2]:.3f}]</span>
            </div>
            
            <div class="position distance">
                <strong>Distance:</strong> {dist_current:.4f} 
                <span class="{'good' if dist_current < 0.01 else 'bad'}">
                    ({'COLLIDES' if dist_current < 0.01 else 'NO COLLISION'})
                </span>
            </div>
        </div>
    """

html_content += """
    </div>
</body>
</html>
"""

# Write HTML file
html_file = os.path.join(os.path.dirname(__file__), 'visualizer.html')
with open(html_file, 'w') as f:
    f.write(html_content)

print(f"✓ HTML visualizer created: {html_file}")
print(f"✓ Open this file in a web browser to see the visualization")
print(f"\nTo view:")
print(f"  firefox {html_file}")
print(f"  or google-chrome {html_file}")
print(f"  or double-click the file")

# Try to open in browser automatically
try:
    webbrowser.open(f'file://{html_file}')
    print(f"✓ Opened in default browser")
except:
    print(f"✓ Please open the file manually in your browser")

print("\n" + "="*60)
print("KEY FINDINGS FROM VISUALIZATION:")
print("="*60)
for angle in angles:
    pos = positions[angle]
    palm_lower = np.array(pos['palm_lower'])
    current_lower = np.array(pos['current_lower'])
    dist_current = np.linalg.norm(current_lower - palm_lower)
    
    status = "✓ COLLIDES" if dist_current < 0.01 else "✗ NO COLLISION"
    print(f"Angle {angle:3d}°: Distance {dist_current:.4f} {status}")
