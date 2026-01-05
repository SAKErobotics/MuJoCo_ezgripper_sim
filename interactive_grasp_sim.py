#!/usr/bin/env python3
"""
Interactive MuJoCo simulation showing EZGripper grasping with L1 tendon structure.
Provides real-time visualization of tendon operation during grasping.
"""

import mujoco
import mujoco.viewer
import numpy as np
import time

def run_grasping_simulation():
    """Run interactive simulation of EZGripper grasping a cylinder"""

    # Load the model with L1_face.stl
    model = mujoco.MjModel.from_xml_path('ezgripper.xml')
    data = mujoco.MjData(model)

    print("EZGripper L1 Tendon Structure Grasping Simulation")
    print("================================================")
    print("Loading EZGripper with L1_face.stl showing tendon structure")
    print("Press 'Ctrl+C' or close window to exit the simulation")
    print("")

    # Simulation parameters
    grasping = False
    grasp_start_time = 0
    grasp_duration = 2.0  # seconds to complete grasp

    def controller(model, data):
        """Control function for grasping motion"""
        nonlocal grasping, grasp_start_time

        current_time = data.time

        # Start grasping after 1 second
        if current_time > 1.0 and not grasping:
            grasping = True
            grasp_start_time = current_time
            print(f"Starting grasp at t={current_time:.2f}s")

        if grasping:
            # Gradually close the gripper tendons over grasp_duration seconds
            grasp_progress = min((current_time - grasp_start_time) / grasp_duration, 1.0)

            # Apply tendon force to close gripper
            # Positive control pulls tendon (closes gripper)
            tendon_force = 50.0 * grasp_progress  # Gradually increasing force

            # Apply to the finger1 tendon actuator
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'gripper_actuator')
            data.ctrl[actuator_id] = tendon_force

            if grasp_progress >= 1.0 and current_time - grasp_start_time > grasp_duration + 0.5:
                print(f"Grasp completed at t={current_time:.2f}s")
                grasping = False

    # Launch interactive viewer with controller
    with mujoco.viewer.launch_passive(model, data, key_callback=None) as viewer:
        # Set up camera for good view of grasping
        viewer.cam.distance = 0.25
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -10
        viewer.cam.lookat[:] = [0.08, 0, 0.02]

        print("Interactive viewer launched!")
        print("Camera controls: drag to rotate, scroll to zoom")
        print("Watch the tendons move as the gripper closes around the grasping point")
        print("L1_face.stl reveals the internal tendon structure")
        print("")

        # Run simulation loop
        while viewer.is_running():
            step_start = time.time()

            # Call controller
            controller(model, data)

            # Step physics
            mujoco.mj_step(model, data)

            # Update viewer
            viewer.sync()

            # Control frame rate
            time_until_next = model.opt.timestep - (time.time() - step_start)
            if time_until_next > 0:
                time.sleep(time_until_next)

if __name__ == '__main__':
    try:
        run_grasping_simulation()
    except KeyboardInterrupt:
        print("\nSimulation stopped by user")
    except Exception as e:
        print(f"Error running simulation: {e}")
        print("Make sure MuJoCo and the model files are properly configured")
