from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    dt = 0.1

    mass = 20
    inertia = 20

    ##############################################################################################
    #############################           Start 2D viewer          #############################
    ##############################################################################################

    node_viewer_2d = Node(
        package="arena_2d",
        executable="viewer",
        output="both",
    )
    ld.add_action(node_viewer_2d)


    ##############################################################################################
    #############################    Start simple tank simulation    #############################
    ##############################################################################################

    node_simulator_euler = Node(
        name="simulator_euler",
        package="arena_2d",
        executable="simulator_euler",
        output="both",
        parameters=[
            {
                "mass": mass,
                "inertia": inertia,
                "water_flow": [0.0, 0.0],
                "drag_coefficients": [0.0, 0.0, 0.0],
                "start_pose": [0.0, 0.0, 0.0],
                "dt": dt,
            }
        ]
    )
    ld.add_action(node_simulator_euler)

    ##############################################################################################
    ####################    Start ground truth (also sends commands to sim)    ###################
    ##############################################################################################

    node_ground_truth = Node(
        name="ground_truth",
        package="arena_2d",
        executable="ground_truth",
        output="both",
        parameters=[
            {
                "mass": mass,
                "inertia": inertia,
                "start_pose": [0.0, 0.0, 0.0],
                "Fx": 20,
                "Mz": 10,
                "launch_duration": 10,
                "dt": dt,
            }
        ]
    )

    # Wait for robot to be spawned, then start controller_manager and joint_state_broadcaster
    # wait_for_simulator = RegisterEventHandler(
    #     OnProcessExit(target_action=spawner_robot_sim, on_exit=[spawner_jsb]))
    # ld.add_action(controls_wait_for_robot)

    ld.add_action(node_ground_truth)

    ##############################################################################################
    ##############################################################################################
    ##############################################################################################

    return ld