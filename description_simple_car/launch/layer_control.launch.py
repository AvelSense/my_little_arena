from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

# ros2 topic pub /controller_caster/commands std_msgs/msg/Float64MultiArray "data:
# - 0.5
# "

# ros2 topic pub /controller_wheels/commands std_msgs/msg/Float64MultiArray "data:
# - 0.5
# - -0.5
# "

def generate_launch_description():
    ld = LaunchDescription()

    ##############################################################################################
    ####################################  Start Zenoh router  ####################################
    ##############################################################################################
    # zenoh_router = Node(
    #     package='rmw_zenoh_cpp',
    #     executable='rmw_zenohd',
    #     name='zenoh_router'
    # )
    # ld.add_action(zenoh_router)

    ##############################################################################################
    #######################          Default values for arguments          #######################
    ##############################################################################################
    package_path = FindPackageShare('description_simple_car')
    default_model_path = PathJoinSubstitution([package_path, 'urdf', 'simple_robot.urdf.xacro'])
    default_controllers_path = PathJoinSubstitution([package_path, 'config', 'controllers.yaml'])
    default_rviz_config_path = PathJoinSubstitution([package_path, 'rviz', 'urdf.rviz'])
    default_world_file = PathJoinSubstitution([package_path, 'worlds', 'base_world.sdf'])

    ##############################################################################################
    #######################              Launch arguments                  #######################
    ##############################################################################################
    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_path,
                                      description="Absolute path to robot urdf file")
    ld.add_action(model_arg)
    controllers_arg = DeclareLaunchArgument(name='controllers_arg', default_value=default_controllers_path,
                                      description="Absolute path to controllers YAML file")
    ld.add_action(controllers_arg)
    world_arg = DeclareLaunchArgument(name='world', default_value=default_world_file,
                                      description="Absolute path to world sdf file")
    ld.add_action(world_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    ##############################################################################################
    ######         Convert xacro to URDF, load URDF and launch robot publisher             #######
    ##############################################################################################
    robot_description_content = Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                                         LaunchConfiguration('model')])
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description,
            {'use_sim_time': True},   # use Gazebo clock, not wall clock
        ]
    )
    ld.add_action(robot_state_pub_node)

    ##############################################################################################
    ##################  Start Gazebo simulator and spawn robot inside world  #####################
    ##############################################################################################
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [
            TextSubstitution(text='-r '),  # literal prefix
            LaunchConfiguration('world'),        # resolved at launch time
        ]}.items(),
    )
    ld.add_action(gz_sim)

    # Clock bridge — must be up before anything uses sim time
    gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )
    ld.add_action(gazebo_bridge)

    # --- Spawn robot ---
    # When this exits, the GazeboSimROS2ControlPlugin has already started
    # its own controller_manager inside the Gazebo process.
    spawner_robot_sim = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'my_robot',
            '-topic', '/robot_description',  # reads URDF from this topic
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.6',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )
    ld.add_action(spawner_robot_sim)

    ##############################################################################################
    ######             Spawn controllers, including joint state broadcaster                #######
    ##############################################################################################
    # Only on real hardware, not in simulation, as the GazeboSimROS2ControlPlugin already starts a controller_manager with the specified controllers.
    control_node= Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, LaunchConfiguration('controllers_arg')],
        output="both",
    )
    ld.add_action(control_node)
    
    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Wait for robot to be spawned, then start controller_manager and joint_state_broadcaster
    controls_wait_for_robot = RegisterEventHandler(
        OnProcessExit(target_action=spawner_robot_sim, on_exit=[spawner_jsb]))
    ld.add_action(controls_wait_for_robot)

    spawner_controller_wheels = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["controller_wheels", "--controller-manager", "/controller_manager"],
    )

    spawner_controller_caster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["controller_caster", "--controller-manager", "/controller_manager"],
    )
    # Wait for joint_state_broadcaster to be spawned, then start controllers
    controllers_wait_for_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawner_jsb,
                      on_exit=[spawner_controller_wheels, spawner_controller_caster]))
    ld.add_action(controllers_wait_for_jsb)

    ##############################################################################################
    ##################                     Visualisation                     #####################
    ##############################################################################################
    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration('rvizconfig')],
        parameters=[{'use_sim_time': True}],
    )
    # Wait for joint_state_broadcaster to be spawned, then start RVIZ
    rviz_wait_for_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawner_jsb, on_exit=[node_rviz]))
    ld.add_action(rviz_wait_for_jsb)


    ##############################################################################################
    ##############################################################################################
    ##############################################################################################

    return ld