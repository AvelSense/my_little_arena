from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    ##############################################################################################
    #######################          Default values for arguments          #######################
    ##############################################################################################
    package_path = FindPackageShare('description_simple_car')
    default_model_path = PathJoinSubstitution([package_path, 'urdf', 'simple_robot.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([package_path, 'rviz', 'urdf.rviz'])

    ##############################################################################################
    #######################              Launch arguments                  #######################
    ##############################################################################################
    model_arg = DeclareLaunchArgument(name='model', default_value=default_model_path,
                                      description="Absolute path to robot urdf file")
    ld.add_action(model_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    ##############################################################################################
    ######         Convert xacro to URDF, load URDF and launch robot publisher             #######
    ##############################################################################################
    robot_description_content = Command([PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
                                         LaunchConfiguration('model')])
    robot_description = {"robot_description": robot_description_content}

    joint_state_pub_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="both",

    )
    ld.add_action(joint_state_pub_node)

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )
    ld.add_action(robot_state_pub_node)


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
    ld.add_action(node_rviz)


    ##############################################################################################
    ##############################################################################################
    ##############################################################################################

    return ld