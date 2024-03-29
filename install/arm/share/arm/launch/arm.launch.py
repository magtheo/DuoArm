import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
#from launch import LaunchException  # To handle exceptions


def generate_launch_description():

    # Define package_dir correctly
    package_dir = get_package_share_directory('arm')

    urdf_file_path = os.path.join(package_dir, 'urdf', 'robot.urdf')
    #urdf_file_path = "src/arm/urdf/robot.urdf"
    
    # Read the urdf file
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    # ROBOT STATE PUBLISHER
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_content }]
    )


    # RVIZ
    rviz_config_file = os.path.join(package_dir, 'rviz', 'rviz_basic_settings.rviz')
    print("RVIZ CONFIG FILE:" + rviz_config_file)
    node_rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen',
    additional_env={'LD_LIBRARY_PATH': os.environ['LD_LIBRARY_PATH']}
    )


    return LaunchDescription([
        node_robot_state_publisher, 
        TimerAction(
            period=3.0,  # Adjust the delay as necessary
            actions=[node_rviz],
        ),
    ])
