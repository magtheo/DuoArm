import os 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
#from launch import LaunchException  # To handle exceptions


def generate_launch_description():

    # Define package_dir correctly
    package_dir = get_package_share_directory('arm')


    urdf_file_path = "src/arm/urdf/robot.urdf"
    
    # Read the urdf file
    with open(urdf_file_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_content }]
    )

    rviz_config_file = os.path.join(package_dir, 'rviz', 'my_robot_rviz_config.rviz')
    print("RVIZ CONFIG FILE:" + rviz_config_file)
    node_rviz = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
    )


    return LaunchDescription([
        node_robot_state_publisher, 
        node_rviz,
    ])
