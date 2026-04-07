import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # 1. Define Paths
    
    description_pkg_share = get_package_share_directory('model_description')
    bringup_pkg_share = get_package_share_directory('smorphi_bringup')
    #smorphi_node_pkg_share = get_package_share_directory('smorphi_node')

    #RPLIDAR 
    ydlidar_pkg_share = get_package_share_directory('ydlidar_ros2_driver')

    
    urdf_path = os.path.join(description_pkg_share, 'urdf', 'smorphi.urdf.xacro')
    #rviz_config_path = os.path.join(bringup_pkg_share, 'config', 'rviz2.rviz')

    # 2. Robot State Publisher Node
    # This uses 'xacro' command to process the file into raw URDF string
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            #'robot_description': Command(['xacro ', urdf_path])
           'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]), value_type=str)
        }]
    )

    # 3. Joint State Publisher Node
    # If you want the GUI version, use 'joint_state_publisher_gui'
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    
    # 4. RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        #arguments=['-d', rviz_config_path],
    )

    # 5. 
    #smorphi_node = Node(
     #   package='smorphi_node',
      #  executable='smorphi_node.py',
       # output='screen'
   # )
    ydlidar_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(
             os.path.join(ydlidar_pkg_share, 'launch', 'ydlidar_launch.py')
         )
    )
    # Create Launch Description and add actions
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        #smorphi_node,
        ydlidar_launch
    ])
