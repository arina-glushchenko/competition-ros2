import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_directory = get_package_share_directory('robot_bringup')
    camera_directory = get_package_share_directory('autorace_camera')
    move_directory = get_package_share_directory('comp_nedorosl')

    camera_calibration = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(camera_directory, 'launch', 'extrinsic_camera_calibration.launch.py')
    ))

    pid_reg = Node(
    	package= 'comp_nedorosl',
    	executable= 'pid',
    	namespace='drive',
    	name= 'pid',
    	output='screen'
    )
    
    detect_line = Node(
    	package= 'comp_nedorosl',
    	executable= 'detect_line',
    	namespace='detect',
        name='line',
    	output='screen'
    )
    
    detector_sign = Node(
    	package= 'comp_nedorosl',
    	executable= 'sign_recognition',
    	namespace='detect',
        name='line',
    	output='screen',
        #parameters=[{'model_path':move_directory}]
        )
    
    return LaunchDescription([
        # launches
        camera_calibration,

        # my_nodes
        pid_reg,
        detect_line,
        detector_sign,

    ])
