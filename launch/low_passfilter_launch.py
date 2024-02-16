#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import DeclareLaunchArgument

from launch.substitutions import LaunchConfiguration, PythonExpression


lifecycle_nodes = ['lowpass_filter']
autostart = LaunchConfiguration('autostart')

declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

def generate_launch_description():

	return LaunchDescription([
		Node(
			package='low_passfilter',
			executable='lowpass_filter',
			name='lowpass_filter',
			output='screen',
		),

		Node(
			package='nav2_lifecycle_manager',
			executable='lifecycle_manager',
			name='lifecycle_manager_navigation',
			output='screen',
			parameters=[{'node_names': lifecycle_nodes},
			   {'autostart':True}])


	])
