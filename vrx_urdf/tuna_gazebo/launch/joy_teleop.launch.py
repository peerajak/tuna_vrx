from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'linear_axis',
            default_value='1',
            description='Joystick axis for linear control'
        ),
        DeclareLaunchArgument(
            'angular_axis',
            default_value='0',
            description='Joystick axis for angular control'
        ),
        DeclareLaunchArgument(
            'scale_linear',
            default_value='1.0',
            description='Maximum linear velocity (m/s)'
        ),
        DeclareLaunchArgument(
            'scale_angular',
            default_value='0.5',
            description='Maximum angular velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'deadzone',
            default_value='0.1',
            description='Joystick deadzone threshold'
        ),
        
        # Joy node (reads actual joystick input)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': '/dev/input/js0',  # Adjust if needed
                'deadzone': 0.1,
                'autorepeat_rate': 20.0,
            }]
        ),
        
        # Joy teleop node
        Node(
            package='tuna_gazebo',
            executable='joy_teleop.py',
            name='joy_teleop',
            parameters=[{
                'linear_axis': LaunchConfiguration('linear_axis'),
                'angular_axis': LaunchConfiguration('angular_axis'),
                'scale_linear': LaunchConfiguration('scale_linear'),
                'scale_angular': LaunchConfiguration('scale_angular'),
                'deadzone': LaunchConfiguration('deadzone'),
                'enable_button': 0,  # Typically A button on XBox controller
                'turbo_button': 4,   # Typically LB button on XBox controller
                'turbo_multiplier': 2.0,
            }],            
            remappings=[
                ('/cmd_vel', '/tuna/diff_drive_controller/cmd_vel')
            ],
            output='screen'
        )
    ])