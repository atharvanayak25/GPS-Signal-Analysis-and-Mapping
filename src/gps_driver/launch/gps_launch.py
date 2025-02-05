from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gps_driver',  # Your package name
            executable='talker',   # The executable defined in setup.py
            name='gps_driver_node',
            parameters=[{
                'port': LaunchConfiguration('port', default='/dev/ttyUSB0'),
                'baudrate': LaunchConfiguration('baudrate', default='4800'),
                'sampling_rate': LaunchConfiguration('sampling_rate', default='10')
            }],
            output='screen'
        )
    ])