from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from lolo_msgs.msg import Topics


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='lolo'
    )

    extended_translator_node = Node(
        package='lolo_extended_interface_translator',
        namespace=robot_ns,
        executable='lolo_extended_translator_node',
        name='extended_translator'
    )

    ctd_parser_node = Node(
        package='rbr_ctd_driver',
        namespace=robot_ns,
        executable='rbr_ctd_driver',
        name='extended_translator',
        parameters=[{
                "input_topic": Topics.CTD_RAW_TOPIC,
                "output_topic": Topics.CTD_TOPIC,
            }]
    )
    

    return LaunchDescription([
        robot_ns_launch_arg,
        extended_translator_node,
        ctd_parser_node
    ])