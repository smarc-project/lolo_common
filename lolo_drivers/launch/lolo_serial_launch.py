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

    ctd_reader_node = Node(
        package='serial_parser',
        namespace=robot_ns,
        executable='serial_parser',
        name='ctd_reader',
        parameters=[{
                "port" : "/dev/ttyUSB0",
                "baudrate" : 115200,
                "listen_to_topic" : "unused",
                "publish_to_topic" : Topics.CTD_RAW_TOPIC,
                "poll_rate" : 10,
            }]
    )

    turb_reader_node = Node(
        package='serial_parser',
        namespace=robot_ns,
        executable='serial_parser',
        name='turb_reader',
        parameters=[{
                "port" : "/dev/ttyUSB1",
                "baudrate" : 115200,
                "listen_to_topic" : "unused",
                "publish_to_topic" : Topics.TURB_RAW_TOPIC,
                "poll_rate" : 10,
            }]
    )

    fluo_reader_node = Node(
        package='serial_parser',
        namespace=robot_ns,
        executable='serial_parser',
        name='fluo_reader',
        parameters=[{
                "port" : "/dev/ttyUSB2",
                "baudrate" : 115200,
                "listen_to_topic" : "unused",
                "publish_to_topic" : Topics.FLUORESCENCE_RAW_TOPIC,
                "poll_rate" : 10,
            }]
    )

    svs_reader_node = Node(
        package='serial_parser',
        namespace=robot_ns,
        executable='serial_parser',
        name='svs_reader',
        parameters=[{
                "port" : "/dev/ttyUSB3",
                "baudrate" : 9600,
                "listen_to_topic" : "unused",
                "publish_to_topic" : Topics.SVS_RAW_TOPIC,
                "poll_rate" : 10,
            }]
    )

    do_reader_node = Node(
        package='serial_parser',
        namespace=robot_ns,
        executable='serial_parser',
        name='do_reader',
        parameters=[{
                "port" : "/dev/ttyUSB5",
                "baudrate" : 38400,
                "listen_to_topic" : Topics.DO_RAW_REQUEST_TOPIC,
                "publish_to_topic" : Topics.DO_RAW_TOPIC,
                "poll_rate" : 10,
            }]
    )

    return LaunchDescription([
        robot_ns_launch_arg,
        ctd_reader_node,
        turb_reader_node,
        fluo_reader_node,
        svs_reader_node,
        do_reader_node
    ])