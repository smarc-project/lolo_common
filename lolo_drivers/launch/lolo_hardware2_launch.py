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
        name='ctd_parser',
        parameters=[{
                "input_topic": Topics.CTD_RAW_TOPIC,
                "output_topic": Topics.CTD_TOPIC,
            }]
    )

    turbidity_parser_node = Node(
        package='turbidity_driver',
        namespace=robot_ns,
        executable='turbidity_driver',
        name='turbidity_parser',
        parameters=[{
            "input_topic": Topics.TURB_RAW_TOPIC,
            "output_topic": Topics.TURB_TOPIC,
        }]
    )

    do_parser_node = Node(
        package='do_driver',
        namespace=robot_ns,
        executable='do_driver',
        name='do_parser',
        parameters=[{
            "input_topic": Topics.DO_RAW_TOPIC,
            "output_topic": Topics.DO_TOPIC,
            "request_topic" : Topics.DO_RAW_REQUEST_TOPIC,
        }]
    )
    
    svs_parser_node = Node(
        package='svs_driver',
        namespace=robot_ns,
        executable='svs_driver',
        name='svs_parser',
        parameters=[{
            "input_topic": Topics.SVS_RAW_TOPIC,
            "output_topic": Topics.SVS_TOPIC,
        }]
    )

    chlorophyll_parser_node = Node(
        package='hyperion_chlorophyll_a_driver',
        namespace=robot_ns,
        executable='chlorophyll_decoder',
        name='chlorophyll_parser',
        parameters=[{
            "input_topic": Topics.FLUORESCENCE_RAW_TOPIC,
            "output_topic": Topics.FLUORESCENCE_TOPIC,
        }]


    svs_to_ins_node = Node(
        package='lolo_svs_to_ins',
        namespace=robot_ns,
        executable='lolo_svs_to_ins_node',
        name='lolo_svs_to_ins',
        parameters=[{
            "svs_topic": Topics.SVS_TOPIC,
            "endpoint_ip" : "192.168.1.95"
            "endpoint_port" : 9008
        }]
    )


    return LaunchDescription([
        robot_ns_launch_arg,
        extended_translator_node,
        ctd_parser_node,
        turbidity_parser_node,
        do_parser_node,
        svs_parser_node,
        chlorophyll_parser_node
    ])