from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from lolo_msgs.msg import Topics


def generate_launch_description():
    namespace = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='lolo'
    )

    mbes_ip = "192.168.1.53"
    fls_ip = "192.168.1.89"

    mbes_settings_node = Node(
        package="norbit_wbms_driver",
        executable="wbms_driver",
        name="settings_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "sonar_ip": mbes_ip,
                "sonar_port": 2209,
            }
        ],
    )

    mbes_bathy_node = Node(
        package="norbit_wbms_driver",
        executable="bathymetry_parser",
        name="bathymetry_parser",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "sonar_ip": mbes_ip,
                "bathy_port": 2210,
                "output_topic": Topics.MBES_BATHY_TOPIC,
            }
        ],
    )

    mbes_bathy_translator_node = Node(
        package="norbit_wbms_driver",
        executable="bathymetry_to_pointcloud",
        name="bathymetry_to_pointcloud",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "input_topic": Topics.MBES_BATHY_TOPIC,
                "output_topic": Topics.MBES_BATHY_POINTCLOUD_TOPIC,
                "frame_id" : "lolo/mbes_link",
            }
        ],
    )

    mbes_watercolumn_node = Node(
        package="norbit_wbms_driver",
        executable="watercolumn_parser",
        name="watercolumn_parser",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "sonar_ip": mbes_ip,
                "watercolumn_port": 2211,
                "output_topic": Topics.MBES_WC_TOPIC,
                "output_image_topic": Topics.MBES_IMAGE_TOPIC,
            }
        ],
    )


    #FLS
    fls_settings_node = Node(
        package="norbit_wbms_driver",
        executable="wbms_driver",
        name="settings_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "sonar_ip": fls_ip,
                "sonar_port": 2209,
            }
        ],
    )

    fls_watercolumn_node = Node(
        package="norbit_wbms_driver",
        executable="watercolumn_parser",
        name="watercolumn_parser",
        output="screen",
        namespace=namespace,
        parameters=[
            {
                "sonar_ip": fls_ip,
                "watercolumn_port": 2211,
                "output_topic": Topics.FLS_WC_TOPIC,
                "output_image_topic": Topics.FLS_IMAGE_TOPIC,
            }
        ],
    )




    return LaunchDescription([
        robot_ns_launch_arg,  
        mbes_settings_node,
        mbes_bathy_node,
        mbes_bathy_translator_node,
        mbes_watercolumn_node,
        fls_settings_node,
        fls_watercolumn_node
    ])