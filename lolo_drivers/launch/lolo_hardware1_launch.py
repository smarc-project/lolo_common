from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from lolo_msgs.msg import Topics


def generate_launch_description():
    robot_ns = LaunchConfiguration('robot_name')

    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='lolo'
    )

    lolo_hardware_interface_node = Node(
        package='lolo_interface',
        respawn=True,
        namespace=robot_ns,
        executable='lolo_interface',
        name='lolo_hardwater_interface_node'
    )

    nmea_navsat_driver_node = Node(
        package='nmea_navsat_driver',
        namespace=robot_ns,
        executable='nmea_socket_driver',
        name='lolo_nmea_socket_driver_node',
        parameters=[{
                "port": 9009,
            }],
        remappings=[
            ('fix', Topics.GPS_TOPIC),
            ('vel', Topics.GPS_TOPIC + "/vel"),
            ('heading', Topics.GPS_TOPIC + "/heading"),
        ]
    )

    ixblue_ins_driver_node = Node(
        package='ixblue_ins_driver',
        namespace=robot_ns,
        executable='output_node',
        name="ixblue_ins_driver",
        parameters=[{
                "frame_id": "imu_link_ned",
                "udp_port": 8200,
                "ip": "192.168.1.100",
                "time_source": "ins",
                "time_origin": "unix",
                "expected_frequency": 10.0,
                "max_latency": 1.0,
                "conection_lost_timeout": 10.0,
                "use_compensated_acceleration": False,
            }],
        remappings=[
            ('ix/ins', Topics.INS_RAW_TOPIC),
            ('standard/imu', Topics.INS_IMU_TOPIC),
        ]
    )

    nortek_dvl_driver_node = Node(
        package='nortek_dvl333_driver',
        namespace=robot_ns,
        executable='dvl_node',
        name="nortek_dvl_driver_node",
        parameters=[{
                "tcp_mode": False,
                "sonar_ip": "192.168.1.95",
                "buffer_size_bytes": 512000,
            }]
    )


    return LaunchDescription([
        robot_ns_launch_arg,
        lolo_hardware_interface_node,
        nmea_navsat_driver_node,
        ixblue_ins_driver_node,
        nortek_dvl_driver_node,
    ])
