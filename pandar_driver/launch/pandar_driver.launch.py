import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='pandar_driver',
            executable='pandar_driver_node',
            output='screen',
            parameters=[{'pcap':''},{'device_ip':'192.168.1.200'},{'lidar_port':'2368'},{'gps_port':'10110'},{'scan_phase':'0'},{'model':'Pandar40P'},{'frame_id':'pandar'}]
        )

    ])
