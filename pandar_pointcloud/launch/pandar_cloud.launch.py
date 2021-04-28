import launch
import launch_ros.actions

my_calibraion_file = '/home/autoware/temp/hesai_pandar_ros2/src/pandar_pointcloud/config/40p.csv'

def generate_launch_description():

    pandar_pcl = launch_ros.actions.Node(
            package='pandar_pointcloud',
            executable='pandar_cloud_node',
            output='screen',
            remappings=[('pandar_points','pointcloud_raw'),('pandar_pointx_ex','pointcloud_raw_ex')],
            parameters=[{'pcap':''},{'device_ip':'192.168.1.201'},{'model':'Pandar40P'},{'scan_phase':0.0},{'calibration':my_calibraion_file}]
    )

    return launch.LaunchDescription([
        pandar_pcl,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=pandar_pcl,
                on_exit=[launch.actions.EmitEvent(
                    event=launch.events.Shutdown())],
            )),
    ])