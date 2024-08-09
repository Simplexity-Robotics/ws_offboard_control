import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch the camera node
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.AnyLaunchDescriptionSource('/root/ws_offboard_control/src/rb5_ros2/rb5_ros2_vision/launch/rb_camera_main_ocv.launch.xml')
        ),
        # Run the underwater rectangle detector node
        launch_ros.actions.Node(
            package='buoy_detector',
            executable='buoy_detector',
            name='buoy_detector',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
