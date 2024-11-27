import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node  # Correct import for Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Correct import
from launch_ros.substitutions import FindPackageShare  # To find the package path for usb_cam

def generate_launch_description():
    return LaunchDescription([
        # Declare necessary launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),

        # Launch the my_service_server node
        Node(
            package='my_service_server',  # Package name where your service server is defined
            executable='my_service_server',  # The node executable name
            name='service_server_node',  # Node name (optional)
            output='screen',  # Output to the console
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # Launch the cpp_node node
        Node(
            package='cpp_package',  # Package where your cpp_node is defined
            executable='cpp_node',  # The node executable name
            name='cpp_node',  # Node name (optional)
            output='screen',  # Output to the console
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),

        # Log a message about using simulation time
        LogInfo(
            condition=launch.conditions.LaunchConfigurationEquals('use_sim_time', 'true'),
            msg="Using simulation time"
        ),

        # Use IncludeLaunchDescription to launch the camera.launch.py file from the usb_cam package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [FindPackageShare('usb_cam'), '/launch/camera.launch.py']
            ),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
        ),
    ])

