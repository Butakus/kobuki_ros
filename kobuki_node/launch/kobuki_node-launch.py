import os

import ament_index_python.packages
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import yaml


def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('kobuki_node')
    # There are two different ways to pass parameters to a non-composed node;
    # either by specifying the path to the file containing the parameters, or by
    # passing a dictionary containing the key -> value pairs of the parameters.
    # When starting a *composed* node on the other hand, only the dictionary
    # style is supported.  To keep the code between the non-composed and
    # composed launch file similar, we use that style here as well.
    params_file = os.path.join(share_dir, 'config', 'kobuki_node_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['kobuki_ros_node']['ros__parameters']

    ld = LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='', description='Namespace for the robot'),
    ])

    namespace = LaunchConfiguration('namespace')

    kobuki_ros_node = launch_ros.actions.Node(package='kobuki_node',
                                              executable='kobuki_ros_node',
                                              output='both',
                                              parameters=[params],
                                              namespace=namespace)

    return launch.LaunchDescription([kobuki_ros_node])
