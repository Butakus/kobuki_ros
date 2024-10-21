# Copyright 2022 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros.descriptions
from launch.substitutions import Command
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


import os
import yaml
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription

import os
import yaml
import tempfile
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def modify_yaml_with_prefix(original_yaml_path, prefix):
    """
    Modifica el archivo .yaml para añadir el prefijo a los nombres de los tópicos.
    """
    with open(original_yaml_path, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)

    # Verificar si yaml_data es una lista (esperado)
    if not isinstance(yaml_data, list):
        raise ValueError("El archivo YAML no tiene el formato correcto. Se esperaba una lista de diccionarios.")

    # Iterar sobre cada remapeo de tópico y añadir el prefijo
    for remap in yaml_data:
        if isinstance(remap, dict):  # Asegurarse de que sea un diccionario
            ros_topic_name = remap.get('ros_topic_name', '')
            gz_topic_name = remap.get('gz_topic_name', '')

            # Añadir el prefijo a los nombres de los tópicos
            remap['ros_topic_name'] = f"{prefix}/{ros_topic_name}"
            remap['gz_topic_name'] = f"/{prefix}{gz_topic_name}"
        else:
            print(f"Se omite un elemento no válido en el YAML: {remap}")

    # Crear un archivo temporal para almacenar el nuevo .yaml
    temp_yaml = tempfile.NamedTemporaryFile(delete=False, mode='w', encoding='utf-8')
    temp_yaml_path = temp_yaml.name
    yaml.dump(yaml_data, temp_yaml, default_flow_style=False)
    temp_yaml.close()

    return temp_yaml_path

def start_bridge(context):
    # Condición para ejecutar el bridge solo si 'gazebo' es 'true'
    if LaunchConfiguration('gazebo').perform(context) == 'true':
        kobuki_pkg = get_package_share_directory('kobuki_description')

        # Ruta original del archivo .yaml
        original_yaml_path = os.path.join(
            kobuki_pkg, 'config/bridge', 'kobuki_bridge.yaml'
        )

        # Obtener el prefijo del launch argument
        prefix = LaunchConfiguration('prefix').perform(context)

        # Modificar el archivo .yaml con el prefijo
        modified_yaml_path = modify_yaml_with_prefix(original_yaml_path, prefix)

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='bridge_ros_gz',
            parameters=[
                {
                    'config_file': modified_yaml_path,  # Usar el archivo modificado
                    'use_sim_time': True,
                    'expand_gz_topic_names': True,  # Asegurarse que los nombres de tópicos se expandan correctamente
                }
            ],
            output='screen',
        )

        return [bridge]
    
    return []



def start_camera(context):
    if LaunchConfiguration('camera').perform(context) == 'true' and LaunchConfiguration('gazebo').perform(context) == 'true':
        camera_bridge_image = Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='bridge_gz_ros_camera_image',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=['/rgbd_camera/image'])

        camera_bridge_depth = Node(
            package='ros_gz_image',
            executable='image_bridge',
            name='bridge_gz_ros_camera_depth',
            output='screen',
            parameters=[{
                'use_sim_time': True,
            }],
            arguments=['/rgbd_camera/depth_image'])
        
        return [camera_bridge_image, camera_bridge_depth]
   
    return []

def generate_launch_description():

    kobuki_pkg = get_package_share_directory('kobuki_description')

    lidar_arg = DeclareLaunchArgument(
        'lidar', default_value='false',
        description='Enable lidar sensor')
    
    camera_arg = DeclareLaunchArgument(
        'camera', default_value='false',
        description='Enable camera sensor')

    structure_arg = DeclareLaunchArgument(
        'structure', default_value='true',
        description='Enable structure elements')

    prefix_arg = DeclareLaunchArgument(
        'prefix', default_value='',
        description='Prefix to apply to the topics')
    
    gazebo_arg = DeclareLaunchArgument(
        'gazebo', default_value='false',
        description='Enable gazebo plugins')

    description_file = DeclareLaunchArgument(
        'description_file',
        default_value=os.path.join(kobuki_pkg, 'urdf', 'kobuki.urdf.xacro'),
        description='Absolute path to the robot description file'
    )

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')

    namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Namespace to apply to the nodes'
    )


    robot_model = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('namespace'),
        parameters=[{
            'robot_description': launch_ros.descriptions.ParameterValue(
                Command([
                    'xacro ', LaunchConfiguration('description_file'),
                    ' lidar:=', LaunchConfiguration('lidar'),
                    ' camera:=', LaunchConfiguration('camera'),
                    ' structure:=', LaunchConfiguration('structure'),
                    ' prefix:=', LaunchConfiguration('prefix'),
                    ' gazebo:=', LaunchConfiguration('gazebo')
                ]), value_type=str),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
    )

    # TF Tree
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=LaunchConfiguration('prefix'),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    ld = LaunchDescription()
    ld.add_action(lidar_arg)
    ld.add_action(camera_arg)
    ld.add_action(structure_arg)
    ld.add_action(prefix_arg)
    ld.add_action(gazebo_arg)
    ld.add_action(description_file)
    ld.add_action(namespace_arg)
    ld.add_action(use_sim_time)
    ld.add_action(robot_model)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(OpaqueFunction(function=start_bridge))
    ld.add_action(OpaqueFunction(function=start_camera))

    return ld