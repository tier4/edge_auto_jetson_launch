# Copyright 2022 Tier IV, Inc. All rights reserved.
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

from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import yaml


def create_launch_arg(name: str, default_value=None):
    return DeclareLaunchArgument(name, default_value=default_value)


def create_include_launch(path: list, args: dict, cond=IfCondition("True")):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=args.items(), condition=cond)


def create_node(pkg: str, exec: str, ns: str, remaps=None, params=None, cond=IfCondition("True")):
    return Node(package=pkg, executable=exec, namespace=ns, output="screen", remappings=remaps, parameters=params, condition=cond)


def generate_arguments():

    actions = []

    actions.append(create_launch_arg("camera_modes_path", PathJoinSubstitution(
        [FindPackageShare("perception_ecu_launch"), "config/camera_modes.yaml"])))

    actions.append(create_launch_arg("sensing_namespace", "/sensing/camera"))
    actions.append(create_launch_arg("image_topic_name", "image_raw"))
    actions.append(create_launch_arg("vehicle_id", "default"))
    actions.append(create_launch_arg("use_sensor_data_qos", "False"))

    actions.append(create_launch_arg("launch_sensor_trigger", "True"))

    actions.append(create_launch_arg("perception_namespace",
                   "/perception/object_recognition/detection"))
    yolox_default_basedir = [FindPackageShare("tensorrt_yolox"), "data"]
    actions.append(create_launch_arg("yolox_model_path", PathJoinSubstitution(
        yolox_default_basedir + ["yolox-tiny.onnx"])))
    actions.append(create_launch_arg("yolox_label_path",
                   PathJoinSubstitution(yolox_default_basedir + ["label.txt"])))

    actions.append(create_launch_arg("launch_image_view", "True"))

    return actions


def generate_camera_processes(context: LaunchContext):

    actions = []

    camera_modes_path = context.launch_configurations["camera_modes_path"]
    enable_trigger = bool(context.launch_configurations["launch_sensor_trigger"])
    enable_viewer = bool(context.launch_configurations["launch_image_view"])

    try:
        with open(camera_modes_path, "r") as f:
            camera_modes = yaml.safe_load(f)
    except Exception as e:
        print(e)
        return

    def create_individual_param_path(filename):
        config_basedir_path = PathJoinSubstitution(
            [FindPackageShare("individual_params"), "config", LaunchConfiguration("vehicle_id")])
        return PathJoinSubstitution([config_basedir_path, camera_name, filename])

    def create_launch_path(pkgname, filename):
        return PathJoinSubstitution([FindPackageShare(pkgname), "launch", filename])

    def create_container_name(camera_name):
        return "camera_container_{}".format(camera_name)

    def create_rois_name(camera_name):
        return camera_name.replace("camera", "rois")

    for camera_name in camera_modes.keys():

        camera_mode = camera_modes[camera_name]

        enable_object_recognition = False
        if camera_mode == "disable":
            continue
        elif camera_mode == "image":
            enable_object_recognition = False
        elif camera_mode == "object_recognition":
            enable_object_recognition = True
        else:
            raise NotImplementedError(camera_mode)

        container_name = create_container_name(camera_name)
        rois_name = create_rois_name(camera_name)

        config_v4l2_path = create_indivisual_param_path("v4l2_camera.param.yaml")
        config_info_path = create_individual_param_path("camera_info.yaml")
        config_trigger_path = create_individual_param_path("trigger.param.yaml")

        launch_container_path = create_launch_path(
            "perception_ecu_launch", "camera_container.launch.py")
        launch_driver_path = create_launch_path("v4l2_camera", "v4l2_camera.launch.py")
        launch_object_recognition_path = create_launch_path(
            "perception_ecu_launch", "object_recognition.launch.py")

        actions.append(create_include_launch(
            path=launch_container_path,
            args={
                "container": container_name,
            },
        ))

        actions.append(create_include_launch(
            path=launch_driver_path,
            args={
                "container": container_name,
                "camera_info_url": config_info_path,
                "camera_name": camera_name,
                "image_topic": LaunchConfiguration("image_topic_name"),
                "use_sensor_data_qos": LaunchConfiguration("use_sensor_data_qos"),
                "v4l2_camera_namepace": LaunchConfiguration("sensing_namespace"),
                "v4l2_camera_param_path": config_v4l2_path,
            },
        ))

        actions.append(create_node(
            pkg="sensor_trigger",
            exec="sensor_trigger_exe",
            ns=PathJoinSubstitution([LaunchConfiguration("sensing_namespace"), camera_name]),
            params=[config_trigger_path],
            cond=IfCondition(str(enable_trigger)),
        ))

        actions.append(create_include_launch(
            path=launch_object_recognition_path,
            args={
                "container": container_name,
                "namespace": LaunchConfiguration("perception_namespace"),
                "input_topic": PathJoinSubstitution([LaunchConfiguration("sensing_namespace"), camera_name, LaunchConfiguration("image_topic_name")]),
                "output_topic_name": rois_name,
                "model_path": LaunchConfiguration("yolox_model_path"),
                "label_path": LaunchConfiguration("yolox_label_path"),
            },
            cond=IfCondition(str(enable_object_recognition)),
        ))

        actions.append(create_node(
            pkg="image_view",
            exec="image_view",
            ns=PathJoinSubstitution([LaunchConfiguration("perception_namespace"), rois_name]),
            remaps=[
                ("image", PathJoinSubstitution([LaunchConfiguration(
                    "perception_namespace"), rois_name, "debug/image"]))
            ],
            cond=IfCondition(str(enable_viewer and enable_object_recognition)),
        ))

        actions.append(create_node(
            pkg="image_view",
            exec="image_view",
            ns=PathJoinSubstitution([LaunchConfiguration("sensing_namespace"), camera_name]),
            remaps=[
                ("image", PathJoinSubstitution([LaunchConfiguration(
                    "sensing_namespace"), camera_name, LaunchConfiguration("image_topic_name")]))
            ],
            cond=IfCondition(str(enable_viewer and not enable_object_recognition)),
        ))

    return actions


def generate_launch_description():

    return LaunchDescription(generate_arguments() + [OpaqueFunction(function=generate_camera_processes)])
