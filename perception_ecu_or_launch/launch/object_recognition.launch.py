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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("container")
    add_launch_arg("camera_id")
    add_launch_arg("data_path", "/opt/autoware")
    add_launch_arg("model_file", "yolox-tiny.onnx")
    add_launch_arg("label_file", "label.txt")
    add_launch_arg("input_image_topic", "image_raw")
    add_launch_arg("use_intra_process", "True")

    composable_nodes = [
        ComposableNode(
            package="tensorrt_yolox",
            plugin="tensorrt_yolox::TrtYoloXNode",
            name=["tensorrt_yolox", LaunchConfiguration("camera_id")],
            namespace=["/perception/object_recognition/detection"],
            remappings=[
                (
                    "~/in/image",
                    [
                        "/sensing/camera/camera",
                        LaunchConfiguration("camera_id"),
                        "/image_raw",
                    ],
                ),
                (
                    "~/out/objects",
                    [
                        "/perception/object_recognition/detection/rois",
                        LaunchConfiguration("camera_id"),
                    ],
                ),
                (
                    "~/out/image",
                    [
                        "/perception/object_recognition/detection/rois",
                        LaunchConfiguration("camera_id"),
                        "/debug/image",
                    ],
                ),
                (
                    "~/out/image/compressed",
                    [
                        "/perception/object_recognition/detection/rois",
                        LaunchConfiguration("camera_id"),
                        "/debug/image/compressed",
                    ],
                ),
                (
                    "~/out/image/compressedDepth",
                    [
                        "/perception/object_recognition/detection/rois",
                        LaunchConfiguration("camera_id"),
                        "/debug/image/compressedDepth",
                    ],
                ),
                (
                    "~/out/image/theora",
                    [
                        "/perception/object_recognition/detection/rois",
                        LaunchConfiguration("camera_id"),
                        "/debug/image/theora",
                    ],
                ),
            ],
            parameters=[
                {
                    "model_path": [
                        LaunchConfiguration("data_path"),
                        "/",
                        LaunchConfiguration("model_file"),
                    ],
                    "label_path": [
                        LaunchConfiguration("data_path"),
                        "/",
                        LaunchConfiguration("label_file"),
                    ],
                    "precision": "fp16",
                },
            ],
            extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
        ),
    ]

    # If an existing container name is provided, load composable nodes into it
    # This will block until a container with the provided name is available and nodes are loaded
    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration("container"),
    )

    return LaunchDescription([*launch_arguments, load_composable_nodes])
