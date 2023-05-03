# Copyright 2023 Open Source Robotics Foundation, Inc.
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

import launch.actions
import launch.event_handlers
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    receiver_action = launch_ros.actions.Node(
        package='ssce_messages_not_processed',
        executable='receiver',
        output='screen')
    return LaunchDescription([
        receiver_action,
        launch.actions.RegisterEventHandler(
            launch.event_handlers.OnProcessExit(
                target_action=receiver_action,
                on_exit=[
                    launch.actions.LogInfo(msg=["Receiver process exited, stopping the test..."]),
                    launch.actions.EmitEvent(event=launch.events.Shutdown()),
                ],
            ),
        ),
        launch_ros.actions.Node(
            package='ssce_messages_not_processed',
            executable='sender.py',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
        ),
    ])
