#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Dokozoyanonukko
# SPDX-License-Identifier: BSD-3-Clause

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

def generate_launch_description():

    event_node = launch_ros.actions.Node(
        package='mypkg',
        executable='event',
        name='event',
        output='screen'
    )

    judge_node = launch_ros.actions.Node(
        package='mypkg',
        executable='judge',
        name='judge',
        output='screen'
    )

    status_node = launch_ros.actions.Node(
        package='mypkg',
        executable='status',
        name='status',
        output='screen'
    )

    response_node = launch_ros.actions.Node(
        package='mypkg',
        executable='response',
        name='response',
        output='screen'
    )

    return launch.LaunchDescription([
        event_node,
        judge_node,
        status_node,
        response_node
    ])
