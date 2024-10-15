from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        # joyノードの起動設定
        Node(
            package='joy',  # joyのパッケージ名 (標準的なjoyパッケージを使う場合)
            executable='joy_node',  # joyの実行可能ファイル名
            name='joy_one',
            output='screen',
            remappings=[('/joy', '/joy_one')],
        ),
        # controllerノードの起動設定
        Node(
            package='robocon2024-1',  # controllerのパッケージ名
            executable='controller_one',  # controllerの実行可能ファイル名
            name='controller_one',
            output='screen',
        ),

    ])
