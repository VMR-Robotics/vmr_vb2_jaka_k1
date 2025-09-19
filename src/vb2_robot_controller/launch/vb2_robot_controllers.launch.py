import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pathlib import Path
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("vb2_robot_description", package_name="vb2_robot_controller").to_moveit_configs()
    moveit_config_package_path = Path(moveit_config.package_path)

    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))


    # 机器人状态发布器的启动文件，用于发布机器人的状态信息
    ld.add_action(DeclareLaunchArgument("publish_frequency", default_value="15.0"))
    ld.add_action(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            respawn=True,
            output="screen",
            parameters=[
                moveit_config.robot_description,
                {
                    "publish_frequency": LaunchConfiguration("publish_frequency"),
                },
            ],
        )
    )

    # 启动 RViz 文件，如果 use_rviz 参数为 true，则启动 RViz 可视化工具
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config_package_path / 'config/vb2_robot.rviz'),
        )
    )
    ld.add_action(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # ros2_control_node 用于仿真或驱动机器人的控制器
    ld.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                str(moveit_config_package_path / "config/ros2_controllers.yaml"),
            ],
            remappings=[
                ("/controller_manager/robot_description", "/robot_description"),
            ],
        )
    )

    # 启动控制器
    ld.add_action(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[ 
                "joint_state_broadcaster",
                "left_arm_controller",
                "right_arm_controller",
                "waist_group_controller",
                "head_group_controller",
                "left_hand_controller",
                "right_hand_controller",
                "chassis_controller"

            ],
            output="screen",
        )
    )


    return ld
