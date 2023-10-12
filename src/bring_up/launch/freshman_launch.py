from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="platform",
            executable="object",
            name="my_platform",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="global_planner_task",
            executable="global_planner",
            name="global_planner_1",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="local_planner_task",
            executable="local_planner",
            name="local_planner_1",
            output="screen",
            emulate_tty=True
        ),
        Node(
            package="behavior_decision",
            executable="btree",
            name="behavior_decision_1",
            output="screen",
            emulate_tty=True
        ),
    ])