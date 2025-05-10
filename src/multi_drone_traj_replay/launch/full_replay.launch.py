from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='multi_drone_traj_replay',
            executable='traj_replay_node',
            output='screen'
        )
    ])


# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     return LaunchDescription([
#         # ROS-GZ Bridge Node
#         # Node(
#         #     package='ros_gz_bridge',
#         #     executable='parameter_bridge',
#         #     name='bridge_node',
#         #     arguments=[
#         #         '/clock@gz.msgs.Clock@rosgraph_msgs/msg/Clock',
#         #         '/world/default/spawn_entity@ros_gz_interfaces/srv/SpawnEntity',
#         #         '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose'
#         #     ],
#         #     output='screen'
#         # ),

#         # Your Drone Trajectory Replay Node
#         Node(
#             package='multi_drone_traj_replay',
#             executable='traj_replay_node',
#             name='trajectory_replayer',
#             output='screen'
#         )
#     ])
