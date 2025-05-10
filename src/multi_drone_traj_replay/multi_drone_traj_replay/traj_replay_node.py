#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose
import pandas as pd
import math
import subprocess
import time

class TrajectoryReplayer(Node):
    def __init__(self):
        super().__init__('trajectory_replayer')

        self.df = pd.read_csv('/home/rezapirayesh/gz_ros2_ws/src/multi_drone_traj_replay/traj_eval_Ours.csv')
        self.max_step = self.df['timestep'].max()
        self.num_drones = self.df['agent_id'].max() + 1
        self.timer_period = 0.001
        self.current_step = 0

        self.pose_client = self.create_client(SetEntityPose, '/world/default/set_pose')
        while not self.pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for set_pose service...')

        self.tail_points = {i: [] for i in range(self.num_drones)}

        self.spawn_goal_markers()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def spawn_goal_markers(self):
        last_df = self.df[self.df['timestep'] == self.max_step]
        for _, row in last_df.iterrows():
            name = f"goal_{int(row['agent_id'])}"
            self.spawn_model('goal_marker', name, row['x'], row['y'], row['z'])

    def spawn_model(self, model_name, instance_name, x, y, z):
        sdf_file = f"/home/rezapirayesh/gz_ros2_ws/src/multi_drone_traj_replay/models/{model_name}/model.sdf"
        pose = f"position: {{x: {x}, y: {y}, z: {z}}}"
        cmd = [
            "gz", "service", "-s", "/world/default/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "300",
            "--req", f'sdf_filename: "{sdf_file}", name: "{instance_name}", pose: {{{pose}}}'
        ]
        self.get_logger().info(f"Spawning {instance_name} at ({x}, {y}, {z})")
        subprocess.run(cmd)
        time.sleep(0.1)

    def spawn_tail_point(self, drone_id, x, y, z):
        name = f"tail_{drone_id}_{len(self.tail_points[drone_id])}"
        self.spawn_model("tail_marker", name, x, y, z)

    def timer_callback(self):
        if self.current_step > self.max_step:
            self.get_logger().info('Trajectory complete.')
            self.destroy_timer(self.timer)
            return

        step_df = self.df[self.df['timestep'] == self.current_step]

        for _, row in step_df.iterrows():
            agent_id = int(row['agent_id'])

            # Tail marker
            pt = (row['x'], row['y'], row['z'])
            self.tail_points[agent_id].append(pt)
            self.spawn_tail_point(agent_id, *pt)

            # Move drone
            req = SetEntityPose.Request()
            req.entity.name = f"drone_{agent_id}"
            req.pose.position.x = float(row['x'])
            req.pose.position.y = float(row['y'])
            req.pose.position.z = float(row['z'])

            yaw = float(row['yaw'])
            req.pose.orientation.z = math.sin(yaw * 0.5)
            req.pose.orientation.w = math.cos(yaw * 0.5)

            self.pose_client.call_async(req)

        self.current_step += 100


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
