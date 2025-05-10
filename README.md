# Multi-Agent Drone Trajectory Replay in ROS 2 + Gazebo Harmonic

This project simulates **multi-drone trajectory replay** in a 3D environment using **ROS 2 Jazzy**, **Gazebo Harmonic**, and the `ros_gz` bridge. Drones are spawned using URDF models and replay precomputed trajectories from CSV files. Visual markers indicate goals and paths.

---

## 🛰️ Features

- 🧠 Trajectory replay for up to **32 drones**
- 📁 Reads CSV file with `(timestep, agent_id, x, y, z, yaw, pitch)` format
- 🌍 Custom Gazebo world with static obstacles
- ⚙️ ROS 2 node to publish poses using `/world/default/set_pose` via `ros_gz`
- 🎯 Tail and goal marker visualization through Gazebo model spawning
- 🔁 Seamless integration with ROS 2 and Gazebo using `ros_gz_bridge`

---

## 📁 Project Structure

```bash
gz_ros2_ws/
├── src/
│   ├── multi_drone_traj_replay/
│   │   ├── models/                  # SDF models for drones, markers
│   │   ├── worlds/                  # Custom Gazebo worlds
│   │   ├── multi_drone_traj_replay/ # Python node for trajectory replay
│   │   ├── launch/                  # Launch files
│   │   ├── traj_eval_Ours.csv       # Input trajectory
│   ├── ros_gz/                      # Gazebo <-> ROS 2 bridge (fork of ros_gz from Ignition)


🔧 Dependencies
ROS 2 Jazzy

Gazebo Harmonic

Python 3.10+

ros_gz bridge (added via .repos file)

🚀 Quick Start
1. Clone This Repo and Its Dependencies
bash
git clone https://github.com/YOUR_USERNAME/ros2_gz_multi_drone_replay_sim.git
cd ros2_gz_multi_drone_replay_sim

2. Build the Workspace
bash

colcon build
source install/setup.bash
3. Launch the Simulation World
bash
gz sim src/multi_drone_traj_replay/worlds/plane_world.sdf
4. Start the ROS-Gazebo Bridge

bash
ros2 run ros_gz_bridge parameter_bridge \
  /world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose@gz.msgs.Pose@gz.msgs.Boolean
5. Spawn Drones


python3 src/multi_drone_traj_replay/spawn.py
6. Replay Trajectories


ros2 launch multi_drone_traj_replay full_replay.launch.py
🧠 Trajectory CSV Format
Each row represents the state of a drone at a given timestep:
timestep,agent_id,x,y,z,yaw,pitch
0,0,5.0,1.0,0.0,0.0
0,1,6.0,1.5,0.0,0.1
...
🎯 Visual Markers
🟡 Tail markers: small yellow spheres dropped at each timestep

❌ Goal markers: red cross placed at the final location of each drone

🧩 ROS 2 Node
The traj_replay_node.py:

Sends pose commands using the /world/default/set_pose service

Spawns SDF-based visual markers

Uses a high-frequency timer to step through the CSV

📦 About ros_gz
The ros_gz bridge provides communication between ROS 2 and Gazebo:

Publishes/subscribes to Gazebo topics/services

Used to set drone positions and spawn models

Automatically downloaded via ros2_gz_multi_drone_replay_sim.repos

If you want to use a custom version, fork https://github.com/gazebosim/ros_gz.

📡 RQT Graph

📜 License
MIT License.

🤝 Contributing
Contributions, pull requests, and stars are welcome!

🗂 Future Work
Integrate reinforcement learning for control

Add safety via Control Barrier Functions (CBFs)

Enable real-time collision detection



## 🔗 Dependencies

Clone `ros_gz` before building:

```bash
cd src
git clone https://github.com/gazebosim/ros_gz.git

cd ..
colcon build
source install/setup.bash

