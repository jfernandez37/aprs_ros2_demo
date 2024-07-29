Before starting, the teach pendant must be in remote mode and the computer you are using must be connected to the APRS lab network.

First, source humble: `source /opt/ros/humble/setup.bash` . Then, to start the motoman microros agent, run this command:

`docker run \
  -it \
  --rm \
  --net=host \
  --user=$(id -u):$(id -g) \
  microros/micro-ros-agent:humble \
    udp4 \
    --port 8230`

Open a second terminal, enter the command `ros2 topic list` and you should see a few topics including `joint_states` and `robot_status`.

Notes when generating a trajectory:

Velocity must be filled, but a list of 7 zeroes works. Efforts do not not need to be filled. To execute a trajectory, a follow joint message must be built and sent to the `follow_joint_trajectory` action server.