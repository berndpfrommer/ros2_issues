# ros2_issues

Package to demonstrate ROS2 rmw performance problems

## How to install

Clone into your ROS2 workspace and build:
```
mkdir -p ~/ws/src
cd ~/ws/src
git clone git@github.com:berndpfrommer/ros2_issues.git
```

## How to run demo under ROS2
Prepare:
```
cd ~/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
. install/setup.bash
```
Run publisher (here with cyclone dds) in one terminal:
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run ros2_issues publisher_node --ros-args -p num_elements:=5000 -p rate:=1000
```
Parameters:
- ``num_elements``: number of elements in the array
- ``rate``: rate at which to publish
- ``q_size``: size of send queue

Now run rostopic hz in another window:
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 topic hz -w 100 /test_publisher/array
```

Check bandwidth and message size;
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 topic bw /test_publisher/array
```

## How to run demo under ROS1
Prepare:
```
cd ~/ws
catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
catkin build
. devel/setup.bash
```
Run publisher (here with cyclone dds) in one terminal:
```
rosrun ros2_issues publisher_node _num_elements:=5000 _rate:=1000
```
Parameters:
- ``num_elements``: number of elements in the array
- ``rate``: rate at which to publish
- ``q_size``: size of send queue

Now run rostopic hz in another window:
```
rostopic hz -w 100 /publisher_node/array
```

Check bandwidth and message size;
```
rostopic bw /publisher_node/array
```

## License

This software is issued under the Apache License Version 2.0.
