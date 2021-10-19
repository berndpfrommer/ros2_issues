# ros2_issues

Package to demonstrate ROS2 rmw performance problems

## How to install

Clone into your ROS2 workspace and build:
```
mkdir -p ~/ws/src
cd ~/ws/src
git clone git@github.com:berndpfrommer/ros2_issues.git
```

## How to run demo under ROS2 using cyclone DDS
Prepare:
```
cd ~/ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
. install/setup.bash
```
Point to cyclone config file:
```
cd ~/ws/src/ros2_issues
export CYCLONEDDS_URI=file://`pwd`/cyclonedds_config.xml
```
Run publisher in one terminal (for standard message tests):
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run ros2_issues publisher_node --ros-args -p num_elements:=50000 -p rate:=1000
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

To test column-major messages with a more realistic scenario, run the publisher with ``-2`` argument:
```
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 run ros2_issues publisher_node -2 --ros-args -p num_elements:=50000 -p rate:=1000
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
rosrun ros2_issues publisher_node _num_elements:=50000 _rate:=1000
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
