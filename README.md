# pcl_tutorial
Tutorial for using Point Cloud Library (PCL) with ROS 2.

Code adapted for ROS 2 from [ROS Industrial: Building a Perception Pipeline](https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/Building-a-Perception-Pipeline.html).

## Getting Started
Install ROS 2, create a workspace then install the following packages:
```
sudo apt install ros-galactic-pcl-ros
sudo apt install ros-galactic-pcl-conversions
```

Clone this repo into a ROS 2 workspace and build: 

```
source <ROS_WS>/install.setup.bash
cd <ROS_WS>/src
git clone https://github.com/adrian-soch/pcl_tutorial
colcon build
```
### Running the code
After building the easiest way to start the node is through the launch file, change the paramters inside `processing_node.launch.py` then:

```
ros2 launch pcl_tutorial processing_node.launch.py
```

Then start a PointCloud2 Publisher, this can be from a real hardware, from a simulation, or a rosbag. To play a rosbag that has pointcloud data recorded:

```
ros2 bag play <PATH_TO_BAG>
```

Optionally, you can pause rosbags, or play them 1 message at a time:
```
ros2 service call /rosbag2_player/toggle_paused rosbag2_interfaces/TogglePaused

ros2 service call /rosbag2_player/play_next rosbag2_interfaces/PlayNext
```

### Debugging

An easy way to debug ROS 2 C++ programs is through VSCode.
1. Build the package with simlinks, and the build type flag
1. Run the ROS node with gdb prefix (or add to launch file)
1. Add the following to the `launch.json' in the `.vscode` folder

```
colcon build --packages-select lesson_perception --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

ros2 run --prefix 'gdbserver localhost:3000' lesson_perception perception_node

Create VSCode `launch.json`
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "ROS2 C++ Debugger",
            "request": "launch",
            "type": "cppdbg",
            "miDebuggerServerAddress": "localhost:3000",
            "cwd": "/",
            "program": "/home/<PATH_TO_ROS_WS>/install/<PACKAGE>/lib/<PACKAGE>/<EXECUTABLE>"
        }
    ]
}

# Run debugger in vscode
```

## Resources

 - Info on PCL, or for use witout ROS see : https://pcl.readthedocs.io/projects/tutorials/en/latest/.
 - Splitting ROS nodes into multiple files (compared to the basic tutorial): https://answers.ros.org/question/380079/how-to-write-header-file-for-ros-2-publisher/.
  


