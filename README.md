# pcl_tutorial
Tutorial for using Point Cloud Library (PCL) with ROS 2.

Code adapted for ROS 2 from [ROS Industrial: Building a Perception Pipeline](https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/Building-a-Perception-Pipeline.html).

<img src="https://user-images.githubusercontent.com/6884645/227073828-3c013dbb-7dfb-47a7-97af-400fb76cb268.png" height="160" />

![Screenshot from 2023-03-22 21-02-56](https://user-images.githubusercontent.com/6884645/227073304-b5e61753-bbda-4aab-89bc-43532cbe2d79.png)

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

### Visualization


1. To view the point cloud topics, run `rviz2` in a new terminal.

1. Click `Add` near the bottom right, select the `By topic` tab, and then select the point cloud 2 topic that you want to see.

2. Go to `Global Options -> Fixed Frame` and select the frame the data is in. Otherwise there will be a `Global Status` error that prevents you from seeing the data.

3. To make LiDAR points easier to see go to `PointCloud2 -> Style` and select `Points` from the dropdown menu. Feel free to try changing any other settings

1. You can save the current configuration and rviz2 will open to this configuration by default. This can be changed at any time.

<img src="https://user-images.githubusercontent.com/6884645/227073525-df459713-0294-4f44-a154-9ad2990aff16.png" height="400" />

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
  


