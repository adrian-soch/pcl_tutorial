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
cd <ROS_WS>/src
git clone https://github.com/adrian-soch/pcl_tutorial
colcon build
```

## Resources

 - Info on PCL, or for use witout ROS see : https://pcl.readthedocs.io/projects/tutorials/en/latest/.
 - Splitting ROS nodes into multiple files (compared to the basic tutorial): https://answers.ros.org/question/380079/how-to-write-header-file-for-ros-2-publisher/.
  


