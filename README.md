# Bulldog Workspace

## Dependencies
[Intel(R) RealSense(TM) ROS Wrapper](https://github.com/IntelRealSense/realsense-ros)  
[Grasp Pose Detection (GPD)](https://github.com/atenpas/gpd)  
[mask_rcnn_ros](https://github.com/qixuxiang/mask_rcnn_ros)  

**Note**: You should install all the dependencies correctly before compiling the bulldog workspace.

## Steps to setup:  
cd bulldog_ws/gpg   
mkdir build && cd build  
cmake ..  
make  
sudo make install  
cd ..  
catkin_make  

## Test Mask_rcnn_ros with GPD
roslaunch bulldog_gazebo bulldog_empty_world.launch  
(manually add something in the vision of the robot, like a can of beer)  
roslaunch mask_rcnn_ros mask_rcnn_service.launch  
roslaunch gpd grasp_detection_service.launch  
roslaunch bulldog_gazebo_moveit_config bulldog_gazebo_planning_execution.launch  
rosrun gpd cloud_transform_server  
rosrun gpd bulldog_gpd.py  
