# Installation

## Dependencies

1. ros-control

    http://wiki.ros.org/ros_control

        sudo apt-get install ros-$ROS_DISTRO-ros-control

2. object_recognition_msgs

    http://wiki.ros.org/object_recognition_msgs

        sudo apt-get install ros-$ROS_DISTRO-object-recognition-msgs

3. soem

    http://wiki.ros.org/soem

        sudo apt-get install ros-$ROS_DISTRO-soem

4. moveit

    http://wiki.ros.org/moveit_ros
    
        sudo apt-get install ros-$ROS_DISTRO-moveit-ros

5. QT4

        sudo apt-get install qt4-default

7. socketcan_interface

        sudo apt-get install ros-$ROS_DISTRO-socketcan-interface

8. universal_robot

    redirect into bulldog worspace.

        cd <path-to-workspace>

    clone the git repository from https://github.com/ros-industrial/universal_robot.

        git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git

    Note: we use **melodic-devel** branch here. And compile it from scratch.

9.  robotiq

    redirect into bulldog worspace.

        cd <path-to-workspace>

    clone the git repository from https://github.com/ros-industrial/robotiq.

        git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/robotiq.git
    
    Note: we use **kinect-devel** branch here. And compile it from scratch.

## Step by Step

1. clean the workspace

        cd <workspace>
        rm -rfv build devel
        cd <workspace>/src
        rm -rfv mask_rcnn_ros
        git clone https://github.com/qixuxiang/mask_rcnn_ros.git

2. install dependencies

        cd <workspace>
        catkin_make

    Follow the error prompts and install all the packages needed. If the CMAKE complains
    about the packages you have already installed, use `catkin_make clean` to clean
    the workspace and try again.

3. install "bulldog" package

        cd <workspace>
        catkin_make

## Troubleshooting

1. mask_rcnn_ros incompatible with Keras >= 2.0.8

        cd <workspace>/src/mask_rcnn_ros/node
        vim model.py

    Type the following command to replace all 'topology' with 'saving'

        :%s/topology/saving/g
    
    Save and quit

        :wq

2. Imported target "opencv_xphoto" includes non-existent path ...

    Modify "OpenCVConfig.cmake" fiel

        cd /opt/ros/indigo/share/OpenCV-3.1.0-dev
        sudo vim OpenCVConfig.cmake

    In line 113 delete "CACHE"

    i.e from
        
        get_filename_component(OpenCV_CONFIG_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH CACHE)

    to

        get_filename_component(OpenCV_CONFIG_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH)

3. gpd mask test

    Use 'gpd_mask_test.py~' instead of 'gdp_mask_test.py'

        roscd gpd
        cd scripts
        rm gpd_mask_test.py
        mv gpd_mask_test.py~ gpd_mask_test.py


4. Mask RCNN

    Add suffix.

        roscd mask_rcnn_ros/nodes
        mv mask_rcnn_node mask_rcnn_node.py

    Add 'mask_rcnn_service' node.

5. ur_modern_driver

    Error: 
    
        'const struct hardware_interface::ControllerInfo’ has no member named ‘hardware_interface’

    Solution:

        https://github.com/ros-industrial/ur_modern_driver/issues/135

6. universal_robot

    Error:

        fatal error: moveit_msgs/GetKinematicSolverInfo.h: No such file or directory
            #include <moveit_msgs/GetKinematicSolverInfo.h>

7. 

