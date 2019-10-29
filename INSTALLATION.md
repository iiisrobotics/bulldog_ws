# Installation

## Dependencies

1. ros-control

    http://wiki.ros.org/ros_control

        sudo apt-get install ros-$ROS_DISTRO-ros-control

2. robot_localization

    http://wiki.ros.org/robot_localization

        sudo apt-get install ros-$ROS_DISTRO-robot-localization

3. controller_manager

    http://wiki.ros.org/controller_manager

        sudo apt-get install ros-$ROS_DISTRO-controller-manager

4. soem

    http://wiki.ros.org/soem

        sudo apt-get install ros-$ROS_DISTRO-soem

5. moveit

    http://wiki.ros.org/moveit_ros
    
        sudo apt-get install ros-$ROS_DISTRO-moveit-ros

6. moveit_planners_ompl

    http://wiki.ros.org/moveit_planners_ompl

        sudo apt-get install ros-$ROS_DISTRO-moveit-planners-ompl

7. openni2_camera

    http://wiki.ros.org/openni2_camera

    	sudo apt-get install ros-$ROS_DISTRO-openni2-camera

8. socketcan_interface

    http://wiki.ros.org/socketcan_interface

        sudo apt-get install ros-$ROS_DISTRO-socketcan-interface

9. industrial_msgs

    http://wiki.ros.org/industrial_msgs

        sudo apt-get install ros-$ROS_DISTRO-industrial-msgs

10. twist_mux

    http://wiki.ros.org/twist_mux

        sudo apt-get install ros-$ROS_DISTRO-twist-mux

11. lms1xx

    http://wiki.ros.org/LMS1xx

        sudo apt-get install ros-$ROS_DISTRO-lms1xx

12. ddynamic_reconfigure

        sudo apt-get install ros-$ROS_DISTRO-ddynamic-reconfigure

13. interactive_marker_twist_server

    http://wiki.ros.org/interactive_marker_twist_server

        sudo apt-get install ros-$ROS_DISTRO-interactive-marker-twist-server

14. gazebo-ros-pkgs

    http://wiki.ros.org/gazebo_plugins

        sudo apt-get install ros-$ROS_DISTRO-gazebo-ros-pkgs

15. gazebo-ros-control

    http://wiki.ros.org/gazebo_ros_control

        sudo apt-get install ros-$ROS_DISTRO-gazebo-ros-control

16. ros-controllers

    http://wiki.ros.org/ros_controllers

        sudo apt-get install ros-$ROS_DISTRO-ros-controllers

17. hector_gazebo_plugins

    http://wiki.ros.org/hector_gazebo_plugins

        sudo apt-get install ros-$ROS_DISTRO-hector-gazebo-plugins

----

You can run

    git submodule init
    git submodule update

to clone all the following repository at once!

17. realsense-ros

    Intel RealSense RGBD sensors' ROS library.

    https://github.com/IntelRealSense/realsense-ros

    Follow the documentation to install realsense wrappers for ROS.

    Note: I have modify the code from official repository. If you have
    any question, feel free to contact me.

18. gazebo_grasp_fix_plugin/general-message-pkgs

    General message packages to use the gazebo grasp fix plugin.

    https://github.com/JenniferBuehler/general-message-pkgs.git


19. gazebo_grasp_fix_plugin/gazebo-pkgs

    Gazebo grasp fix plugin.

    https://github.com/JenniferBuehler/gazebo-pkgs.git

20. moveit_grasps

    Moveit grasping pipeline for box size object only.

    https://github.com/YeeCY/moveit_grasps.

    Note: we use **kinetic-devel** branch here.
21. pick_and_place

    Bulldog pick and place pipeline.

    https://github.com/YeeCY/pick_and_place

    Note: we use **kinetic-devel** branch here.

22. darknet_ros

    Yolo realtime object detector with ROS. We use it to find different objects
    in grasping pipeline.

    https://github.com/YeeCY/darknet_ros.git

    Note: we use **kinetic-devel** branch here.

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

1. Gazebo hanging

    If you wait the Gazebo to startup for a long time without response, that means you
    don't have models to spawn the environment.

    Go to https://bitbucket.org/osrf/gazebo_models, download all the models, and put them
    into 

2. Invalid parameter "prefix" for Realsense D415

    The original D415 URDF contains no prefix for multiple instantiation spawning, you
    need to add prefix parameter to the macro descrpiton and modify the defintion of
    "aluminum" material.

3. No p gain specified for pid.

    This kind of errors are bugs of Gazebo7. It has been removed in Gazebo9 with ROS Melodic.
    **Just ignore them!!!**

4. mask_rcnn_ros incompatible with Keras >= 2.0.8

        cd <workspace>/src/mask_rcnn_ros/node
        vim model.py

    Type the following command to replace all 'topology' with 'saving'

        :%s/topology/saving/g
    
    Save and quit

        :wq

5. Imported target "opencv_xphoto" includes non-existent path ...

    Modify "OpenCVConfig.cmake" fiel

        cd /opt/ros/indigo/share/OpenCV-3.1.0-dev
        sudo vim OpenCVConfig.cmake

    In line 113 delete "CACHE"

    i.e from
        
        get_filename_component(OpenCV_CONFIG_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH CACHE)

    to

        get_filename_component(OpenCV_CONFIG_PATH "${CMAKE_CURRENT_LIST_FILE}" PATH)

6. gpd mask test

    Use 'gpd_mask_test.py~' instead of 'gdp_mask_test.py'

        roscd gpd
        cd scripts
        rm gpd_mask_test.py
        mv gpd_mask_test.py~ gpd_mask_test.py


7. mask_rcnn_ros incompatible with Keras >= 2.0.8

        cd <workspace>/src/mask_rcnn_ros/node
        vim model.py

    Type the following command to replace all 'topology' with 'saving'

        :%s/topology/saving/g
    
    Save and quit

        :wq

8. ur_modern_driver

    Error: 
    
        'const struct hardware_interface::ControllerInfo’ has no member named ‘hardware_interface’

    Solution:

        https://github.com/ros-industrial/ur_modern_driver/issues/135

9. Disable self collisions between following links with `bulldog_robot_moveit_config_rs/launch/setup_assistant` to avoid planning failures.

    left_gripper_finger_1_link_1 - left_gripper_finger_1_link_paraproximal_actuating_hinge
    left_gripper_finger_middle_link_1 - left_gripper_finger_middle_link_paraproximal_actuating_hinge
    left_gripper_finger_1_link_0 - left_gripper_finger_1_link_median_bar
    left_gripper_finger_2_link_paraproximal_actuating_hinge - left_gripper_finger_2_link_1
    right_gripper_finger_1_link_1 - right_gripper_finger_1_link_paraproximal_actuating_hinge
    right_gripper_finger_middle_link_1 - right_gripper_finger_middle_link_paraproximal_actuating_hinge
    right_gripper_finger_1_link_0 - right_gripper_finger_1_link_median_bar
