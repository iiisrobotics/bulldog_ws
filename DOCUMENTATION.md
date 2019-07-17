# Documentation

## Installation

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

3. gpd

    Use 'gpd_mask_test.py~' instead of 'gdp_mask_test.py'

        roscd gpd
        cd scripts
        rm gpd_mask_test.py
        mv gpd_mask_test.py~ gpd_mask_test.py


4. Mask RCNN 

    Add suffix.

        roscd mask_rcnn_ros/nodes
        mv mask_rcnn_node mask_rcnn_node.py

    Add 'mask_rcnn_service' executable script.

    Add 'mask_rcnn_service.launch' launch file.

5. Caffe building

    "cudnn.hpp" (location: include/caffe/util/cudnn.hpp) may not be compatible 
    with the current version of cuDNN, you should update this file.

    Check the following ISSUE for more details: https://github.com/BVLC/caffe/issues/5793.

6. point cloud transform in GPD

    How to get the lastest frame transform?

    https://answers.ros.org/question/245111/proper-way-to-transform-a-point-cloud/

    Error example:
        
        [ERROR] [1562670811.260886536, 354.307000000]: Lookup would require extrapolation into the past.  Requested time 354.239000000 but the earliest data is at time 354.351000000, when looking up transform from frame [camera_frame_optical] to frame [base_link]

7. Intel Realsense

    1. Documentation
   
        Follow the official documentation to install the ROS wrapper for Realsense cameras from **source**!!!.

        https://github.com/IntelRealSense/realsense-ros.

        Note: We use version 2.2.1 to make it compatible with **indigo**. Remember to check it out.

    2. Dependencies

        You can find dependencies here: https://github.com/IntelRealSense/realsense-ros/blob/2.2.7/.travis.yml

        **ddynamic_reconfigure** cannot be installed via `apt-get` on Ubuntu 14.04.
        
        Try to install it from source: https://github.com/pal-robotics/ddynamic_reconfigure.

        You need to update `boost` to version 1.56. Follow the instructions on the official website:
        https://www.boost.org/doc/libs/1_56_0/doc/html/bbv2/installation.html.

    3. Build options

        Note to build with

            catkin_make
    
        instead of 

            catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
