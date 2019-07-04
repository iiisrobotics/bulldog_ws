#!/usr/bin/env python

from __future__ import absolute_import
from __future__ import print_function
from __future__ import division

import numpy as np

import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from mask_rcnn_ros.srv import MaskDetect
from gpd.srv import detect_grasps
from gpd.msg import CloudIndexed
from geometry_msgs.msg import Point
from std_msgs.msg import Int64

import moveit_commander
import geometry_msgs.msg

if __name__ == "__main__":
    rospy.init_node("gpd_mask")

    rospy.loginfo("Getting image...")
    image = rospy.wait_for_message("/camera/rgb/image_raw", Image)
    pc2 = rospy.wait_for_message("/camera/depth/points", PointCloud2)

    rospy.loginfo("Waiting for Mask RCNN service...")
    rospy.wait_for_service('mask_rcnn_srv')

    response = None
    try:
        rospy.loginfo("Calling Mask RCNN service...")
        mask_srv = rospy.ServiceProxy('mask_rcnn_srv', MaskDetect)
        response = mask_srv(image)
    except rospy.ServiceException, e:
        rospy.loginfo("Mask RCNN service call failed: %s" % e)

    cloud_indexed = CloudIndexed()
    if response is not None and len(response.detection.masks) > 0:
        for class_name, mask in zip(response.detection.class_names,
                                    response.detection.masks):
            if class_name == 'bottle':
                one_idx = np.nonzero(mask)
                one_idx = np.ravel_multi_index(one_idx)
                cloud_indexed.indices = one_idx.tolist()

                indices = []
                for i in range(mask.step * mask.height):
                    if mask.data[i] == '\xff':
                        indices.append(Int64(i))
                break
        else:
            rospy.loginfo("No bottle found!")
    else:
        rospy.loginfo("Detect nothing!")

    cloud_indexed.cloud_sources.cloud = pc2
    cloud_indexed.cloud_sources.view_points.append(Point(0,0,0))
    cloud_indexed.cloud_sources.camera_source.append(Int64(0))

    print("generating grasp...")
    rospy.wait_for_service('/detect_grasps_server/detect_grasps')
    detect_grasp = rospy.ServiceProxy(
        '/detect_grasps_server/detect_grasps', detect_grasps)
    gpd_response = detect_grasp(cloud_indexed)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("left_arm")

    pose_target = geometry_msgs.msg.Pose()
    pose_target.orientation.x = 0
    pose_target.orientation.y = 0
    pose_target.orientation.z = 0.6
    pose_target.orientation.w = 0.8

    pose_target.position = gpd_response.grasp_configs.grasps[0].bottom
    group.set_pose_target(pose_target)

    plan = group.plan()

    rospy.loginfo("============> Waiting while RVIZ displays plan...")
