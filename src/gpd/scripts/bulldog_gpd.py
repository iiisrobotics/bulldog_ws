#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from gpd.msg import CloudIndexed
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int64

from mask_rcnn_ros.srv import MaskDetect
from gpd.srv import DetectGrasps
from gpd.srv import CloudTransform

import sensor_msgs.point_cloud2
import moveit_commander
import tf.transformations


def cloud_transformation(srv, cloud):
	"""Transform point cloud into robot frame.

	Parameters
	----------
	- srv: str
		Name of the point cloud transform service.

	- cloud: sensor_msgs.msg.PointCloud2
		Point cloud in the local frame.

	Returns
	----------
	- cloud_transformed: sensor_msgs.msg.PointCloud2
		Point cloud in the robot frame.

	"""
	rospy.loginfo("Waiting for point cloud transform service...")
	rospy.wait_for_service(srv)
	rospy.loginfo("Point cloud transform service is available.")

	cloud_transform_response = None
	try:
		rospy.loginfo("Transforming point cloud...")
		cloud_transformer = rospy.ServiceProxy(srv, CloudTransform)
		cloud_transform_response = cloud_transformer(cloud)
	except rospy.ServiceException as e:
		rospy.logerr("Point cloud transform service call failed: %s" % e)
		raise SystemExit()
	cloud_transformed = cloud_transform_response.cloud_transformed
	
	return cloud_transformed


def mask_rcnn_detection(srv, image):
	"""Detection objects in a image by calling Mask RCNN.

	Parameters
	----------
	- srv: str
		Name of the Mask RCNN service.
	
	- image: sensor_msgs.msg.Image
		Image message acquired from the camera.

	Returns
	----------
	- detection: mask_rcnn_ros.msg.Detection
		Detection result of the image.

	"""
	rospy.loginfo("Waiting for Mask RCNN service...")
	rospy.wait_for_service(srv)
	rospy.loginfo("Mask RCNN service is available.")

	mask_rcnn_response = None
	try:
		rospy.loginfo("Mask RCNN Detecting...")
		mask_rcnn_detector = rospy.ServiceProxy(srv, MaskDetect)
		mask_rcnn_response = mask_rcnn_detector(image)
	except rospy.ServiceException as e:
		rospy.logerr("Mask RCNN service call failed: %s" % e)
		raise SystemExit()
	detection = mask_rcnn_response.detection

	return detection


def grasps_detection(srv, cloud_indexed):
	"""Find reasonable grasp candidates among all the samples.

	Parameters
	----------
	- srv:
		Name of the grasps detection service.

	- cloud_indexed: gpd.msg.CloudIndexed
		The transformed point cloud and a list of indices into it at which to
		sample grasp candidates. 

	Returns
	----------
	- grasp_configs: gpd.msg.GraspConfigList
		A list of grasp configurations each of which describes a grasp by its 
		6-DOF pose, consisting of a 3-DOF position and 3-DOF orientation, and 
		the opening width of the robot hand.

	"""
	rospy.loginfo("Waiting for grasp detection service...")
	rospy.wait_for_service(srv)
	rospy.loginfo("Grasp detection service is available.")

	detect_grasps_response = None
	try:
		rospy.loginfo("Generating grasp...")
		grasps_detector = rospy.ServiceProxy(srv, DetectGrasps)
		detect_grasps_response = grasps_detector(cloud_indexed)
	except rospy.ServiceException as e:
		rospy.logerr("Grasp detection service call failed: %s" % e)
		raise SystemExit()
	grasp_configs = detect_grasps_response.grasp_configs

	return grasp_configs


def process_cloud(cloud_transformed, detection):
	"""Process the transformed point cloud with respect to the Mask RCNN 
	detection.
	
	Parameters
	----------
	- cloud_transformed: sensor_msgs.msg.PointCloud2
		Point cloud in the robot frame.

	- detection: mask_rcnn_ros.msg.Detection
		Detection result of the image.

	Returns
	----------
	- cloud_indexed: gpd.msg.CloudIndexed
		The transformed point cloud and a list of indices into it at which to
		sample grasp candidates. 

	"""
	cloud_indexed = CloudIndexed()
	cloud_indexed.cloud_sources.cloud = cloud_transformed
	cloud_indexed.cloud_sources.view_points.append(Point(0, 0, 0))
	cloud_indexed.cloud_sources.camera_source.append(Int64(0))

	#
	# convert PointCloud2 to a list of points
	#
	cloud_points = np.reshape(
		np.fromstring(cloud_transformed.data, dtype=np.float32), [-1, 4]
	)
	# odd rows and first three columns are informative
	cloud_points = cloud_points[::2, 0:3]
	# remove nans
	cloud_points = cloud_points[~np.isnan(cloud_points[:, 0])]
	print("Number of indices in original cloud: %d" % len(cloud_points))

	if len(detection.masks) > 0:
		for class_name, mask in zip(detection.class_names,
									detection.masks):
			#
			# name filtering
			#
			mask_indices = name_filtering(class_name, mask)

			#
			# least squares filtering, extract the nonplanar indices
			#
			if len(mask_indices) > 0:
				least_squares_indices = least_squares_filtering(cloud_points)
			else:
				continue

			if len(mask_indices) > 0 and len(least_squares_indices) > 0:
				# TODO: Filter after Mask RCNN
				cloud_indices = mask_indices + least_squares_indices
				print("Number of indices in filtered cloud: %d" %
					len(cloud_indices))

				cloud_indexed.indices = [Int64(idx) for idx in cloud_indices]
				break
		else:
			rospy.logfatal("No bottle or cup found!")
			raise SystemExit()
	else:
		rospy.logfatal("Detect nothing!")
		raise SystemExit()

	return cloud_indexed


def name_filtering(class_name, mask):
	"""Extract point cloud of typical objects.

	Parameters
	----------
	- class_name: str
		Name of the detection from the Mask RCNN.

	- mask: sensor_msgs/Image
		Mask of the corresponding detection from the Mask RCNN.

	Returns
	----------
	- mask_indices: list
		List of index indicating the valid point cloud through Mask RCNN.

	"""
	if (class_name == 'bottle') or (class_name == 'cup'):
		mask_data = np.fromstring(mask.data, dtype=np.uint8)
		mask_indices = np.where(mask_data != 0)[0]
	else:
		mask_indices = []

	return mask_indices


def least_squares_filtering(cloud_points, dist_thresh=0.01):
	"""Extract the nonplanar indices through least squares fitting.

	Parameters
	----------
	- cloud_points: list
		List of points in the point cloud.

	- dist_thresh: float
		Threshold of the distances between the fitting plane and points.

	Returns
	----------
	- least_squares_indices: list
		List of index indicating the valid point cloud through least squares
		fitting.

	Note
	----------
	The normal plane equation is a * x + b * y + c * z + d = 0, we assume
	c = -1 and let a * x + b * y + d = z to solve the least squares fitting.

	"""
	cloud_points = np.copy(cloud_points)

	#
	# solve least squares fitting
	#
	A = np.stack([
		cloud_points[:, 0],
		cloud_points[:, 1],
		np.ones(cloud_points.shape[0])
	], axis=1)
	b = cloud_points[:, 2]
	coeff, _, _, _ = np.linalg.lstsq(A, b, rcond=None)

	#
	# distance between the fitting plane and points
	#
	dist = np.square(coeff[0] * cloud_points[:, 0] +
					 coeff[1] * cloud_points[:, 1] +
					 coeff[2] -
					 cloud_points[:, 2])
	least_squares_indices = np.where(dist >= dist_thresh)[0]

	return least_squares_indices


def main():
	"""Main entrance

	Parameters
	----------

	Returns
	----------

	"""
	rospy.init_node("gpd_mask")

	rospy.loginfo("Getting image...")
	image = rospy.wait_for_message("left_gripper_sensor_d415_camera/color/image_raw", Image)
	cloud = rospy.wait_for_message("left_gripper_sensor_d415_camera/depth/color/points", PointCloud2)

	# tf_buffer = tf2_ros.Buffer()
	# tf_listener = tf2_ros.TransformListener(tf_buffer)
	# trans = tf_buffer.lookup_transform(
	# 	"base_link", pc2.header.frame_id, rospy.Time(), rospy.Duration(2.0))
	# pc2_new = do_transform_cloud(pc2, trans)
	# try:
	# 	mask_srv = rospy.ServiceProxy("pc_transform_srv", pc_transform)
	# 	pc_transform_res = mask_srv(pc2)
	# except rospy.ServiceException, e:
	# 	rospy.loginfo "service call failed: %s"%e
	# rospy.loginfo("transformed pc")
	# pc2 = pc_transform_res.out_cloud

	cloud_transformed = cloud_transformation(
		"cloud_transform_server/transformation",
		cloud
	)
	
	detection = mask_rcnn_detection("mask_rcnn/detection", image)

	cloud_indexed = process_cloud(cloud_transformed, detection)

	grasp_configs = grasps_detection("detect_grasps_server/detect_grasps",
									 cloud_indexed)

	# get transform from camera to bask link
	# listener = tf.TransformListener()
	# listener.waitForTransform(
	# 	pc2.header.frame_id, "base_link", rospy.Time(), rospy.Duration(2.0))
	# (trans, rot) = listener.lookupTransform(
	# 	"base_link",
	# 	pc2.header.frame_id,
	# 	rospy.Time.now(),
	# 	rospy.Duration(2.0)
	# )

	# bottom_stamped = PointStamped()
	# bottom_stamped.header.frame_id = pc2.header.frame_id
	# bottom_stamped.header.stamp = rospy.Time(0)
	# bottom_stamped.point = gpd_response.grasp_configs.grasps[0].bottom
	# p = listener.transformPoint("base_link", bottom_stamped)

	group = None
	try:
		moveit_commander.RobotCommander()
		group = moveit_commander.MoveGroupCommander("left_arm")
	except moveit_commander.MoveItCommanderException as e:
		rospy.logerr("%s", e)
		rospy.logerr("Please startup MoveIt planning context first!")
		raise SystemExit()

	#
	# interpret pose configuration
	#
	# find the closest grasp first
	pose_positions = [grasp.bottom for grasp in grasp_configs.grasps]
	positions = [[position.x, position.y, position.z]
					  for position in pose_positions]
	dist = np.linalg.norm(positions, axis=1)
	min_dist_idx = np.argmin(dist)

	# print(min_dist_idx)
	grasp = grasp_configs.grasps[min_dist_idx]
	pose_position = pose_positions[min_dist_idx]
	pose_rotation_matrix = np.array([
		[grasp.approach.x, grasp.binormal.x, grasp.axis.x, 0.0],
		[grasp.approach.y, grasp.binormal.y, grasp.axis.y, 0.0],
		[grasp.approach.z, grasp.binormal.z, grasp.axis.z, 0.0],
		[0.0, 0.0, 0.0, 1.0]
	])

	print(pose_rotation_matrix)
	pose_quaternion = tf.transformations.quaternion_from_matrix(
		pose_rotation_matrix)

	# take inverse quaternion to rotate in odom (a.k.a. base_link) frame
	pose_quaternion = Quaternion(x=pose_quaternion[0], y=pose_quaternion[1],
								 z=pose_quaternion[2], w=-pose_quaternion[3])

	current_pose = group.get_current_pose()
	print("Current pose:")
	print(current_pose)

	target_pose = PoseStamped()
	target_pose.header.stamp = rospy.get_time()
	target_pose.header.frame_id = group.get_pose_reference_frame()
	target_pose.pose.position = pose_position
	# target_pose.pose.position.x = 0.920550534598
	# target_pose.pose.position.y = -0.214042859617
	# target_pose.pose.position.z = 0.457450517917
	target_pose.pose.orientation = pose_quaternion
	# target_pose.pose.orientation.x = 0.7976158200489984
	# target_pose.pose.orientation.y = 0.6031400140057149
	# target_pose.pose.orientation.z = 0.005281569338155196
	# target_pose.pose.orientation.w = -0.0017978148058544854

	# pose: 
	# 	position: 
	# 		x: 0.920550534598
	# 		y: -0.214042859617
	# 		z: 0.457450517917
	# 	orientation: 
	# 		x: 0.7976158200489984
	# 		y: 0.6031400140057149
	# 		z: 0.005281569338155196
	# 		w: -0.0017978148058544854

	# print("End effector frame: %s" % group.get_end_effector_link())

	print("Target pose:")
	print(target_pose)

	group.set_pose_target(target_pose)

	# plan = group.go(wait=True)
	plan = group.plan()
	# group.execute(plan, wait=True)

	rospy.loginfo("============ Waiting while RVIZ displays motion planning...")
	rospy.sleep(3.0)


if __name__ == "__main__":
	main()
