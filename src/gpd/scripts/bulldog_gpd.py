#!/usr/bin/env python
import rospy
import numpy as np

from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from gpd.msg import CloudIndexed
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header
from std_msgs.msg import Int64

from mask_rcnn_ros.srv import MaskDetect
from gpd.srv import DetectGrasps
from gpd.srv import CloudTransform

import moveit_commander
import tf.transformations
import sensor_msgs.point_cloud2


POSE_REFERENCE_FRAME = 'odom'
GRASP_FILTERING_FRAME = 'left_arm_base'  # 'left_gripper_tool0', 'left_arm_base'


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
	"""Find reasonable grasps among all the cloud samples.

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

	try:
		rospy.loginfo("Generating grasps...")
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
		Point cloud in the robot base_link frame.

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
	# get NAN points from the point cloud
	#
	cloud_points = []
	for cloud_point in sensor_msgs.point_cloud2.read_points(cloud_transformed):
		cloud_points.append([cloud_point[0], cloud_point[1], cloud_point[2]])
	cloud_points = np.asarray(cloud_points)
	nan_mask = np.isnan(cloud_points[:, 0])
	# print(cloud_points.shape)
	# cloud_points = cloud_points[~nan_mask]
	# print(cloud_points.shape)

	if len(detection.masks) > 0:
		for class_name, mask in zip(detection.class_names,
									detection.masks):
			#
			# name filtering
			#
			mask_indices = name_filtering(class_name, mask, nan_mask)

			if len(mask_indices) > 0:
				cloud_indices = mask_indices
				cloud_indexed.indices = [Int64(idx) for idx in cloud_indices]
				break

			#
			# least squares filtering, extract the nonplanar indices
			#
			# if len(mask_indices) > 0:
			# 	least_squares_indices = least_squares_filtering(cloud_points,
			# 													mask_indices)
			# else:
			# 	continue

			# if len(mask_indices) > 0 and len(least_squares_indices) > 0:
			# 	cloud_indices = least_squares_indices
			# 	print("Number of indices in filtered cloud: %d" %
			# 		len(cloud_indices))

			# 	cloud_indexed.indices = [Int64(idx) for idx in cloud_indices]
			# 	break
		else:
			rospy.logfatal("No bottle or cup found!")
			raise SystemExit()
	else:
		rospy.logfatal("Detect nothing!")
		raise SystemExit()

	return cloud_indexed


def name_filtering(class_name, mask, nan_mask):
	"""Extract point cloud of typical objects.

	Parameters
	----------
	- class_name: str
		Name of the detection from the Mask RCNN.

	- mask: sensor_msgs/Image
		Mask of the corresponding detection from the Mask RCNN.

	- nan_mask: numpy.ndarray
		Boolean mask to indicate the NAN points in the point cloud.

	Returns
	----------
	- mask_indices: list
		List of index indicating the valid point cloud through Mask RCNN.

	"""
	if (class_name == 'bottle') or (class_name == 'cup'):
		mask_data = np.fromstring(mask.data, dtype=np.uint8)
		mask_indices = np.where((mask_data != 0) & ~nan_mask)[0].tolist()
	else:
		mask_indices = []

	return mask_indices


def least_squares_filtering(cloud_points, mask_indices, dist_thresh=4e-4):
	"""Extract the nonplanar indices through least squares fitting.

	Parameters
	----------
	- cloud_points: list
		List of points in the point cloud.

	- mask_indices: list
		List of index indicating the valid point cloud through Mask RCNN.

	- dist_thresh: float (optional, default = 0.0004)
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
	mask_points = cloud_points[mask_indices]
	mask_idxs = np.array(mask_indices)

	#
	# solve least squares fitting
	#
	mat = np.stack([
		mask_points[:, 0],
		mask_points[:, 1],
		np.ones(mask_points.shape[0])
	], axis=1)
	vec = mask_points[:, 2]
	coeff, _, _, _ = np.linalg.lstsq(mat, vec, rcond=None)

	#
	# distance between the fitting plane and points
	#
	dist = np.square(coeff[0] * mask_points[:, 0] +
					 coeff[1] * mask_points[:, 1] +
					 coeff[2] -
					 mask_points[:, 2])
	least_squares_indices = mask_idxs[dist >= dist_thresh].tolist()

	return least_squares_indices


def find_closest_grasp(grasps, frame='odom',
					   min_y_thresh=-0.05, 
					   min_y_soft_thresh=0.08,
					   min_dist_thresh=0.81):
	"""Find the closest grasp to a specified frame.

	Parameters
	----------
	- grasps: gpd.msg.GraspConfig
		Grasps describe by their 6-DOF pose, consisting of a 3-DOF position
		and 3-DOF orientation, and the opening width of the robot hand.

	- frame: str (optional, default = 'odom' (a.k.a. 'base_link') )
		Name of the reference frame.

	- min_y_thresh: float (optional, default = -0.05)
		Minimum value for closest grasp along y axis, i.e. the left arm cannot
		move to the right side of the robot, and the right arm cannot
		move to the left side of the robot. Remember to define this threshold
		in 'odom' frame.

	- min_y_soft_thresh: float (optional, default = 0.08)
		Minimum value for closest grasp along y axis which we begin to consider
		distance threshold, i.e. when targer position's y value is in
		[min_y_soft_thresh, min_y_thresh), we consider the planar distance of
		the target position from the arm base frame.

	- min_dist_thresh: float (optional, default = 0.81)
		Max value for closest grasp from the are base link, i.e. the arms
		cannot go farther than this distance.

	Returns
	----------
	- closest_grasp: gpd.msg.GraspConfig
		The closest grasp in the specified frame.

	"""
	#
	# We use the pose of the gripper base for planning.
	# The cloud represents points in 'odom' frame by default.
	#
	pose_positions = [PointStamped(
		header=Header(frame_id=POSE_REFERENCE_FRAME),
		point=grasp.bottom
	) for grasp in grasps]
	if frame != POSE_REFERENCE_FRAME:
		listener = tf.TransformListener()

		listener.waitForTransform(
			frame, POSE_REFERENCE_FRAME, rospy.Time(0), rospy.Duration(1))
		try:
			pose_positions = [listener.transformPoint(frame, pose_position)
							  for pose_position in pose_positions]
		except (tf.LookupException,
				tf.ConnectivityException,
				tf.ExtrapolationException) as e:
			rospy.logerr("%s" % e)
			raise SystemExit()
	pose_plannar_positions = [[pose_position.point.x,
							   pose_position.point.y]
							  for pose_position in pose_positions]

	dist = np.linalg.norm(pose_plannar_positions, axis=1)
	closest_dist = np.min(dist)
	rospy.loginfo("Closest Distance: %f." % closest_dist)
	closest_idx = np.argmin(dist)
	closest_grasp = grasps[closest_idx]

	#
	# thresholding
	#
	if closest_dist >= min_dist_thresh:
		if closest_grasp.bottom.y <= min_y_thresh:
			rospy.logerr("%s arm cannot move to the other side of the robot!" %
						("Left" if min_y_thresh < 0 else "Right"))
			rospy.logerr("Target position: (%f, %f, %f)." %
						(closest_grasp.bottom.x,
						closest_grasp.bottom.y,
						closest_grasp.bottom.z))
			rospy.logerr("Please move the robot first!")
			raise SystemExit()
		elif min_y_soft_thresh >= closest_grasp.bottom.y > min_y_thresh:
			rospy.logerr("%s arm cannot move farther!" %
					 	 ("Left" if min_y_thresh < 0 else "Right"))
			rospy.logerr("Target plannar distance: %f." % closest_dist)
			rospy.logerr("Please move the robot first!")
			raise SystemExit()

	return closest_grasp


def configure_target_pose(target_position, target_quaternion):
	"""Configure target pose for planning

	Parameters
	----------
	- target_position: geometry_msgs.msg.Point
		Position of the target pose.

	- target_quaternion: geometry_msgs.msg.Quaternion
		Rotation of the target pose.

	Returns
	----------
	target_pose: geometry_msgs.msg.PoseStamped
		Target pose with a header.

	Notes
	----------
	pose: 
		position: 
			x: 0.98714264072
			y: -0.0181142373857
			z: 0.480302787791
		orientation: 
			x: 0.6859901647729069
			y: 0.7275965355007923
			z: 0.004507587996639602
			w: 0.0006760270237895833

	"""
	target_pose = PoseStamped()
	target_pose.header.stamp = rospy.get_time()
	target_pose.header.frame_id = POSE_REFERENCE_FRAME
	target_pose.pose.position = target_position
	# target_pose.pose.position.x = 0.98714264072
	# target_pose.pose.position.y = -0.0181142373857
	# target_pose.pose.position.z = 0.480302787791 + 0.3
	target_pose.pose.orientation = target_quaternion
	# target_pose.pose.orientation.x = 0.6859901647729069
	# target_pose.pose.orientation.y = 0.7275965355007923
	# target_pose.pose.orientation.z = 0.004507587996639602
	# target_pose.pose.orientation.w = 0.0006760270237895833

	return target_pose


def main():
	"""Main entrance

	Parameters
	----------

	Returns
	----------

	"""
	#
	# node initialization
	#
	rospy.init_node('gpd_mask')
	image_stream = rospy.get_param(
		'image_stream', 'left_gripper_sensor_d415_camera/color/image_raw')
	cloud_stream = rospy.get_param(
		'image_stream', 'left_gripper_sensor_d415_camera/depth_registered/points')

	#
	# get image and point cloud
	#
	rospy.loginfo("Getting image and cloud...")
	image = rospy.wait_for_message(image_stream, Image)
	cloud = rospy.wait_for_message(cloud_stream, PointCloud2)
	rospy.loginfo("Image and cloud are available.")

	#
	# transform point cloud form camera frame to odom (a.k.a. base_link) frame.
	#
	cloud_transformed = cloud_transformation(
		"cloud_transform_server/transformation",
		cloud
	)

	#
	# Mask RCNN detection
	#
	detection = mask_rcnn_detection("mask_rcnn/detection", image)

	#
	# process point cloud through Mask RCNN detection
	#
	cloud_indexed = process_cloud(cloud_transformed, detection)

	#
	# find reasonable grasps
	#
	grasp_configs = grasps_detection("detect_grasps_server/detect_grasps",
									 cloud_indexed)

	#
	# grasp filtering: find the closest grasp first
	#
	grasp = find_closest_grasp(grasp_configs.grasps, GRASP_FILTERING_FRAME)

	#
	# bring up MoveIt motion planning context
	#
	try:
		moveit_commander.RobotCommander()
		group = moveit_commander.MoveGroupCommander("left_arm")
	except RuntimeError as e:
		rospy.logerr("%s" % e)
		rospy.logerr("Please startup MoveIt planning context first!")
		raise SystemExit()

	#
	# interpret pose configuration
	#
	# We use the pose of the gripper base for planning.
	target_position = grasp.bottom
	target_rotation_matrix = np.array([
		[grasp.approach.x, grasp.binormal.x, grasp.axis.x, 0.0],
		[grasp.approach.y, grasp.binormal.y, grasp.axis.y, 0.0],
		[grasp.approach.z, grasp.binormal.z, grasp.axis.z, 0.0],
		[0.0, 0.0, 0.0, 1.0]
	])
	target_quaternion = tf.transformations.quaternion_from_matrix(
		target_rotation_matrix)
	# take inverse quaternion to rotate the gripper in the odom
	# (a.k.a. base_link) frame
	target_quaternion = Quaternion(x=target_quaternion[0],
								   y=target_quaternion[1],
								   z=target_quaternion[2],
								   w=target_quaternion[3])

	#
	# configure target pose
	#
	# target_position = None
	# target_quaternion = None
	target_pose = configure_target_pose(target_position, target_quaternion)

	print("Target pose:")
	print(target_pose)

	#
	# planning
	#
	group.set_pose_target(target_pose)

	plan = group.plan()
	# plan = group.go(wait=True)
	# group.execute(plan, wait=True)

	rospy.loginfo("============ Waiting while RVIZ displays motion planning...")
	rospy.sleep(3.0)


if __name__ == "__main__":
	main()
