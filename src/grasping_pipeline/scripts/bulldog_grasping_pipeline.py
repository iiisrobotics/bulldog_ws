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
from pick_and_place.srv import PickAndPlace

import moveit_commander
import tf.transformations
import sensor_msgs.point_cloud2

import actionlib
import darknet_ros_msgs.msg


IMAGE_SHAPE = None
OBJECT_DETECTION_TIMEOUT = 5.0

IMAGE_STREAM_DEFAULT_NAME = 'left_gripper_sensor_d415_camera/color/image_raw'
CLOUD_STREAM_DEFAULT_NAME = 'left_gripper_sensor_d415_camera/depth_registered/points'

CLOUD_TRANSFROM_SERVER_DEFAULT_NAME = 'cloud_transform_server/transformation'
MASK_RCNN_SERVICE_DEFAULT_NAME = 'mask_rcnn/detection'
YOLO_ACTION_DEFAULT_NAME = 'darknet_ros/check_for_objects'
DETECT_GRASPS_SERVICE_DEFAULT_NAME = 'detect_grasps_server/detect_grasps'
PICK_AND_PLACE_SERVICE_DEFAULT_NAME = 'bulldog_pick_and_place/pick_and_place'

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
	rospy.loginfo("[GraspingPipeline] Waiting for point cloud transform service...")
	rospy.wait_for_service(srv)
	rospy.loginfo("[GraspingPipeline] Point cloud transform service is available.")

	try:
		rospy.loginfo("[GraspingPipeline] Transforming point cloud...")
		cloud_transformer = rospy.ServiceProxy(srv, CloudTransform)
		cloud_transform_response = cloud_transformer(cloud)
	except rospy.ServiceException as e:
		rospy.logerr("[GraspingPipeline] Point cloud transform service call failed: %s" % e)
		raise SystemExit()
	cloud_transformed = cloud_transform_response.cloud_transformed
	
	return cloud_transformed


def mask_rcnn_detection(srv, image):
	"""Detection objects in a image using Mask RCNN.

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
	rospy.loginfo("[GraspingPipeline] Waiting for Mask RCNN service...")
	rospy.wait_for_service(srv)
	rospy.loginfo("[GraspingPipeline] Mask RCNN service is available.")

	try:
		rospy.loginfo("[GraspingPipeline] Mask RCNN Detecting...")
		mask_rcnn_detector = rospy.ServiceProxy(srv, MaskDetect)
		mask_rcnn_response = mask_rcnn_detector(image)
	except rospy.ServiceException as e:
		rospy.logerr("[GraspingPipeline] Mask RCNN service call failed: %s" % e)
		raise SystemExit()
	detection = mask_rcnn_response.detection

	return detection


def yolo_detection(act, image):
	"""Detection objects in a image using YOLO detector.

	Parameters
	----------
	- act: str
		Namespaces of the YOLO detection action.
	
	- image: sensor_msgs.msg.Image
		Image message acquired from the camera.

	Returns
	----------
	- bounding_boxes: darknet_ros_msgs.msg.BoundingBoxes
		Bounding box of each detected object.

	"""
	yolo_client = actionlib.SimpleActionClient(
		act, darknet_ros_msgs.msg.CheckForObjectsAction)

	rospy.loginfo("[GraspingPipeline] Waiting for Yolo action...")
	yolo_client.wait_for_server()
	rospy.loginfo("[GraspingPipeline] Yolo action is available.")

	goal = darknet_ros_msgs.msg.CheckForObjectsGoal(
		id=0, image=image)

	#
	# actual goal and result
	#
	yolo_client.send_goal(goal)
	done = yolo_client.wait_for_result()
	if not done:
		rospy.logerr("[GraspingPipeline] Yolo detection action call failed!")
		raise SystemExit()
	result = yolo_client.get_result()

	bounding_boxes = result.bounding_boxes.bounding_boxes
	return bounding_boxes


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
	rospy.loginfo("[GraspingPipeline] Waiting for grasp detection service...")
	rospy.wait_for_service(srv)
	rospy.loginfo("[GraspingPipeline] Grasp detection service is available.")

	try:
		rospy.loginfo("[GraspingPipeline] Generating grasps...")
		grasps_detector = rospy.ServiceProxy(srv, DetectGrasps)
		detect_grasps_response = grasps_detector(cloud_indexed)
	except rospy.ServiceException as e:
		rospy.logerr("[GraspingPipeline] Grasp detection service call failed: %s" % e)
		raise SystemExit()
	grasp_configs = detect_grasps_response.grasp_configs

	return grasp_configs


def pick_and_place(srv, grasp_poses):
	"""Pick and place objects with possible grasp poses

	Parameters
	----------
	- srv: str
		Name of the pick and place service.

	- grasp_poses: [geometry_msgs.msg.PoseStamped, ...]
		Grasp poses in the global frame.

	Returns
	----------
	- success: boolean
		True on success.

	"""
	rospy.loginfo("[GraspingPipeline] Waiting for pick and place service...")
	rospy.wait_for_service(srv)
	rospy.loginfo("[GraspingPipeline] Pick and place service is available.")

	try:
		rospy.loginfo("[GraspingPipeline] Pick and place...")
		pick_and_place_pipeline = rospy.ServiceProxy(srv, PickAndPlace)
		pick_and_place_response = pick_and_place_pipeline(grasp_poses)
	except rospy.ServiceException as e:
		rospy.logerr("[GraspingPipeline] Pick and place service call failed: %s" % e)
		raise SystemExit()
	success = pick_and_place_response.success
	
	return success


def mask_rcnn_process_cloud(cloud_transformed, detection):
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
	print(cloud_points.shape)
	cloud_points = cloud_points[~nan_mask]
	print(cloud_points.shape)
	# cloud_points[:, 0] -= 0.1 # offset along x axis
	# cloud_indexed.cloud_sources.cloud = sensor_msgs.point_cloud2.create_cloud_xyz32(
	# 	cloud_transformed.header, cloud_points.tolist()
	# )

	if len(detection.masks) > 0:
		for class_name, mask in zip(detection.class_names,
									detection.masks):
			#
			# name filtering
			#
			mask_indices = mask_rcnn_name_filtering(class_name, mask, nan_mask)

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
			rospy.logerr("[GraspingPipeline] No bottle or cup found!")
			raise SystemExit()
	else:
		rospy.logerr("[GraspingPipeline] Detect nothing!")
		raise SystemExit()

	return cloud_indexed


def process_cloud(cloud_transformed, image_size, bounding_boxes):
	"""Process the transformed point cloud with respect to the Mask RCNN 
	detection.
	
	Parameters
	----------
	- cloud_transformed: sensor_msgs.msg.PointCloud2
		Point cloud in the robot base_link frame.

	- image_size: (int, int)
		Size of the image

	- bounding_boxes: darknet_ros_msgs.msg.BoundingBoxes
		Bounding box of each detected object.

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

	if len(bounding_boxes) > 0:
		for bounding_box in bounding_boxes:
			#
			# name filtering
			#
			mask_indices = name_filtering(
				bounding_box.Class,
				image_size,
				bounding_box.xmin,
				bounding_box.ymin,
				bounding_box.xmax,
				bounding_box.ymax,
				nan_mask
			)

			if len(mask_indices) > 0:
				cloud_indices = mask_indices
				cloud_indexed.indices = [Int64(idx) for idx in cloud_indices]
				break
		else:
			rospy.logerr("[GraspingPipeline] No bottle or cup in the view!")
			raise SystemExit()

	return cloud_indexed


def mask_rcnn_name_filtering(class_name, mask, nan_mask):
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


def name_filtering(object_name,
				   image_size,
				   x_min, y_min, x_max, y_max,
				   nan_mask):
	"""Extract point cloud of typical objects.

	Parameters
	----------
	- object_name: str
		Name of the detected object.

	- image_size: (int, int)

	- x_min: float
		Top-left x coordinate of the bounding box.

	- y_min: float
		Top-left y coordinate of the bounding box.

	- x_max: float
		Bottom-right x coordinate of the bounding box.

	- y_max: float
		Bottom-right y coordinate of the bounding box.

	- nan_mask: numpy.ndarray
		Boolean mask to indicate the NAN points in the point cloud.

	Returns
	----------
	- valid_indices: list
		List of index indicating the valid point cloud through Mask RCNN.

	"""
	if (object_name == 'bottle') or (object_name == 'cup'):
		object_mask = np.zeros(image_size, dtype=np.bool)
		object_mask[y_min:y_max, x_min:x_max] = True
		object_mask = np.reshape(object_mask, -1)
		valid_indices = np.where((object_mask != 0) & ~nan_mask)[0].tolist()
	else:
		valid_indices = []

	return valid_indices


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


def find_closest_grasp(grasps, frame='base_link',
					   min_y_thresh=-0.05):
	"""Find the closest grasp to a specified frame.

	Parameters
	----------
	- grasps: gpd.msg.GraspConfig
		Grasps describe by their 6-DOF pose, consisting of a 3-DOF position
		and 3-DOF orientation, and the opening width of the robot hand.

	- frame: str (optional, default = 'base_link')
		Name of the reference frame.

	- min_y_thresh: float (optional, default = -0.05)
		Minimum value for closest grasp along y axis, i.e. the left arm cannot
		move to the right side of the robot, and the right arm cannot
		move to the left side of the robot. Remember to define this threshold
		in 'base_link' frame.

	Returns
	----------
	- closest_grasp: gpd.msg.GraspConfig
		The closest grasp in the specified frame.

	"""
	#
	# We use the pose of the gripper base for planning.
	# The cloud represents points in 'base_link' frame by default.
	#
	pose_positions = [PointStamped(
		header=Header(frame_id=POSE_REFERENCE_FRAME),
		point=grasp.bottom
	) for grasp in grasps]
	# pose_positions = [PointStamped(
	# 	header=Header(frame_id=POSE_REFERENCE_FRAME),
	# 	point=grasp.bottom
	# ) for grasp in grasps if -grasp.axis.z <= 0.0]
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
			rospy.logerr("[GraspingPipeline] %s" % e)
			raise SystemExit()
	if len(pose_positions) == 0:
		rospy.logerr("[GraspingPipeline: No grasp found!")
		raise SystemExit()

	pose_plannar_positions = [[pose_position.point.x,
							   pose_position.point.y]
							  for pose_position in pose_positions]

	dist = np.linalg.norm(pose_plannar_positions, axis=1)
	closest_dist = np.min(dist)
	rospy.loginfo("[GraspingPipeline] Closest Distance: %f." % closest_dist)
	closest_idx = np.argmin(dist)
	closest_grasp = grasps[closest_idx]

	#
	# thresholding
	#
	if closest_grasp.bottom.y <= min_y_thresh:
		rospy.logerr("[GraspingPipeline] %s arm cannot move to the other side of the robot!" %
					("Left" if min_y_thresh < 0 else "Right"))
		rospy.logerr("[GraspingPipeline] Target position: (%f, %f, %f)." %
					(closest_grasp.bottom.x,
					closest_grasp.bottom.y,
					closest_grasp.bottom.z))
		rospy.logerr("[GraspingPipeline] Please move the robot first!")
		raise SystemExit()

	return closest_grasp


def configure_grasp_poses(grasp_configs):
	"""Configure grasp poses for planning

	Parameters
	----------
	- grasp_configs: gpd.msg.GraspConfigList
		A list of grasp configurations each of which describes a grasp by its 
		6-DOF pose, consisting of a 3-DOF position and 3-DOF orientation, and 
		the opening width of the robot hand.

	Returns
	----------
	target_poses: [geometry_msgs.msg.PoseStamped, ...]
		Target pose with a header.

	Notes
	----------
	pose: 
		position: 
			x: 0.896101885954
			y: 0.220898042324
			z: 0.440645337477
		orientation: 
			x: 0.628300287611576
			y: -0.39971341565013935
			z: -0.5883724060255893
			w: -0.3150965657765391
	"""
	grasp_poses = []
	for grasp in grasp_configs.grasps:
		grasp_position = grasp.bottom
		grasp_rotation_matrix = np.array([
			[-grasp.binormal.x, -grasp.axis.x, grasp.approach.x, 0.0],
			[-grasp.binormal.y, -grasp.axis.y, grasp.approach.y, 0.0],
			[-grasp.binormal.z, -grasp.axis.z, grasp.approach.z, 0.0],
			[0.0, 0.0, 0.0, 1.0]
		])

		grasp_quaternion = tf.transformations.quaternion_from_matrix(
			grasp_rotation_matrix)
		# take inverse quaternion to rotate the gripper in the base_link
		grasp_quaternion = Quaternion(x=grasp_quaternion[0],
									  y=grasp_quaternion[1],
									  z=grasp_quaternion[2],
									  w=grasp_quaternion[3])

		grasp_pose = PoseStamped()
		grasp_pose.header.stamp = rospy.Time.now()
		grasp_pose.header.frame_id = POSE_REFERENCE_FRAME
		grasp_pose.pose.position = grasp_position
		# grasp_pose.pose.position.x = 0.795009083413
		# grasp_pose.pose.position.y = 0.22062083617
		# grasp_pose.pose.position.z = 0.489056381231
		grasp_pose.pose.orientation = grasp_quaternion
		# grasp_pose.pose.orientation.x = -0.08121355241516261
		# grasp_pose.pose.orientation.y = -0.052377064390203044
		# grasp_pose.pose.orientation.z = -0.5590192309531775
		# grasp_pose.pose.orientation.w = 0.8235037956527536
		grasp_poses.append(grasp_pose)

	return grasp_poses


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
		'image_stream', IMAGE_STREAM_DEFAULT_NAME)
	cloud_stream = rospy.get_param(
		'cloud_stream', CLOUD_STREAM_DEFAULT_NAME)

	#
	# get image and point cloud
	#
	rospy.loginfo("[GraspingPipeline] Getting image and cloud...")
	image = rospy.wait_for_message(image_stream, Image)
	cloud = rospy.wait_for_message(cloud_stream, PointCloud2)
	rospy.loginfo("[GraspingPipeline] Image and cloud are available.")


	#
	# transform point cloud form camera frame to base_link frame.
	#
	cloud_transformed = cloud_transformation(
		CLOUD_TRANSFROM_SERVER_DEFAULT_NAME, cloud)

	#
	# Mask RCNN detection
	#
	# detection = mask_rcnn_detection(MASK_RCNN_SERVICE_DEFAULT_NAME, image)

	#
	# Yolo detection
	#
	start_time = rospy.Time.now()
	while (rospy.Time.now() - start_time).to_sec() <= OBJECT_DETECTION_TIMEOUT:
		bounding_boxes = yolo_detection(YOLO_ACTION_DEFAULT_NAME, image)
		if len(bounding_boxes) > 0:
			break
		else:
			rospy.logerr("[GraspingPipeline] No object found!")
	else:
		rospy.logerr("[GraspingPipeline] Object detection timeout!")
		raise SystemExit()

	#
	# process point cloud through Mask RCNN detection
	#
	# cloud_indexed = mask_rcnn_process_cloud(cloud_transformed, detection)
	image_size = (image.height, image.width)
	cloud_indexed = process_cloud(
		cloud_transformed, image_size, bounding_boxes)

	#
	# find reasonable grasps
	#
	grasp_configs = grasps_detection(DETECT_GRASPS_SERVICE_DEFAULT_NAME,
									 cloud_indexed)

	#
	# interpret pose configuration
	#
	# We use the pose of the gripper tool 0 link for planning.
	target_poses = configure_grasp_poses(grasp_configs)

	#
	# planning
	#
	pick_and_place(PICK_AND_PLACE_SERVICE_DEFAULT_NAME, target_poses)

	rospy.loginfo("[GraspingPipeline] Waiting while RVIZ displays motion planning...")
	rospy.sleep(3.0)


if __name__ == "__main__":
	main()
