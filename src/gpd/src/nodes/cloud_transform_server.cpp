//server node to transform pointcloud from focal frame to base link frame
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>

#include <gpd/cloud_transform.h>

class CloudTransformServer
{
private:
	tf::TransformListener listener_;

public:
	CloudTransformServer()
	{
	}

	bool transformCloud(gpd::cloud_transform::Request &req,
		gpd::cloud_transform::Response &res)
	{
		ROS_INFO("Receive point cloud msg!");
	
		//listener.waitForTransform("base_link", req.in_cloud.header.frame_id, ros::Time(0), ros::Duration(2.0));
		//printf("get!");
		bool error_flag = true;
		while(error_flag){
			error_flag = false;
			try{
				listener_.waitForTransform(
					"base_link", 
					req.cloud_to_transform.header.frame_id, 
					req.cloud_to_transform.header.stamp, 
					ros::Duration(10.0)
				);
				pcl_ros::transformPointCloud(
					"base_link", 
					req.cloud_to_transform, 
					res.cloud_transformed, 
					listener_
				);
			}
			catch(tf::TransformException &ex){
				error_flag = true;
				ROS_ERROR("%s", ex.what());
				ros::Duration(0.5).sleep();
				continue;
			}
		}

		return true;
	}
};


int main(int argc, char **argv){
	ros::init(argc, argv, "cloud_transform");
	
	ros::NodeHandle node("~");
	
	CloudTransformServer cloud_transform_server;
	ros::ServiceServer service = node.advertiseService(
        "transformation", 
		&CloudTransformServer::transformCloud, 
		&cloud_transform_server
	);
	ROS_INFO("Ready to transform point cloud...");
	
	ros::spin();
	
	return 0;
}
