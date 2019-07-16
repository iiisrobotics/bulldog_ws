#include <nodes/cloud_transform_server.h>

int main(int argc, char **argv){
	ros::init(argc, argv, "cloud_transform_server");
	
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
