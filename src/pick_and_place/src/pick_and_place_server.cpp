//---------- pick_and_place
#include "pick_and_place/pick_and_place_server.h"
//----------

namespace pick_and_place
{

PickAndPlaceServer::PickAndPlaceServer(ros::NodeHandle node) :
    node_(node),
    pipeline_(node),
    async_spinner_(2)
{
    //
    // load names from ros parameter server
    //
    node_.getParam("service_topic", service_topic_);

    //
    // setup server
    //
    server_ = node_.advertiseService(service_topic_, 
        &PickAndPlaceServer::serviceCallback, this);
    service_topic_ = server_.getService();
    ROS_DEBUG_STREAM("Pick and place service topic: " << service_topic_);
}

PickAndPlaceServer::~PickAndPlaceServer()
{
    async_spinner_.stop();
}

void PickAndPlaceServer::launch()
{
    async_spinner_.start();
}

bool PickAndPlaceServer::serviceCallback(
    PickAndPlaceRequest &req, PickAndPlaceResponse& res)
{
    ROS_INFO_STREAM("Pick and place request received");

    bool success = pipeline_.run(req.grasp_poses);
    res.success = success;

    return true;
}

} // namespace pick_and_place

