#ifndef PICK_AND_PLACE_SERVER_H
#define PICK_AND_PLACE_SERVER_H

//---------- pick_and_place_pipeline
#include "pick_and_place/pick_and_place_pipeline.h"
//----------

//---------- pick_and_place
#include <pick_and_place/PickAndPlace.h>
//----------

using namespace pick_and_place_pipeline;

namespace pick_and_place
{

class PickAndPlaceServer final
{

private:
    /**
     *  @attrib node_: ros node
     */
    ros::NodeHandle node_;

    /**
     *  @attrib async_spinner_: asynchronous spinner
     */
    ros::AsyncSpinner async_spinner_;
    
    /**
     *  @attrib grasp_poses: all the possible grasp poses
     */
    std::vector<geometry_msgs::PoseStamped> grasp_poses_;

    /**
     *  @attrib service_topic_: pick and place topic 
     */
    std::string service_topic_;

    /**
     *  @attrib server_: pick and place ros server
     */
    ros::ServiceServer server_;

    /**
     *  @attrib pipeline_: pick and place pipeline
     */
    PickAndPlacePipeline pipeline_;

protected:
public:

private:
    /**
     *  @brief  serviceCallback: pick and place service callback function
     *  @param  req: pick and place service request
     *  @param  res: pick and place service response
     *  @return success: true on success
     */
    bool serviceCallback(PickAndPlaceRequest &req, PickAndPlaceResponse& res);

protected:
public:
    /**
     *  @brief  PickAndPlaceServer: default constructor
     *  @param  node: ros node of this server
     */
    PickAndPlaceServer(ros::NodeHandle node = ros::NodeHandle());

    /**
     *  @brief  PickAndPlaceServer: copy constructor (deprecated)
     *  @param  server: a <PickAndPlaceServer> instantiation to copy
     */
    PickAndPlaceServer(const PickAndPlace& server) = delete;

    /**
     *  @brief  ~PickAndPlaceServer: destructor
     */
    ~PickAndPlaceServer();

    /**
     *  @brief  operator=: assignment operator (deprecated)
     *  @param  server: a <PickAndPlaceServer> instantiation to copy
     *  @return *this: this instantiation
     */
    PickAndPlaceServer &operator=(const PickAndPlaceServer& server) = delete;

    /**
     *  @brief  launch: launch the pick and place server
     */
    void launch();

};

} // namespace pick_and_place


#endif // PICK_AND_PLACE_SERVER_H
