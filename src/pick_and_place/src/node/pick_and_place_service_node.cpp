//---------- pick_and_place
#include "pick_and_place/pick_and_place_server.h"
//----------

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pick_and_place_service_node");

    ros::NodeHandle node("~");
    
    pick_and_place::PickAndPlaceServer pick_and_place_server(node);
    pick_and_place_server.launch();

    ros::waitForShutdown();
    return 0;
}
