//---------- robotiq_3f_gripper_commander
#include "robotiq_3f_gripper_commander.h"
//----------

namespace robotiq_3f_gripper_commander
{

Robotiq3FGripperCommander::Robotiq3FGripperCommander(
    ros::NodeHandle node,
    std::string command_topic,
    std::string status_topic,
    const double timeout) :
    node_(node),
    command_topic_(command_topic),
    status_topic_(status_topic),
    timeout_(timeout)
{
    command_pub_ = node_.advertise<RobotOutput>(command_topic_, 1);
    ros::Duration(0.5).sleep();// wait for advertising completion
    if (command_pub_.getNumSubscribers() == 0) {
        ROS_WARN_STREAM(
            "No robotiq 3 finger gripper command subscribers found! "
            << "Command may lost");
    }
    command_topic_ = command_pub_.getTopic();
    ROS_DEBUG_STREAM(
        "Robotiq 3 finger gripper command topic: " << command_topic_);


    status_sub_ = node_.subscribe(
        status_topic_, 1, &Robotiq3FGripperCommander::statusCallback, this);
    ros::Duration(0.5).sleep();// wait for subscribing completion    
    if (status_sub_.getNumPublishers() == 0) {
        ROS_WARN_STREAM(
            "No robotiq 3 finger gripper status publishers found! "
            << "Gripper may not startup correctly");
    }
    status_topic_ = status_sub_.getTopic();
    ROS_DEBUG_STREAM(
        "Robotiq 3 finger gripper status topic: " << status_topic_);
}

Robotiq3FGripperCommander::Robotiq3FGripperCommander(
    const Robotiq3FGripperCommander& commander)
{
    Robotiq3FGripperCommander(
        commander.node_, 
        commander.command_topic_,
        commander.status_topic_,
        commander.timeout_);
}

Robotiq3FGripperCommander& Robotiq3FGripperCommander::operator=(
    const Robotiq3FGripperCommander& commander)
{
    Robotiq3FGripperCommander(
        commander.node_,
        commander.command_topic_,
        commander.status_topic_,
        commander.timeout_);
}

Robotiq3FGripperCommander::~Robotiq3FGripperCommander()
{
}

bool Robotiq3FGripperCommander::activate()
{
    ROS_DEBUG_STREAM("Activating robotiq 3 finger gripper...");

    command_ = RobotOutput();
    command_.rACT = 1;
    command_.rGTO = 1;
    command_.rSPA = 255;
    command_.rFRA = 150;
    sendCommand();

    const double ROBOTIQ_3F_GRIPPER_ACTIVATION_DEFAULT_TIMEOUT = 20.0;
    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() <
        ROBOTIQ_3F_GRIPPER_ACTIVATION_DEFAULT_TIMEOUT) {
        if (status_.gACT == 1 &&    // gripper activation
            status_.gGTO == 1) {    // go to position request
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::deactivate()
{
    ROS_DEBUG_STREAM("Deactivating robotiq 3 finger gripper...");

    command_ = RobotOutput();
    command_.rACT = 0;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT == 0 &&        // gripper activation
            status_.gGTO == 0) {        // go to position request
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::open()
{
    ROS_DEBUG_STREAM("Opening robotiq 3 finger gripper...");

    command_.rPRA = 0;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated!"
                << "Please activate it first");
            break;
        }
        else if (status_.gPRA == 0 &&       // echo of the requested position of finger A
                 status_.gSTA == 3) {       // all fingers reached requested position
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::close()
{
    ROS_DEBUG_STREAM("Closing robotiq 3 finger gripper...");

    command_.rPRA = 255;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated!"
                << "Please activate it first");
            break;
        }
        else if (status_.gPRA == 255 &&     // echo of the requested position of finger A
                 (status_.gSTA == 1 ||      // one or two fingers stopped before requested position
                  status_.gSTA == 2 ||      // all fingers stopped before requested position
                  status_.gSTA == 3)) {     // all fingers reached requested position
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::setMode(Robotiq3FGripperModes mode)
{
    command_.rMOD = mode;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated!"
                << "Please activate it first");
            break;
        }
        else if (status_.gMOD == mode) {
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::setPosition(uint8_t position)
{
    command_.rPRA = position;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated!"
                << "Please activate it first");
            break;
        }
        else if (status_.gPRA == position &&    // echo of the requested position of finger A
                 status_.gPOA == position) {    // the actual position of finger A
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::increaseSpeed()
{
    command_.rSPA += 25;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated!"
                << "Please activate it first");
            break;
        }
        else if (status_.gFLT == 0) {
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::decreaseSpeed()
{
    command_.rSPA -= 25;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated!"
                << "Please activate it first");
            break;
        }
        else if (status_.gFLT == 0) {
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::increaseForce()
{
    command_.rFRA += 25;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated!"
                << "Please activate it first");
            break;
        }
        else if (status_.gFLT == 0) {
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::decreaseForce()
{
    command_.rFRA -= 25;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated!"
                << "Please activate it first");
            break;
        }
        else if (status_.gFLT == 0) {
            success = true;
            break;
        }
    }

    return success;
}

void Robotiq3FGripperCommander::sendCommand()
{
    command_pub_.publish(command_);
}

void Robotiq3FGripperCommander::statusCallback(const RobotInput& status)
{
    // ROS_DEBUG_STREAM("Updating robotiq 3 finger gripper status...");
    status_ = status;
}

} // namespace robotiq_3f_gripper_commander

