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
    status_topic_(command_topic),
    timeout_(timeout)
{
    command_pub_ = node_.advertise<RobotOutput>(command_topic, 1);
    if (command_pub_.getNumSubscribers() == 0) {
        ROS_WARN_STREAM(
            "No robotiq 3 finger gripper command subscribers found! \
            Command may lost");
    }
    else {
        ROS_DEBUG_STREAM(
            "Robotiq 3 finger gripper command topic: " << command_topic_);
    }

    status_sub_ = node_.subscribe(
        status_topic, 1, &Robotiq3FGripperCommander::statusCallback, this);
    if (status_sub_.getNumPublishers() == 0) {
        ROS_WARN_STREAM("No robtiq 3 finger gripper status publishers found! \
            Gripper may not startup");
    }
    else {
        ROS_DEBUG_STREAM(
            "Robotiq 3 finger gripper status topic: " << status_topic_);
    }
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
    deactivate();
}

bool Robotiq3FGripperCommander::activate()
{
    command_ = RobotOutput();
    command_.rACT = 1;
    command_.rGTO = 1;
    command_.rSPA = 255;
    command_.rFRA = 150;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT == 1 &&
            status_.gMOD == 1) {
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::deactivate()
{
    command_ = RobotOutput();
    command_.rACT = 0;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT == 0) {
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::open()
{
    command_.rPRA = 0;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated! \
                Please activate it first");
            break;
        }
        else if (status_.gPRA == 0) {
            success = true;
            break;
        }
    }

    return success;
}

bool Robotiq3FGripperCommander::close()
{
    command_.rPRA = 255;
    sendCommand();

    bool success = false;
    ros::Time send_time = ros::Time::now();
    while ((ros::Time::now() - send_time).toSec() < timeout_) {
        if (status_.gACT = 0) {
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated! \
                Please activate it first");
            break;
        }
        else if (status_.gPRA == 255) {
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
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated! \
                Please activate it first");
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
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated! \
                Please activate it first");
            break;
        }
        else if (status_.gPRA == position) {
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
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated! \
                Please activate it first");
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
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated! \
                Please activate it first");
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
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated! \
                Please activate it first");
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
            ROS_WARN_STREAM("Robotiq 3 finger gripper is deactivated! \
                Please activate it first");
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
    status_ = status;
}

} // namespace robotiq_3f_gripper_commander

