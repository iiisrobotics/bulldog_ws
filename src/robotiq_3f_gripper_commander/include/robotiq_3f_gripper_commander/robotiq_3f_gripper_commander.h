#ifndef ROBOTIQ_3F_GRIPPER_COMMANDER_H
#define ROBOTIQ_3F_GRIPPER_COMMANDER_H

//---------- ros
#include <ros/ros.h>
//----------

//---------- robotiq 3 finger control
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput.h>
#include <robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotInput.h>
//----------

namespace robotiq_3f_gripper_commander
{

static const std::string ROBOTIQ_3F_GRIPPER_DEFAULT_COMMAND_TOPIC = 
    "Robotiq3FGripperRobotOutput";
static const std::string ROBOTIQ_3F_GRIPPER_DEFAULT_STATUS_TOPIC = 
    "Robotiq3FGripperRobotInput";
static const double ROBOTIQ_3F_GRIPPER_DEFAULT_TIMEOUT = 3.0;

typedef robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotInput 
    RobotInput;
typedef robotiq_3f_gripper_articulated_msgs::Robotiq3FGripperRobotOutput
    RobotOutput;

enum Robotiq3FGripperModes {
    BASIC = 0,
    PINCH = 1,
    WIDE = 2,
    SCISSOR = 3
};

class Robotiq3FGripperCommander final
{

private:
    /**
     *  @attrib node_: ros node
     */
    ros::NodeHandle node_;

    /**
     *  @attrib command_topic_: topic to send the gripper control
     *                          command
     */
    std::string command_topic_;

    /**
     *  @attib status_topic_: topic to receive the gripper status
     */
    std::string status_topic_;

    /**
     *  @attrib timeout_: timeout limit of synchronous waits. (seconds)
     */
    double timeout_;

    /**
     *  @attrib command_pub_: gripper command publisher, i.e. gripper 
     *                        controller
     */
    ros::Publisher command_pub_;

    /**
     *  @attrib status_sub_: gripper status subscriber, i.e. gripper monitor
     */
    ros::Subscriber status_sub_;

    /**
     *  @attrib command_: gripper control command
     */
    RobotOutput command_;
    
    /**
     *  @attrib status_: gripper status
     */
    RobotInput status_;

protected:
public:

private:
    /**
     *  @brief  sendCommand: send the current command to the actual gripper 
     *                       controller
     */
    void sendCommand();

    /**
     *  @brief  statusCallback: callback function of the gripper status
     *                                 monitor
     *  @param  status: status of the gripper
     */
    void statusCallback(const RobotInput& status);

    /**
     *  @brief  isAllElementEqual: check all elements in an array are
     *                             equivalent
     *  @param  array: the array
     *  @param  num_element: number of element in the array
     *  @return success: true on success
     */
    bool isAllElementEqual(uint8_t array[], const size_t num_element);

protected:
public:
    /**
     *  @brief  Robotiq3FGripperCommander: custom topic constructor
     *  @param  command_topic: topic to send the gripper control
     *                         command
     *  @param  status_topic: topic to receive the gripper status
     */
    Robotiq3FGripperCommander(
        ros::NodeHandle node, 
        std::string command_topic = ROBOTIQ_3F_GRIPPER_DEFAULT_COMMAND_TOPIC, 
        std::string status_topic = ROBOTIQ_3F_GRIPPER_DEFAULT_STATUS_TOPIC,
        double timeout = ROBOTIQ_3F_GRIPPER_DEFAULT_TIMEOUT
    );

    /**
     *  @brief  Robotiq3FGripperCommander: copy constructor
     *  @param  pick_and_place_pipeline: a <Robotiq3FGripperCommander>
     *                                   instantiation to copy
     */
    Robotiq3FGripperCommander(const Robotiq3FGripperCommander& commander);
    
    /**
     *  @brief  ~Robotiq3FGripperCommander: destructor
     */
    ~Robotiq3FGripperCommander();

    /**
     *  @brief  Robotiq3FGripperCommander: copy constructor
     *  @param  pick_and_place_pipeline: a <Robotiq3FGripperCommander>
     *                                   instantiation to copy
     *  @return *this: this instantiation
     */
    Robotiq3FGripperCommander& operator=(
        const Robotiq3FGripperCommander& commander);

    /**
     *  @brief  activate: activate the gripper while waiting for completion
     *  @return success: true on success
     */
    bool activate();

    /**
     *  @brief  deactivate: deactivate the gripper while waiting for completion
     *  @return success: true on success
     */
    bool deactivate();

    /**
     *  @brief  open: open the gripper while waiting for completion
     *  @return success: true on success
     */
    bool open();

    /**
     *  @brief  close: close the gripper while waiting for completion
     *  @return success: true on success
     */
    bool close();

    /**
     *  @brief  setMode: switch between BASIC, WIDE, PINCH, SCISSOR mode.
     *  @param  mode: mode to select
     *  @return success: true on success
     */
    bool setMode(Robotiq3FGripperModes mode);

    /**
     *  @brief  setPosition: set position of the gripper (0 - 255)
     *  @param  position: the custom position
     *  @return success: true on success
     */
    bool setPosition(uint8_t position);

    /**
     *  @brief  increaseSpeed: speed up the gripper (+25 between 0 - 255)
     *  @return success: true on success
     */
    bool increaseSpeed();

    /**
     *  @brief  decreaseSpeed: slow down the gripper (-25 between 0 - 255)
     *  @return success: true on success
     */
    bool decreaseSpeed();

    /**
     *  @brief  increaseForce: increase the grasp force (+25 between 0 - 255)
     */
    bool increaseForce();

    /**
     *  @brief  decreaseForce: decrease the grasp force (-25 between 0 - 255)
     */
    bool decreaseForce();

};

// Create boost pointers for this class
typedef boost::shared_ptr<Robotiq3FGripperCommander> 
    Robotiq3FGripperCommanderPtr;
typedef boost::shared_ptr<const Robotiq3FGripperCommander> 
    Robotiq3FGripperCommanderConstPtr;

} // namespace robotiq_3f_gripper_commander

#endif // ROBOTIQ_3F_GRIPPER_COMMANDER_H
