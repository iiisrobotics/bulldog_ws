#include <ros/ros.h>
#include <bulldog_driver/motor_controller.h>

MotorController::MotorController(std::string dev)
{
  controller = new RoboteqDevice(); 
  port.assign(dev); 
}

MotorController::~MotorController() 
{
  if (controller != NULL)
    delete controller;
}

int MotorController::open()
{
  while ((this->controller->Connect(port) != RQ_SUCCESS) && ros::ok())
  {
    ROS_DEBUG("MotorController::open: Error opening controller port");
    usleep(2000000);
  }
  ROS_DEBUG("MotorController::open: device opened successfully");
  return 0;
}

int MotorController::close()
{
  if (controller != NULL) 
  {
    controller->Disconnect();
    ROS_DEBUG("MotorController::close: controller disconnected");
    return -1;
  }
  return 0;
}

float MotorController::getBatteryVoltage()
{
  int status = 0;
  int volts = 0;

  //vdr:vmot:v5out--1:2:3
  status = controller->GetValue(_VOLTS, 2, volts);
  if(status != RQ_SUCCESS)
  {
    ROS_DEBUG("MotorController::getBatteryVoltage: error = %d", status);
    return -1;
  }
  else
  {
    return (float)volts/10.0;
  }
}

float MotorController::getControllerTemp()
{
  int status = 0;
  int temp = 0;

  //tm:t1:t2
  status = controller->GetValue(_TEMP, 1, temp);
  if(status != RQ_SUCCESS)
  {
    ROS_DEBUG("MotorController::getControllerTemp: error = %d", status);
    return -1;
  }
  else
  {
    return temp;
  } 
}

int MotorController::getDigitalInput(int number)
{
  int input = 0;
  int status = 0;

  if((number <= 0) || (number > DIGITAL_INPUTS))
  {
    ROS_DEBUG("MotorController::getDigitalInput: input %d out of range", number);
    return -1;
  }

  status = controller->GetValue(_DIN, number, input);
  if(status != RQ_SUCCESS)
  {
    ROS_DEBUG("MotorController::getDigitalInput: error = %d", status);
    return -1;
  }
  else
  {
    ROS_DEBUG("MotorController:getDigitalInput: Input[%d] = %d", number, input);
    return input;
  } 
}

int MotorController::getEncoderCount(int channel)
{
  int status = 0;
  int enc = 0;

  status = controller->GetValue(_ABCNTR, channel, enc);
  if(status != RQ_SUCCESS)
  {
    ROS_DEBUG("MotorController::getEncoderCount: error = %d", status);
    return -1;
  }
  else
  {
    return enc;
  } 
}

int MotorController::getEncoderSpeed(int channel)
{
  int status = 0;
  int speed=0;

  status = controller->GetValue(_ABSPEED, channel, speed);
  if(status != RQ_SUCCESS)
  {
    ROS_DEBUG("MotorController::getEncoderSpeed: error = %d", status);
    return -1;
  }
  else
  {
    return speed;
  } 
}

float MotorController::getMotorCurrent(int channel)
{
  int status = 0;
  int current = 0;

  status = controller->GetValue(_MOTAMPS, channel, current);
  if(status != RQ_SUCCESS)
  {
    ROS_DEBUG("MotorController::getMotorCurrent: error = %d", status);
    return -1;
  }
  else
  {
    return (float)current/10.0;
  } 
}

int MotorController::setMotorSpeed(int channel, int speed)
{
  int status = 0;

  status = controller->SetCommand(_GO, channel, speed);
  if(status != RQ_SUCCESS)
  {
    ROS_DEBUG("MotorController::setMotorSpeed: error = %d", status);
    return -1;
  }
  else
  {   
    return 0;
  }
}
