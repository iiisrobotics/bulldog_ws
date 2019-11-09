#include <ros/ros.h>
#include <bulldog_driver/bulldog_hw.h>

namespace bulldog
{

  BulldogHW::BulldogHW(std::string controller_port, std::string display_port, std::string bms_port, double diagnostic_period)
  {
    using namespace hardware_interface;

    motor_controller = new MotorController(controller_port);
    motor_controller->open();

    uart_display = new UARTDisplay(display_port);
    uart_display->connect();
    uart_display->writeDisplay("SPG(3);\r\n");
    ros::Duration(1).sleep();

    bms = new BMS(bms_port);
    bms->connect();

    left_encoder_speed = 0;
    right_encoder_speed = 0;

    left_encoder_counts = 0;
    right_encoder_counts = 0;

    diagnostic_dt = diagnostic_period;
    diagnostic_current_time = ros::Time::now();
    diagnostic_last_time = ros::Time::now();

    last_emergency_status = 0;
    emergency_stop = false;
    battery_capacity = 0;

    joint_name_.resize(4);
    joint_position_.resize(4);
    joint_velocity_.resize(4);
    joint_effort_.resize(4);
    joint_velocity_command_.resize(4);

    joint_name_[0] = "front_left_wheel_joint";
    joint_name_[1] = "rear_left_wheel_joint";
    joint_name_[2] = "front_right_wheel_joint";
    joint_name_[3] = "rear_right_wheel_joint";

    for (unsigned int i = 0; i < joint_name_.size(); i++)
    {
      js_interface_.registerHandle(JointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]));
      vj_interface_.registerHandle(JointHandle(js_interface_.getHandle(joint_name_[i]), &joint_velocity_command_[i]));
    }

    registerInterface(&js_interface_);
    registerInterface(&vj_interface_);

    updater.setHardwareID("Bulldog");
    updater.add(robot_status_task);
    updater.add(battery_status_task);
    updater.add(motor_status_task);
    updater.add(controller_status_task);
  }

  void BulldogHW::read()
  {
    left_encoder_speed = motor_controller->getEncoderSpeed(1);
    right_encoder_speed = motor_controller->getEncoderSpeed(2);

    if (left_encoder_speed != -1 && right_encoder_speed != -1)
    {
      joint_velocity_[0] = left_encoder_speed * SP;
      joint_velocity_[1] = joint_velocity_[0];
      joint_velocity_[2] = -right_encoder_speed * SP;
      joint_velocity_[3] = joint_velocity_[2];
    }

    left_encoder_counts = motor_controller->getEncoderCount(1);
    right_encoder_counts = motor_controller->getEncoderCount(2);
    if (left_encoder_counts != -1 && right_encoder_counts != -1)
    {
      joint_position_[0] = left_encoder_counts * PP;
      joint_position_[1] = joint_position_[0];
      joint_position_[2] = -right_encoder_counts * PP;
      joint_position_[3] = joint_position_[2];
    }

    diagnostic_current_time = ros::Time::now();
    if ((diagnostic_current_time - diagnostic_last_time).toSec() > diagnostic_dt)
    {
      //int emergency_status = motor_controller->getDigitalInput(3);
      int emergency_status = 0;
      if (emergency_status)  
      {
        uart_display->writeDisplay("DS24(290,295,'brake     ',32,0);");
        emergency_stop = true;
      }   
      else
      {
        uart_display->writeDisplay("DS24(290,295,'normal     ',32,0);");
      }
 
      if ((last_emergency_status) && (!emergency_status))
      {
        emergency_stop = false;
      }
      last_emergency_status = emergency_status;

      float speed = joint_velocity_[0] * 0.1816;
      float odom = joint_position_[0] * 0.1816;

      float left_motor_current = motor_controller->getMotorCurrent(1);
      float right_motor_current = motor_controller->getMotorCurrent(2);
      float battery_voltage = motor_controller->getBatteryVoltage();
      float controller_temp = motor_controller->getControllerTemp();

      battery_capacity = (battery_voltage - 30.0) / 0.2475;
      // ros::Duration bms_period(0.1);
      // if ((diagnostic_current_time - bms_last_time).toSec() > bms_period.toSec())
      // {
      //   float soc = bms->getSOC();
      //   if (soc > 0)
      //   {
      //     soc = (soc > 100) ? 100 : soc;
      //     soc = (soc < 0) ? 0 : soc;
      //     battery_capacity = soc;
      //   }
      //   bms_last_time = diagnostic_current_time;
      // }
      
      robot_status_task.update(emergency_stop);
      battery_status_task.update(battery_voltage, battery_capacity);
      motor_status_task.update(left_motor_current, right_motor_current, left_encoder_counts, right_encoder_counts);
      controller_status_task.update(controller_temp);
      updater.force_update();

      stringstream speed_str;
      stringstream odom_str;
      stringstream battery_capacity_str;

      stringstream left_motor_current_str;
      stringstream right_motor_current_str;
      stringstream battery_voltage_str;
      stringstream controller_temp_str;

      speed_str << speed;
      odom_str << odom;
      battery_capacity_str << battery_capacity;

      left_motor_current_str << left_motor_current;
      right_motor_current_str << right_motor_current;
      controller_temp_str << controller_temp;
      battery_voltage_str << battery_voltage;

      uart_display->writeDisplay("DS24(290,160,'" + battery_voltage_str.str() + "     " + "',32,0);");
      uart_display->writeDisplay("DS24(290,230,'" + battery_capacity_str.str() + "%     ',32,0);");       
      uart_display->writeDisplay("DS24(290,350,'" + speed_str.str() + "        " + "',32,0);");
      uart_display->writeDisplay("DS24(700,160,'" + left_motor_current_str.str() + "    " + "',32,0);");
      uart_display->writeDisplay("DS24(700,230,'" + right_motor_current_str.str() + "    " + "',32,0);");
      uart_display->writeDisplay("DS24(700,295,'" + controller_temp_str.str() + "     " + "',32,0);");
      uart_display->writeDisplay("DS24(700,350,'" + odom_str.str() + "  " + "',32,0);");

      uart_display->writeDisplay("\r\n");

      diagnostic_last_time = diagnostic_current_time;
    }
  }

  void BulldogHW::write()
  {
    if (emergency_stop)
    {
      motor_controller->setMotorSpeed(1, 0);
      motor_controller->setMotorSpeed(2, 0);
    }
    else
    {
      motor_controller->setMotorSpeed(1, joint_velocity_command_[0] * CP);
      motor_controller->setMotorSpeed(2, -joint_velocity_command_[2] * CP);
    }
  }

}
