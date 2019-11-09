#ifndef __BULLDOGHW_H_
#define __BULLDOGHW_H_

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>

#include <bulldog_driver/bulldog_diagnostics.h>

#include <bulldog_driver/motor_controller.h>
#include <bulldog_driver/uart_display.h>
#include <bulldog_driver/bms.h>

// Beagle
#define SP 0.00197274263962
#define PP 0.0000591822791885
#define CP 112.646331944

// Bulldog
// #define SP 0.00327249234749
// sp和pp类似，sp是电机输出轴速度的（就是车轮转速）
// #define PP 0.000245436926062
// pp是编码器脉冲数转为电机输出轴转角（弧度）的系数，比如减速比10，电机一圈100个脉冲，当读到脉冲个数为1000时，说明电机转了10圈，电机轴转了1圈，就是轮子转了6.28弧度
// #define CP 101.859163579
// cp正好相反，把计算的轮子角速度，转变到电机转速，电机转速单位是rpm，转每分钟
namespace bulldog
{

  class BulldogHW : public hardware_interface::RobotHW
  {
    public:
      BulldogHW(std::string controller_port, std::string display_port, std::string bms_port, double diagnostic_period);
      void read();
      void write();

    public:
      MotorController *motor_controller;
      UARTDisplay *uart_display;
      BMS *bms;

    protected:

    private:
      hardware_interface::JointStateInterface    js_interface_;
      hardware_interface::VelocityJointInterface vj_interface_;

      std::vector<double> joint_velocity_command_;
  
      std::vector<std::string> joint_name_;
      std::vector<double> joint_position_;
      std::vector<double> joint_velocity_;
      std::vector<double> joint_effort_;

    private:
      bulldog::RobotStatusTask robot_status_task;
      bulldog::BatteryStatusTask battery_status_task;
      bulldog::MotorStatusTask motor_status_task;
      bulldog::ControllerStatusTask controller_status_task;
      diagnostic_updater::Updater updater;

      int left_encoder_speed;
      int right_encoder_speed;

      int64_t left_encoder_counts;
      int64_t right_encoder_counts;

      double diagnostic_dt;
      ros::Time diagnostic_current_time, diagnostic_last_time, bms_last_time;
      int last_emergency_status;
      bool emergency_stop;
      int battery_capacity;
  };

}

#endif
