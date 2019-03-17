#include <ros/ros.h>
#include <roscco/BrakeCommand.h>
#include <roscco/EnableDisable.h>
#include <roscco/SteeringCommand.h>
#include <roscco/ThrottleCommand.h>

double calc_exponential_average(double AVERAGE, double SETPOINT, double FACTOR);
double linear_tranformation(double VALUE, double HIGH_1, double LOW_1, double HIGH_2, double LOW_2);

class RosccoReport
{
public:
  RosccoReport();
  void EnableControl(bool enable_control);
  void SteerTest();
  // Number of messages to retain when the message queue is full
  const int QUEUE_SIZE_ = 10;

private:
  ros::NodeHandle nh_;

  ros::Publisher throttle_pub_;
  ros::Publisher brake_pub_;
  ros::Publisher steering_pub_;
  ros::Publisher enable_disable_pub_;
};

/**
 * @brief RosccoReport
 * class initializer
 *
 * This function constructs a class which subscribes to ROS Joystick messages, converts the inputs to ROSCCO relevant
 * values and publishes ROSCCO messages on a 20 Hz cadence.
 */
RosccoReport::RosccoReport()
{
  brake_pub_ = nh_.advertise<roscco::BrakeCommand>("brake_command", QUEUE_SIZE_);
  throttle_pub_ = nh_.advertise<roscco::ThrottleCommand>("throttle_command", QUEUE_SIZE_);
  steering_pub_ = nh_.advertise<roscco::SteeringCommand>("steering_command", QUEUE_SIZE_);
  enable_disable_pub_ = nh_.advertise<roscco::EnableDisable>("enable_disable", QUEUE_SIZE_);
}

/**
 * @brief Report enable control function
 *
 * @param bool to enable or disable control.
 */
void RosccoReport::EnableControl(bool enable_control)
{
  // gamepad triggers default 0 prior to using them which is 50% for the logitech and xbox controller the initilization
  // is to ensure the triggers have been pulled prior to enabling OSCC command
  ROS_INFO("In Enable Control %i", enable_control);
  roscco::EnableDisable enable_msg;
  enable_msg.header.stamp = ros::Time::now();
  enable_msg.enable_control = true;
  enable_disable_pub_.publish(enable_msg);
  ROS_INFO("Published Msg");
}

void RosccoReport::SteerTest()
{
    roscco::SteeringCommand steering_msg;

    steering_msg.header.stamp = ros::Time::now();
    steering_msg.steering_torque = 0.2;
    steering_pub_.publish(steering_msg);

    steering_msg.header.stamp = ros::Time::now();
    steering_msg.steering_torque = 0.3;
    steering_pub_.publish(steering_msg);

    steering_msg.header.stamp = ros::Time::now();
    steering_msg.steering_torque = 0.4;
    steering_pub_.publish(steering_msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "roscco_report");
  RosccoReport roscco_report;
  ros::Rate loop_rate_(50);
  int count = 0;
  while(ros::ok())
  {
    roscco_report.EnableControl(true);
    roscco_report.SteerTest();
    ros::spinOnce();
    loop_rate_.sleep();
    count++;
  }
}
