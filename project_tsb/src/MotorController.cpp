#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "project_tsb_msgs/msg/control_forces.hpp"
#include "project_tsb_msgs/msg/motor_currents.hpp"


// ROS2 Node class
class MotorController : public rclcpp::Node
{
  public:
    MotorController()
    : Node("Motor_Controller")
    {
      // Setup publishers and subscribers
      publisher_ = this->create_publisher<project_tsb_msgs::msg::MotorCurrents>("boat_input", 10); //Publisher used to publish the odometry
      subscriber_ = this->create_subscription<project_tsb_msgs::msg::ControlForces>("pid_output", 10, std::bind(&MotorController::convert, this, std::placeholders::_1));

      RCLCPP_INFO(this->get_logger(), "Forces to Currents conversion node started");
    }

  private:
    void convert(const project_tsb_msgs::msg::ControlForces::SharedPtr msg)
    {
      // Extract current forces from msg
      double force_u = msg->force_u;
      double force_r = msg->force_r;

      // Calculate force for each motor
      double force_p = (force_u*d_ - force_r) / (d2_);
      double force_s = (force_u*d_ + force_r) / (d2_);

      // Calculate current
      double current_p = p0_ + p1_*abs(force_p) + p2_*force_p*force_p;
      if (force_p < 0.0) current_p = -current_p;

      double current_s = p0_ + p1_*abs(force_s) + p2_*force_s*force_s;
      if (force_s < 0.0) current_s = -current_s;
      
      // Publish currents
      project_tsb_msgs::msg::MotorCurrents output;
      output.header.stamp = this->now();
      output.current_p = current_p;
      output.current_s = current_s;

      publisher_->publish(output);
      RCLCPP_INFO(this->get_logger(), "Received forces: Force_u = %f, Force_r = %f. Calculated forces: Force_p = %f, Force_s = %f. Published currents: Current_p = %f, Current_s = %f", force_u, force_r, force_p, force_s, current_p, current_s);
    }

    rclcpp::Publisher<project_tsb_msgs::msg::MotorCurrents>::SharedPtr publisher_;
    rclcpp::Subscription<project_tsb_msgs::msg::ControlForces>::SharedPtr subscriber_;

    // Distance between motors
    const double d_ = 0.45;
    const double d2_ = 0.9;
    // Constants for Force->Current Conversion
    double const p0_ = -0.20863;
    double const p1_ = 0.17324;
    double const p2_ = 0.00649;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorController>());
  rclcpp::shutdown();
  return 0;
}
