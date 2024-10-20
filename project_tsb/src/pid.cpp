#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

using namespace std::chrono_literals;

// ROS2 Node class
class PIDController : public rclcpp::Node
{
  public:
    PIDController()
    : Node("PID_Controller")
    {

      // References
      this->ref_u = 0.0;
      this->ref_psi = 0.0;

      // Errors
      this -> e_u = 0.0;
      this -> e_psi = 0.0;

      // Integrator(sum) and Derivator(last value)
      // TODO: criar classe para dar handle ao integrador/derivador?
      this->last_time = this->now();
      this -> e_u_sum = 0.0;
      this -> e_psi_sum = 0.0;
      this -> e_u_prev = 0.0;
      this -> e_psi_prev = 0.0;

      // PID Outputs
      this->tau_v = 0;
      this->tau_r = 0;

      // PID Gains
      this->kp_u = 0.0;
      this->ki_u = 0.0;
      this->kd_u = 0.0;
      this->kp_psi = 0.0;
      this->ki_psi = 0.0;
      this->kd_psi = 0.0;

      // Constants
      this->m_u = 50;
      this->m_v = 60;
      this->m_r = 4.64;
      this->m_u_v = this->m_u - this->m_v;
      this->d_u = 0.2;
      this->d_v = 55.1;
      this->d_r = 0.14;
      this->d_u_u = 25;
      this->d_v_v = 0.01;
      this->d_r_r = 6.23;

      // Setup publishers and subscribers
      publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic2", 10); //Publisher used to publish the odometry
      subscriber_ref_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("topic1", 10, std::bind(&PIDController::update_ref, this, std::placeholders::_1));
      subscriber_state_ = this->create_subscription<nav_msgs::msg::Odometry>("topic3", 10, std::bind(&PIDController::update_state, this, std::placeholders::_1));
    }

  private:
    void update_ref(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      this->ref_u = msg->data[0];
      this->ref_psi = msg->data[1];

      //TODO: maybe update PID right away


      RCLCPP_INFO(this->get_logger(), "Received new reference"); //TODO: log reference values also

    }
    void update_state(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
      // Calculate next PID output and publish it

      tf2::Quaternion q;
      tf2::fromMsg(msg->pose.pose.orientation, q);
      tf2::Matrix3x3 m(q); //https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);


      // Update error
      this->e_u = this->ref_u - msg->twist.twist.linear.x;
      this->e_psi = this->ref_psi - yaw;

      // Update Integrator and Derivative term

      // Calculate PID output

      // Publish output


      RCLCPP_INFO(this->get_logger(), "Current state: u = %f, psi = %f", msg->twist.twist.linear.x, msg->twist.twist.angular.z);

      RCLCPP_INFO(this->get_logger(), "Published new output"); //TODO: log output values also

    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_ref_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_state_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double ref_u, ref_psi;
    double e_u, e_psi;
    rclcpp::Time last_time;
    double e_u_sum, e_psi_sum;
    double e_u_prev, e_psi_prev;
    double tau_v, tau_r;
    double kp_u, ki_u, kd_u, kp_psi, ki_psi, kd_psi;
    double m_u, m_v, m_r, m_u_v;
    double d_u, d_v, d_r, d_u_u, d_v_v, d_r_r;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDController>());
  rclcpp::shutdown();
  return 0;
}
