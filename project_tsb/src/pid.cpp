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
      // Delcare parameters
      // TODO: MOVE TO 2 ARRAYS
      this->declare_parameter("kp_u", 0.0);
      this->declare_parameter("ki_u", 0.0);
      this->declare_parameter("kd_u", 0.0);
      this->declare_parameter("kp_psi", 0.0);
      this->declare_parameter("ki_psi", 0.0);
      this->declare_parameter("kd_psi", 0.0);
      this->declare_parameter("deltat", 2.0);

      // References
      this->ref_u = 0.0;
      this->ref_psi = 0.0;

      // Current state
      this->u = 0.0;
      this->psi = 0.0;

      // Errors
      this -> e_u = 0.0;
      this -> e_psi = 0.0;

      // Sampling time
      this->deltat = this->get_parameter("deltat").as_double();

      // Integrator(sum) and Derivator(last value)
      // TODO: criar classe para dar handle ao integrador/derivador?
      rclcpp::Time zero_time(0, 0, this->get_clock()->get_clock_type());
      this->last_time = zero_time;
      this->e_u_sum = 0.0;
      this->e_psi_sum = 0.0;
      this->e_u_prev = 0.0;
      this->e_psi_prev = 0.0;

      // PID Outputs
      this->tau_u = 0.0;
      this->tau_r = 0.0;

      // PID Gains
      this->kp_u = this->get_parameter("kp_u").as_double();
      this->ki_u = this->get_parameter("ki_u").as_double();
      this->kd_u = this->get_parameter("kd_u").as_double();
      this->kp_psi = this->get_parameter("kp_psi").as_double();
      this->ki_psi = this->get_parameter("ki_psi").as_double();
      this->kd_psi = this->get_parameter("kd_psi").as_double();

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

      // Setup callback for live parameter updates
      param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PIDController::on_parameters_change, this, std::placeholders::_1));

      // Setup state update timer
      // Initialize internal timer used for state update
      timer_ = this->create_wall_timer(std::chrono::duration<double>(this->deltat), std::bind(&PIDController::update_output, this));

      RCLCPP_INFO(this->get_logger(), "PID Controller node started");
      RCLCPP_INFO(this->get_logger(), "Controller Gains: kp_u = %f, ki_u = %f, kd_u = %f, kp_psi = %f, ki_psi = %f, kd_psi = %f", this->kp_u, this->ki_u, this->kd_u, this->kp_psi, this->ki_psi, this->kd_psi);
    }

  private:
    void update_ref(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      if (this->ref_u != msg->data[0] || this->ref_psi != msg->data[1] * (M_PI / 180)) { //Used to avoid consoel spam
        this->ref_u = msg->data[0];
        this->ref_psi = msg->data[1] * (M_PI / 180); //Ref is given in degres but processed in radians

        while (this->ref_psi > M_PI) this->ref_psi -= 2 * M_PI;
        while (this->ref_psi < -M_PI) this->ref_psi += 2 * M_PI;  

        //RCLCPP_INFO(this->get_logger(), "Received new reference: u: %f  psi: %f", this->ref_u, this->ref_psi);
      }
    }
    void update_state(const nav_msgs::msg::Odometry::SharedPtr msg) // Calculate next PID output and publish it
    {
      double roll, pitch, yaw;

      // Extract yaw from quaternion
      tf2::Quaternion q;
      tf2::fromMsg(msg->pose.pose.orientation, q);
      tf2::Matrix3x3 m(q); //https://answers.ros.org/question/339528/quaternion-to-rpy-ros2/
      m.getRPY(roll, pitch, yaw);

      this->u = msg->twist.twist.linear.x;
      this->psi = yaw;

      //RCLCPP_INFO(this->get_logger(), "Received new system state: u: %f  psi: %f", this->u, this->psi);
    }
    void update_output() { // Calculate next PID output and publish it
      rclcpp::Time zero_time(0, 0, this->get_clock()->get_clock_type());
      bool first_time = (this->last_time == zero_time);
      double e_u_dt, e_psi_dt; 

      // Update error
      this->e_u = this->ref_u - this->u;
      //this->e_psi = fmod(fabs(this->ref_psi - this->psi) + M_PI, 2*M_PI) - M_PI;
      this->e_psi = this->ref_psi - this->psi;
      if (this->e_psi < -M_PI) this->e_psi += 2 * M_PI;
      else if (this->e_psi > M_PI) this->e_psi -= 2 * M_PI;

      // Update Integrator and Derivative term
      if (!first_time) {
        this->e_u_sum += this->e_u * (this->now() - this->last_time).seconds();
        this->e_psi_sum += this->e_psi * (this->now() - this->last_time).seconds();
        e_u_dt = (this->e_u - this->e_u_prev) / (this->now() - this->last_time).seconds();
        e_psi_dt = (this->e_psi - this->e_psi_prev) / (this->now() - this->last_time).seconds();
      }
      
      // Update last time and last error
      this->last_time = this->now();
      this->e_u_prev = e_u;
      this->e_psi_prev = e_psi;

      // Calculate PID output
      this->tau_u = this->kp_u*this->e_u + this->ki_u*this->e_u_sum + this->kd_u*e_u_dt;
      this->tau_r = this->kp_psi*this->e_psi + this->ki_psi*this->e_psi_sum + this->kd_psi*e_psi_dt;

      // Publish output
      std_msgs::msg::Float32MultiArray output;
      output.data.resize(2);
      output.data[0] = this->tau_u;
      output.data[1] = this->tau_r;


      //RCLCPP_INFO(this->get_logger(), "Current state: u = %f, psi = %f Desired state: u = %f, psi = %f Error: u= %f, psi = %f", this->u, this->psi, this->ref_u, this->ref_psi, this->e_u, this->e_psi);
      if (first_time) {
        RCLCPP_INFO(this->get_logger(), "First time running, skipping PID calculation");
        
      } else {
        publisher_->publish(output);
        //RCLCPP_INFO(this->get_logger(), "Published new forces: tau_u = %f, tau_psi = %f", this->tau_u, this->tau_r);
      }

    }
    rcl_interfaces::msg::SetParametersResult on_parameters_change(const std::vector<rclcpp::Parameter> &parameters)
    {
      // Update parameters
      for (const auto &parameter : parameters) {
        if (parameter.get_name() == "deltat") {
          this->deltat = parameter.as_double();
          timer_->cancel();
          timer_ = this->create_wall_timer(std::chrono::duration<double>(this->deltat), std::bind(&PIDController::update_output, this));
        } else if (parameter.get_name() == "kp_u") {
          this->kp_u = parameter.as_double();
        } else if (parameter.get_name() == "ki_u") {
          this->ki_u = parameter.as_double();
        } else if (parameter.get_name() == "kd_u") {
          this->kd_u = parameter.as_double();
        } else if (parameter.get_name() == "kp_psi") {
          this->kp_psi = parameter.as_double();
        } else if (parameter.get_name() == "ki_psi") {
          this->ki_psi = parameter.as_double();
        } else if (parameter.get_name() == "kd_psi") {
          this->kd_psi = parameter.as_double();
        }
      }

      // Send success response
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "Parameter updated";

      RCLCPP_INFO(this->get_logger(), "Parameter %s updated", parameters[0].get_name().c_str());
      return result;
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_ref_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_state_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    double ref_u, ref_psi;
    double u, psi;
    double e_u, e_psi;
    rclcpp::Time last_time;
    double deltat;
    double e_u_sum, e_psi_sum;
    double e_u_prev, e_psi_prev;
    double tau_u, tau_r;
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
