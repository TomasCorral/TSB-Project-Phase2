#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "project_tsb_msgs/msg/boat_position.hpp"
#include "project_tsb_msgs/msg/control_forces.hpp"
#include "project_tsb_msgs/msg/desired_position.hpp"

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
      this->declare_parameter("kp_yaw", 0.0);
      this->declare_parameter("ki_yaw", 0.0);
      this->declare_parameter("kd_yaw", 0.0);
      this->declare_parameter("deltat", 2.0);

      // References
      ref_u_ = 0.0;
      ref_yaw_ = 0.0;

      // Current state
      u_ = 0.0;
      yaw_ = 0.0;

      // Errors
      e_u_ = 0.0;
      e_yaw_ = 0.0;

      // Sampling time
      deltat_ = this->get_parameter("deltat").as_double();

      // Integrator(sum) and Derivator(last value)
      // TODO: criar classe para dar handle ao integrador/derivador?
      rclcpp::Time zero_time(0, 0, this->get_clock()->get_clock_type());
      last_time_ = zero_time;
      e_u_sum_ = 0.0;
      e_yaw_sum_ = 0.0;
      e_u_prev_ = 0.0;
      e_yaw_prev_ = 0.0;

      // PID Outputs
      tau_u_ = 0.0;
      tau_r_ = 0.0;

      // PID Gains
      kp_u_ = this->get_parameter("kp_u").as_double();
      ki_u_ = this->get_parameter("ki_u").as_double();
      kd_u_ = this->get_parameter("kd_u").as_double();
      kp_yaw_ = this->get_parameter("kp_yaw").as_double();
      ki_yaw_ = this->get_parameter("ki_yaw").as_double();
      kd_yaw_ = this->get_parameter("kd_yaw").as_double();

      // Setup publishers and subscribers
      publisher_ = this->create_publisher<project_tsb_msgs::msg::ControlForces>("topic2", 10); //Publisher used to publish the odometry
      subscriber_ref_ = this->create_subscription<project_tsb_msgs::msg::DesiredPosition>("topic1", 10, std::bind(&PIDController::update_ref, this, std::placeholders::_1));
      subscriber_state_ = this->create_subscription<project_tsb_msgs::msg::BoatPosition>("topic3", 10, std::bind(&PIDController::update_state, this, std::placeholders::_1));

      // Setup callback for live parameter updates
      param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PIDController::on_parameters_change, this, std::placeholders::_1));

      // Setup state update timer
      timer_ = this->create_wall_timer(std::chrono::duration<double>(deltat_), std::bind(&PIDController::update_output, this));

      RCLCPP_INFO(this->get_logger(), "PID Controller node started");
      RCLCPP_INFO(this->get_logger(), "Controller Gains: kp_u = %f, ki_u = %f, kd_u = %f, kp_yaw = %f, ki_yaw = %f, kd_yaw = %f", kp_u_, ki_u_, kd_u_, kp_yaw_, ki_yaw_, kd_yaw_);
    }

  private:
    void update_ref(const project_tsb_msgs::msg::DesiredPosition::SharedPtr msg)
    {
      // Extract reference from msg
      if (ref_u_ != msg->u || ref_yaw_ != msg->yaw * (M_PI / 180)) { //Used to avoid consoel spam
        ref_u_ = msg->u;
        ref_yaw_ = msg->yaw * (M_PI / 180); //Ref is given in degres but processed in radians

        while (ref_yaw_ > M_PI) ref_yaw_ -= 2 * M_PI;
        while (ref_yaw_ < -M_PI) ref_yaw_ += 2 * M_PI;  

        //RCLCPP_INFO(this->get_logger(), "Received new reference: u: %f  yaw: %f", ref_u_, ref_yaw_);
      }
    }
    void update_state(const project_tsb_msgs::msg::BoatPosition::SharedPtr msg) // Calculate next PID output and publish it
    {
  
      // Extract current state from msg
      u_ = msg->u;
      yaw_ = msg->yaw;

      //RCLCPP_INFO(this->get_logger(), "Received new system state: u: %f  yaw: %f", u_, yaw_);
    }
    void update_output() { // Calculate next PID output and publish it
      rclcpp::Time zero_time(0, 0, this->get_clock()->get_clock_type());
      bool first_time = (last_time_ == zero_time);
      double e_u_dt, e_yaw_dt; 

      // Update error
      e_u_ = ref_u_ - u_;
      //e_yaw_ = fmod(fabs(ref_yaw_ - yaw_) + M_PI, 2*M_PI) - M_PI;
      e_yaw_ = ref_yaw_ - yaw_;
      if (e_yaw_ < -M_PI) e_yaw_ += 2 * M_PI;
      else if (e_yaw_ > M_PI) e_yaw_ -= 2 * M_PI;

      // Update Integrator and Derivative term
      if (!first_time) {
        e_u_sum_ += e_u_ * (this->now() - last_time_).seconds();
        e_yaw_sum_ += e_yaw_ * (this->now() - last_time_).seconds();
        e_u_dt = (e_u_ - e_u_prev_) / (this->now() - last_time_).seconds();
        e_yaw_dt = (e_yaw_ - e_yaw_prev_) / (this->now() - last_time_).seconds();
      }
      
      // Update last time and last error
      last_time_ = this->now();
      e_u_prev_ = e_u_;
      e_yaw_prev_ = e_yaw_;

      // Calculate PID output
      tau_u_ = kp_u_*e_u_ + ki_u_*e_u_sum_ + kd_u_*e_u_dt;
      tau_r_ = kp_yaw_*e_yaw_ + ki_yaw_*e_yaw_sum_ + kd_yaw_*e_yaw_dt;

      // Publish output
      project_tsb_msgs::msg::ControlForces output;
      output.header.stamp = this->now();
      output.force_u = tau_u_;
      output.force_r = tau_r_;

      //RCLCPP_INFO(this->get_logger(), "Current state: u = %f, yaw = %f Desired state: u = %f, yaw = %f Error: u= %f, yaw = %f", u_, yaw_, ref_u_, ref_yaw_, e_u_, e_yaw_);
      if (first_time) {
        RCLCPP_INFO(this->get_logger(), "First time running, skipping PID calculation");
        
      } else {
        publisher_->publish(output);
        //RCLCPP_INFO(this->get_logger(), "Published new forces: tau_u = %f, tau_yaw = %f", tau_u_, tau_r_);
      }

    }
    rcl_interfaces::msg::SetParametersResult on_parameters_change(const std::vector<rclcpp::Parameter> &parameters)
    {
      // Update parameters
      for (const auto &parameter : parameters) {
        if (parameter.get_name() == "deltat") {
          deltat_ = parameter.as_double();
          timer_->cancel();
          timer_ = this->create_wall_timer(std::chrono::duration<double>(deltat_), std::bind(&PIDController::update_output, this));
        } else if (parameter.get_name() == "kp_u") {
          kp_u_ = parameter.as_double();
        } else if (parameter.get_name() == "ki_u") {
          ki_u_ = parameter.as_double();
        } else if (parameter.get_name() == "kd_u") {
          kd_u_ = parameter.as_double();
        } else if (parameter.get_name() == "kp_yaw") {
          kp_yaw_ = parameter.as_double();
        } else if (parameter.get_name() == "ki_yaw") {
          ki_yaw_ = parameter.as_double();
        } else if (parameter.get_name() == "kd_yaw") {
          kd_yaw_ = parameter.as_double();
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
    rclcpp::Publisher<project_tsb_msgs::msg::ControlForces>::SharedPtr publisher_;
    rclcpp::Subscription<project_tsb_msgs::msg::DesiredPosition>::SharedPtr subscriber_ref_;
    rclcpp::Subscription<project_tsb_msgs::msg::BoatPosition>::SharedPtr subscriber_state_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    double ref_u_, ref_yaw_;
    double u_, yaw_;
    double e_u_, e_yaw_;
    rclcpp::Time last_time_;
    double deltat_;
    double e_u_sum_, e_yaw_sum_;
    double e_u_prev_, e_yaw_prev_;
    double tau_u_, tau_r_;
    double kp_u_, ki_u_, kd_u_, kp_yaw_, ki_yaw_, kd_yaw_;
    const double m_u_=50, m_v_=60, m_r_=4.64; 
    const double m_u_v_= m_u_ - m_v_;
    const double d_u_=0.2, d_v_=55.1, d_r_=0.14, d_u_u_=25, d_v_v_=0.01, d_r_r_=6.23;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDController>());
  rclcpp::shutdown();
  return 0;
}
