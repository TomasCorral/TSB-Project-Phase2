#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "project_tsb_msgs/msg/boat_position.hpp"
#include "project_tsb_msgs/msg/boat_reference.hpp"
#include "project_tsb_msgs/msg/control_forces.hpp"

using namespace std::chrono_literals;

// Convert degrees to radians
double degreesToRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

// Convert radians to degrees
double radiansToDegrees(double radians) {
  return radians * 180.0 / M_PI;
}

// ROS2 Node class
class PIDController : public rclcpp::Node
{
  public:
    PIDController()
    : Node("PID_Controller")
    {
      // Delcare parameters
      this->declare_parameter("deltat", 0.5);
      this->declare_parameter("reset_integrator_u", false);
      this->declare_parameter("reset_integrator_yaw", false);
      this->declare_parameter("skip_derivator_u", false);
      this->declare_parameter("skip_derivator_yaw", false);
      this->declare_parameter("acceptable_yaw_error", 0.0);
      //this->declare_parameter("gain_schedules", std::vector<std::vector<double>>());

      // References
      ref_u_ = 0.0;
      ref_yaw_ = 0.0;

      // Current state
      u_ = 0.0;
      yaw_ = 0.0;

      // Errors
      error_u_ = 0.0;
      error_yaw_ = 0.0;

      // Sampling time
      deltat_ = this->get_parameter("deltat").as_double();

      // Load configs 
      reset_integrator_u_ = this->get_parameter("reset_integrator_u").as_bool();
      reset_integrator_yaw_ = this->get_parameter("reset_integrator_yaw").as_bool();
      skip_derivator_u_ = this->get_parameter("skip_derivator_u").as_bool();
      skip_derivator_yaw_ = this->get_parameter("skip_derivator_yaw").as_bool();
      acceptable_yaw_error_ = this->get_parameter("acceptable_yaw_error").as_double();

      // Integrator(sum) and Derivator(last value)
      error_u_sum_ = 0.0;
      error_yaw_sum_ = 0.0;
      error_u_prev_ = 0.0;
      error_yaw_prev_ = 0.0;

      // PID Outputs
      force_u_ = 0.0;
      force_r_ = 0.0;

      // PID Gains
      gain_schedules_ = {
        { {38.0, 17.5, 0.0}, {2.2, 0.0, 28.0}, -1.5 },
        { {38.0, 17.5, 0.0}, {2.5, 0.0, 14.0}, -1.0 },
        { {30.0, 17.0, 0.0}, {3.5, 0.0, 5.0}, 0.0 },
        { {30.0, 17.0, 0.0}, {4.0, 0.0, 11.0}, 0.5 }, // Tunning fair enough
        { {36.0, 17.5, 0.0}, {2.5, 0.0, 14.0}, 1.0 }, // Tunning fair enough
        { {40.0, 18.0, 0.0}, {2.2, 0.0, 28.0}, 1.5 }, // Tunning fair enough
        { {45.0, 18.5, 0.0}, {1.0, 0.0, 30.0}, 1.8 } //Tunning fair enough, Max speed with 20A in each motor
      };

      // Setup publishers and subscribers
      publisher_ = this->create_publisher<project_tsb_msgs::msg::ControlForces>("pid_output", 10); //Publisher used to publish the odometry
      subscriber_ref_ = this->create_subscription<project_tsb_msgs::msg::BoatReference>("boat_reference", 10, std::bind(&PIDController::update_ref, this, std::placeholders::_1));
      subscriber_state_ = this->create_subscription<project_tsb_msgs::msg::BoatPosition>("boat_state", 10, std::bind(&PIDController::update_state, this, std::placeholders::_1));

      // Setup callback for live parameter updates
      param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PIDController::on_parameters_change, this, std::placeholders::_1));

      // Setup state update timer
      timer_ = this->create_wall_timer(std::chrono::duration<double>(deltat_), std::bind(&PIDController::update_output, this));

      RCLCPP_INFO(this->get_logger(), "PID Controller node started");
      //RCLCPP_INFO(this->get_logger(), "Controller Gains: kp_u = %f, ki_u = %f, kd_u = %f, kp_yaw = %f, ki_yaw = %f, kd_yaw = %f", kp_u_, ki_u_, kd_u_, kp_yaw_, ki_yaw_, kd_yaw_);
    }

  private:
    void update_gains() {
      // Find the gain schedule in which u0 is closest to u_
      double min_diff = std::numeric_limits<double>::max();
      size_t closest_index = 0;
      // Linear search but gain schedules are expected to be small
      for (size_t i = 0; i < gain_schedules_.size(); ++i) {
        double diff = std::abs(gain_schedules_[i].u0 - u_);
        if (diff < min_diff) {
          min_diff = diff;
          closest_index = i;
        }
      }

      const auto &closest_gain_schedule = gain_schedules_[closest_index];
      kp_u_ = closest_gain_schedule.u_gains.kp;
      ki_u_ = closest_gain_schedule.u_gains.ki;
      kd_u_ = closest_gain_schedule.u_gains.kd;
      kp_yaw_ = closest_gain_schedule.yaw_gains.kp;
      ki_yaw_ = closest_gain_schedule.yaw_gains.ki;
      kd_yaw_ = closest_gain_schedule.yaw_gains.kd;

      //RCLCPP_INFO(this->get_logger(), "Current Controller Gains: kp_u = %f, ki_u = %f, kd_u = %f, kp_yaw = %f, ki_yaw = %f, kd_yaw = %f", kp_u_, ki_u_, kd_u_, kp_yaw_, ki_yaw_, kd_yaw_);
    }


    void update_ref(const project_tsb_msgs::msg::BoatReference::SharedPtr msg)
    {
      if (ref_u_ != msg->u) {
        ref_u_ = msg->u;

        // Reset integrator
        if (reset_integrator_u_) error_u_sum_ = 0.0;
        

        // Bypass first integrator update
        if (skip_derivator_u_) skip_next_derivator_u_ = true;
      }
      if (ref_yaw_ != msg->yaw * (M_PI / 180)) {
        // Ref is given in degres but processed in radians
        ref_yaw_ = degreesToRadians(msg->yaw);
        while (ref_yaw_ > M_PI) ref_yaw_ -= 2 * M_PI;
        while (ref_yaw_ < -M_PI) ref_yaw_ += 2 * M_PI; 

        // Reset integrator
        if (reset_integrator_yaw_) error_yaw_sum_ = 0.0;

        // Bypass first integrator update
        if (skip_derivator_yaw_) skip_next_derivator_yaw_ = true;
      }

      ref_received_ = true;
      //RCLCPP_INFO(this->get_logger(), "Received new reference: u: %f  yaw: %f", ref_u_, ref_yaw_);
      
    }
    void update_state(const project_tsb_msgs::msg::BoatPosition::SharedPtr msg) // Calculate next PID output and publish it
    {
      // Extract current state from msg
      u_ = msg->u;
      yaw_ = degreesToRadians(msg->yaw);

      state_received_ = true;

      //RCLCPP_INFO(this->get_logger(), "Received new system state: u: %f  yaw: %f", u_, degreesToRadians(yaw_));
    }
    void update_output() { // Calculate next PID output and publish it
      if (!ref_received_ || !state_received_) return;

      // Get current gains
      update_gains();

      double error_u_dt, error_yaw_dt; 

      // Update error
      error_u_ = ref_u_ - u_;
      //e_yaw_ = fmod(fabs(ref_yaw_ - yaw_) + M_PI, 2*M_PI) - M_PI;
      error_yaw_ = ref_yaw_ - yaw_;
      if (error_yaw_ < -M_PI) error_yaw_ += 2 * M_PI;
      else if (error_yaw_ > M_PI) error_yaw_ -= 2 * M_PI;

      // Update integrator
      error_u_sum_ += error_u_ * deltat_;
      error_yaw_sum_ += error_yaw_ * deltat_;

      // Derivativator, skip first time after reference change
      if (!skip_next_derivator_u_) {
        error_u_dt = (error_u_ - error_u_prev_) / deltat_;
      } else {
        skip_next_derivator_u_ = false;
      }
      
      if (!skip_next_derivator_yaw_) {
        error_yaw_dt = (error_yaw_ - error_yaw_prev_) / deltat_;
      } else {
        skip_next_derivator_yaw_ = false;
      }
      
      // Update last time and last error
      error_u_prev_ = error_u_;
      error_yaw_prev_ = error_yaw_;

      // Calculate PID output
      force_u_ = kp_u_*error_u_ + ki_u_*error_u_sum_ + kd_u_*error_u_dt;
      force_r_ = kp_yaw_*error_yaw_ + ki_yaw_*error_yaw_sum_ + kd_yaw_*error_yaw_dt;

      // Stop yaw movement if error is to small
      if (abs(error_yaw_) < degreesToRadians(acceptable_yaw_error_) && ref_u_ == 0.0) force_r_ = 0.0;


      // Publish output
      project_tsb_msgs::msg::ControlForces output;
      output.header.stamp = this->now();
      output.force_u = force_u_;
      output.force_r = force_r_;

      //RCLCPP_INFO(this->get_logger(), "Current state: u = %f, yaw = %f Desired state: u = %f, yaw = %f Error: u= %f, yaw = %f", u_, degreesToRadians(yaw_), ref_u_, ref_yaw_, error_u_, error_yaw_);
      //RCLCPP_INFO(this->get_logger(), "Published new forces: force_u = %f, force_yaw = %f", force_u_, force_r_);
      publisher_->publish(output);

    }
    rcl_interfaces::msg::SetParametersResult on_parameters_change(const std::vector<rclcpp::Parameter> &parameters)
    {
      // Update parameters
      for (const auto &parameter : parameters) {
        if (parameter.get_name() == "deltat") {
          deltat_ = parameter.as_double();
          timer_->cancel();
          timer_ = this->create_wall_timer(std::chrono::duration<double>(deltat_), std::bind(&PIDController::update_output, this));
        } else if (parameter.get_name() == "reset_integrator_u") {
          reset_integrator_u_ = parameter.as_bool();
        } else if (parameter.get_name() == "reset_integrator_yaw") {
          reset_integrator_yaw_ = parameter.as_bool();
        } else if (parameter.get_name() == "skip_derivator_u") {
          skip_derivator_u_ = parameter.as_bool();
        } else if (parameter.get_name() == "skip_derivator_yaw") {
          skip_derivator_yaw_ = parameter.as_bool();
        } else if (parameter.get_name() == "acceptable_yaw_error") {
          acceptable_yaw_error_ = parameter.as_double();
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
    rclcpp::Subscription<project_tsb_msgs::msg::BoatReference>::SharedPtr subscriber_ref_;
    rclcpp::Subscription<project_tsb_msgs::msg::BoatPosition>::SharedPtr subscriber_state_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // PID variables
    double ref_u_, ref_yaw_;
    double u_, yaw_;
    double error_u_, error_yaw_;
    double error_u_sum_, error_yaw_sum_;
    double error_u_prev_, error_yaw_prev_;

    // Reference and state received flags
    bool ref_received_ = false;
    bool state_received_ = false;

    // Derivator bypass flags
    bool skip_next_derivator_u_ = true;
    bool skip_next_derivator_yaw_ = true;

    // PID Configs
    bool reset_integrator_u_ = false;
    bool reset_integrator_yaw_ = false;
    bool skip_derivator_u_ = false;
    bool skip_derivator_yaw_ = false;
    double acceptable_yaw_error_;

    double deltat_;
    double force_u_, force_r_;
    const double m_u_=50, m_v_=60, m_r_=4.64; 
    const double m_u_v_= m_u_ - m_v_;
    const double d_u_=0.2, d_v_=55.1, d_r_=0.14, d_u_u_=25, d_v_v_=0.01, d_r_r_=6.23;

    // PID Gains
    double kp_u_, ki_u_, kd_u_, kp_yaw_, ki_yaw_, kd_yaw_;
    struct PIDGains {
      double kp;
      double ki;
      double kd;
    };
    struct GainSchedule {
      PIDGains u_gains;
      PIDGains yaw_gains;
      double u0;
      //v0, r0 = 0
    };
    std::vector<GainSchedule> gain_schedules_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDController>());
  rclcpp::shutdown();
  return 0;
}
