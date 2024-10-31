#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "project_tsb_msgs/msg/boat_position.hpp"
#include "project_tsb_msgs/msg/control_forces.hpp"
#include "project_tsb_msgs/msg/desired_position.hpp"
#include "project_tsb_msgs/msg/motor_currents.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

using namespace std::chrono_literals;

template<typename T,typename T1>T max(T &a,T1 b){if(b>a)a=b;return a;}
template<typename T,typename T1>T min(T &a,T1 b){if(b<a)a=b;return a;}

// Convert degrees to radians
double degreesToRadians(double degrees) {
  return degrees * M_PI / 180.0;
}

// Convert radians to degrees
double radiansToDegrees(double radians) {
  return radians * 180.0 / M_PI;
}

// ROS2 Node class
class Simulator : public rclcpp::Node
{
  public:
    Simulator()
    : Node("simulator")
    {
      // Declare parameters
      this->declare_parameter("deltat", 1.0);
      this->declare_parameter("initial_x", 0.0);
      this->declare_parameter("initial_y", 0.0);
      this->declare_parameter("initial_yaw", 0.0);
      this->declare_parameter("initial_u", 0.0);
      this->declare_parameter("initial_v", 0.0);
      this->declare_parameter("initial_r", 0.0);
      this->declare_parameter("current_limit", 40.0);

      // Read Initial state
      initial_x_ = this->get_parameter("initial_x").as_double();
      initial_y_ = this->get_parameter("initial_y").as_double();
      initial_yaw_ = this->get_parameter("initial_yaw").as_double();
      initial_u_ = this->get_parameter("initial_u").as_double();
      initial_v_ = this->get_parameter("initial_v").as_double();
      initial_r_ = this->get_parameter("initial_r").as_double();

      // Read time constant
      deltat_ = this->get_parameter("deltat").as_double();

      // Read current limits
      current_limit_ = this->get_parameter("current_limit").as_double();

      // Initialize boat
      this->init_boat();

      // Initialize internal timer used for state update
      timer_ = this->create_wall_timer(std::chrono::duration<double>(this->deltat_), std::bind(&Simulator::update_state, this));

      // Initialize transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Setup publishers and subscribers
      publisher_ = this->create_publisher<project_tsb_msgs::msg::BoatPosition>("topic3", 10);
      subscriber_ = this->create_subscription<project_tsb_msgs::msg::MotorCurrents>("topic2", 10, std::bind(&Simulator::update_input, this, std::placeholders::_1));

      // Setup reset service
      reset_service_ = this->create_service<std_srvs::srv::Trigger>("reset_boat", std::bind(&Simulator::resetBoatCallback, this, std::placeholders::_1, std::placeholders::_2));

      // Setup callback for live parameter updates
      param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&Simulator::on_parameters_change, this, std::placeholders::_1));

      // Publish transformation
      this->publish_transformation();

      // Publish odometry message
      this->publish_position();

      RCLCPP_INFO(this->get_logger(), "Simulator node started");
    }

  private:
    void init_boat() {
      // Initialize boat state
      x_ = initial_x_;
      y_ = initial_y_;
      yaw_ = degreesToRadians(initial_yaw_);
      u_ = initial_u_;
      v_ = initial_v_;
      r_ = initial_r_;

      // Initialize forces applied
      current_p_ = 0.0;
      current_s_ = 0.0;
    }
    void update_state()
    {
      // Convert Currents to Forces
      // NOTE: FUNCTION NOT GOOD FOR LOW CURRENT VALUES, IN FUTURE SIMULATE A DEADZONE
      double force_p = p0_ + p1_*current_p_ + p2_*current_p_;
      double force_s = p0_ + p1_*current_s_ + p2_*current_s_;

      // Convert Motor Forces to Center of Mass Forces
      double force_u = force_p + force_s;
      double force_r = (force_p - force_s) * d_;

      // Get new state
      // Using different variables to prevent usage of new state to calculate new state
      double new_u = u_ + (force_u + m_v_*v_*r_ - d_u_*u_ - d_u_u_*u_*abs(u_)) * deltat_ / m_u_;
      double new_v = v_ + (-m_u_*u_*r_ - d_v_*v_ - d_v_v_*v_*abs(v_)) * deltat_ / m_v_;
      double new_r = r_ + (force_r + m_u_v_*u_*v_ - d_r_*r_ - d_r_r_*r_*abs(r_)) * deltat_ / m_r_;

      double new_x = x_ + (u_*cos(yaw_) - v_*sin(yaw_)) * deltat_;
      double new_y = y_ + (u_*sin(yaw_) + v_*cos(yaw_)) * deltat_;
      double new_yaw= yaw_ + r_ * deltat_;

      // Update Boat state
      x_ = new_x;
      y_ = new_y;
      yaw_ = new_yaw;
      u_ = new_u;
      v_ = new_v;
      r_ = new_r;

      // Normalize betwwen -pi to pi
      while (yaw_ > M_PI) yaw_ -= 2 * M_PI;
      while (yaw_ < -M_PI) yaw_ += 2 * M_PI;
      
      // Publish transformation
      this->publish_transformation();

      // Publish the odometry message with the new state
      this->publish_position();


      RCLCPP_INFO(this->get_logger(), "Boat position: x = %f, y = %f, Yaw = %f, u = %f, v = %f, r = %f", x_, y_, yaw_, u_, v_, r_);

    }
    void publish_transformation() {

      tf2::Quaternion q;
      q.setRPY(0, 0, yaw_);

      // Build tf
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "world";
      t.child_frame_id = "base_link";
      t.transform.translation.x = x_;
      t.transform.translation.y = y_;
      t.transform.translation.z = 0.0;
      t.transform.rotation = tf2::toMsg(q);

      // Publish tf
      tf_broadcaster_->sendTransform(t);
    }
    void publish_position()
    { 

      // Create message
      project_tsb_msgs::msg::BoatPosition message;
      message.header.stamp = this->now();
      message.header.frame_id = "world"; //Only x,y,yaw are in this frame
      message.x = x_;
      message.y = y_;
      message.yaw = radiansToDegrees(yaw_);
      message.u = u_;
      message.v = v_;
      message.r = r_;

      // Publish message
      publisher_->publish(message);
    }
    void update_input(const project_tsb_msgs::msg::MotorCurrents::SharedPtr msg)
    {
      // Update forces applied to the boat (respect saturation of the boat)
      if (msg->current_p >= 0) {
        current_p_ = min(msg->current_p, current_limit_);

      } else {
        current_p_ = max(msg->current_p, -current_limit_);
      }
      if (msg->current_s >= 0) {
        current_s_ = min(msg->current_s, current_limit_);

      } else {
        current_s_ = max(msg->current_s, -current_limit_);
      }

      //RCLCPP_INFO(this->get_logger(), "Received new forces: Force_u = %f, Force_r = %f", force_u, force_r);
    }
    void resetBoatCallback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
    {
      // Reset boat to initial state
      this->init_boat();

      // Publish transformation
      this->publish_transformation();

      // Publish odometry message
      this->publish_position();

      // Prevent unused parameter warning
      (void)request;

      // Send sucess response
      response->success = true;
      response->message = "Boat reseted";

      RCLCPP_INFO(this->get_logger(), "Boat reseted");
    }
    rcl_interfaces::msg::SetParametersResult on_parameters_change(const std::vector<rclcpp::Parameter> &parameters)
    {
      // Update parameters
      for (const auto &parameter : parameters) {
        if (parameter.get_name() == "deltat") {
          deltat_ = parameter.as_double();
          timer_->cancel();
          timer_ = this->create_wall_timer(std::chrono::duration<double>(deltat_), std::bind(&Simulator::update_state, this));
        } else if (parameter.get_name() == "curent_limit") {
          current_limit_ = parameter.as_double();
        } else if (parameter.get_name() == "initial_x") {
          initial_x_ = parameter.as_double();
        } else if (parameter.get_name() == "initial_y") {
          initial_y_ = parameter.as_double();
        } else if (parameter.get_name() == "initial_yaw") {
          initial_yaw_ = parameter.as_double();
        } else if (parameter.get_name() == "initial_u") {
          initial_u_ = parameter.as_double();
        } else if (parameter.get_name() == "initial_v") {
          initial_v_ = parameter.as_double();
        } else if (parameter.get_name() == "initial_r") {
          initial_r_ = parameter.as_double();
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
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<project_tsb_msgs::msg::BoatPosition>::SharedPtr publisher_;
    rclcpp::Subscription<project_tsb_msgs::msg::MotorCurrents>::SharedPtr subscriber_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    double initial_x_, initial_y_, initial_yaw_, initial_u_, initial_v_, initial_r_;
    double x_, y_, yaw_, u_, v_, r_;
    double deltat_;
    double current_p_, current_s_;
    double current_limit_;
    // Constants for motion model
    const double m_u_=50, m_v_=60, m_r_=4.64; 
    const double m_u_v_= m_u_ - m_v_;
    const double d_u_=0.2, d_v_=55.1, d_r_=0.14, d_u_u_=25, d_v_v_=0.01, d_r_r_=6.23;
    // Distance between motors/2
    const double d_ = 0.45;
    // Constants for Current->Force Conversion
    double const p0_ = 2.09739;
    double const p1_ = 3.34596;
    double const p2_ = -0.06520;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
  return 0;
}
