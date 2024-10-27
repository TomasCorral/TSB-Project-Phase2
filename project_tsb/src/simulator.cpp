#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "project_tsb_msgs/msg/boat_position.hpp"
#include "project_tsb_msgs/msg/control_forces.hpp"
#include "project_tsb_msgs/msg/desired_position.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" 

using namespace std::chrono_literals;

template<typename T,typename T1>T max(T &a,T1 b){if(b>a)a=b;return a;}
template<typename T,typename T1>T min(T &a,T1 b){if(b<a)a=b;return a;}


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
      this->declare_parameter("tau_u_limit", 10.0);
      this->declare_parameter("tau_r_limit", 5.0);

      // Read Initial state
      initial_x_ = this->get_parameter("initial_x").as_double();
      initial_y_ = this->get_parameter("initial_y").as_double();
      initial_yaw_ = this->get_parameter("initial_yaw").as_double();
      initial_u_ = this->get_parameter("initial_u").as_double();
      initial_v_ = this->get_parameter("initial_v").as_double();
      initial_r_ = this->get_parameter("initial_r").as_double();

      // Read time constant
      deltat_ = this->get_parameter("deltat").as_double();

      // Read saturation limits
      tau_u_limit_ = this->get_parameter("tau_u_limit").as_double();
      tau_r_limit_ = this->get_parameter("tau_r_limit").as_double();

      // Initialize boat
      this->init_boat();

      // Initialize internal timer used for state update
      timer_ = this->create_wall_timer(std::chrono::duration<double>(this->deltat_), std::bind(&Simulator::update_state, this));

      // Initialize transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Setup publishers and subscribers
      publisher_ = this->create_publisher<project_tsb_msgs::msg::BoatPosition>("topic3", 10);
      subscriber_ = this->create_subscription<project_tsb_msgs::msg::ControlForces>("topic2", 10, std::bind(&Simulator::update_forces, this, std::placeholders::_1));

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
      yaw_ = initial_yaw_;
      u_ = initial_u_;
      v_ = initial_v_;
      r_ = initial_r_;

      // Initialize forces applied
      tau_u_ = 0.0;
      tau_r_ = 0.0;

      // Constants
      m_u_ = 50;
      m_v_ = 60;
      m_r_ = 4.64;
      m_u_v_ = m_u_ - m_v_;
      d_u_ = 0.2;
      d_v_ = 55.1;
      d_r_ = 0.14;
      d_u_u_ = 25;
      d_v_v_ = 0.01;
      d_r_r_ = 6.23;
    }
    void update_state()
    {
      // Get new state
      // Using different variables to prevent usage of new state to calculate new state
      double new_u = u_ + (tau_u_ + m_v_*v_*r_ - d_u_*u_ - d_u_u_*u_*abs(u_)) * deltat_ / m_u_;
      double new_v = v_ + (-m_u_*u_*r_ - d_v_*v_ - d_v_v_*v_*abs(v_)) * deltat_ / m_v_;
      double new_r = r_ + (tau_r_ + m_u_v_*u_*v_ - d_r_*r_ - d_r_r_*r_*abs(r_)) * deltat_ / m_r_;

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
      message.yaw = yaw_;
      message.u = u_;
      message.v = v_;
      message.r = r_;

      // Publish message
      publisher_->publish(message);
    }
    void update_forces(const project_tsb_msgs::msg::ControlForces::SharedPtr msg)
    {
      // Update forces applied to the boat (respect saturation of the boat)
      if (msg->force_u >= 0) {
        tau_u_ = min(msg->force_u, tau_u_limit_);

      } else {
        tau_u_ = max(msg->force_u, -tau_u_limit_);
      }
      if (msg->force_r >= 0) {
        tau_r_ = min(msg->force_r, tau_r_limit_);

      } else {
        tau_r_ = max(msg->force_r, -tau_r_limit_);
      }

      //RCLCPP_INFO(this->get_logger(), "Received new forces: tau_u = %f, tau_r = %f", tau_u, tau_r);
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
        } else if (parameter.get_name() == "tau_u_limit") {
          tau_u_limit_ = parameter.as_double();
        } else if (parameter.get_name() == "tau_r_limit") {
          tau_r_limit_ = parameter.as_double();
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
    rclcpp::Subscription<project_tsb_msgs::msg::ControlForces>::SharedPtr subscriber_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    double initial_x_, initial_y_, initial_yaw_, initial_u_, initial_v_, initial_r_;
    double x_, y_, yaw_, u_, v_, r_;
    double deltat_;
    double tau_u_, tau_r_;
    double tau_u_limit_, tau_r_limit_;
    double m_u_, m_v_, m_r_, m_u_v_;
    double d_u_, d_v_, d_r_, d_u_u_, d_v_v_, d_r_r_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
  return 0;
}
