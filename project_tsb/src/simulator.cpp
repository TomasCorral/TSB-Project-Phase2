#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
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
      this->declare_parameter("delta_t", 1.0);
      this->declare_parameter("initial_x", 0.0);
      this->declare_parameter("initial_y", 0.0);
      this->declare_parameter("initial_psi", 0.0);
      this->declare_parameter("initial_u", 0.0);
      this->declare_parameter("initial_v", 0.0);
      this->declare_parameter("initial_r", 0.0);
      this->declare_parameter("tau_u_limit", 10.0);
      this->declare_parameter("tau_r_limit", 5.0);

      // Initialize boat
      this->init_boat();

      // Initialize internal timer used for state update
      timer_ = this->create_wall_timer(std::chrono::duration<double>(this->deltat), std::bind(&Simulator::update_state, this));

      // Initialize transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Setup publishers and subscribers
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("topic3", 10); //Publisher used to publish the odometry
      subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("topic2", 10, std::bind(&Simulator::update_forces, this, std::placeholders::_1));

      // Setup reset service
      reset_service = this->create_service<std_srvs::srv::Trigger>("reset_boat", std::bind(&Simulator::resetBoatCallback, this, std::placeholders::_1, std::placeholders::_2));

      // Publish transformation
      this->publish_transformation();

      // Publish odometry message
      this->publish_odometry();

      RCLCPP_INFO(this->get_logger(), "Simulator node started");
    }

  private:
    void init_boat() {
      // Initial state
      this->x = this->get_parameter("initial_x").as_double();
      this->y = this->get_parameter("initial_y").as_double();
      this->psi = this->get_parameter("initial_psi").as_double();
      this->u = this->get_parameter("initial_u").as_double();
      this->v = this->get_parameter("initial_v").as_double();
      this->r = this->get_parameter("initial_r").as_double();


      // Time constant used for state update
      this->deltat = this->get_parameter("delta_t").as_double();

      // Initialize forces applied
      this->tau_u = 0.0;
      this->tau_r = 0.0;

      // Saturation limits
      this->tau_u_limit = this->get_parameter("tau_u_limit").as_double();
      this->tau_r_limit = this->get_parameter("tau_r_limit").as_double();

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
    }
    void update_state()
    {
      // Update robot state
      // TODO: Trocar dinamica com cinematica? Probably not, perguntar
      double new_u = this->u + (this->tau_u + this->m_v*this->v*this->r - this->d_u*this->u - this->d_u_u*this->u*abs(this->u)) * this->deltat / this->m_u;
      double new_v = this->v + (-this->m_u*this->u*this->r - this->d_v*this->v - this->d_v_v*this->v*abs(this->v)) * this->deltat / this->m_v;
      double new_r = this->r + (this->tau_r + this->m_u_v*this->u*this->v - this->d_r*this->r - this->d_r_r*this->r*abs(this->r)) * this->deltat / this->m_r;

      double new_x = this->x + (this->u*cos(this->psi) - this->v*sin(this->psi)) * this->deltat;
      double new_y = this->y + (this->u*sin(this->psi) + this->v*cos(this->psi)) * this->deltat;
      double new_psi = this->psi + this->r * this->deltat;

      // Using different variables to prevent usage of new state to calculate new state
      this->u = new_u;
      this->v = new_v;
      this->r = new_r;
      this->x = new_x;
      this->y = new_y;
      this->psi = new_psi;
      while (this->psi > M_PI) this->psi -= 2 * M_PI; // Normalize betwwen -pi to pi
      while (this->psi < -M_PI) this->psi += 2 * M_PI;
      

      // Publish transformation
      this->publish_transformation();

      // Publish the odometry message with the new state
      this->publish_odometry();


      RCLCPP_INFO(this->get_logger(), "Robot position: x = %f, y = %f, psi = %f, u = %f, v = %f, r = %f", this->x, this->y, this->psi, this->u, this->v, this->r);

    }
    void publish_transformation() {

      tf2::Quaternion q;
      q.setRPY(0, 0, this->psi);

      // Build tf
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "world";
      t.child_frame_id = "base_link";
      t.transform.translation.x = this->x;
      t.transform.translation.y = this->y;
      t.transform.translation.z = 0.0;
      t.transform.rotation = tf2::toMsg(q);

      // Publish tf
      tf_broadcaster_->sendTransform(t);
    }
    void publish_odometry()
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, this->psi);

      // Build odometry
      nav_msgs::msg::Odometry odometry;
      odometry.header.stamp = this->now();
      odometry.header.frame_id = "world";
      odometry.child_frame_id = "base_link";
      odometry.pose.pose.position.x = this->x; //x
      odometry.pose.pose.position.y = this->y; //y
      odometry.pose.pose.position.z = 0.0;
      odometry.pose.pose.orientation = tf2::toMsg(q); //psi inside quaternion
      odometry.twist.twist.linear.x = this->u; //u
      odometry.twist.twist.linear.y = this->v; //v
      odometry.twist.twist.linear.z = 0.0;
      odometry.twist.twist.angular.x = 0.0;
      odometry.twist.twist.angular.y = 0.0;
      odometry.twist.twist.angular.z = this->r; //r

      // Publish odometry
      publisher_->publish(odometry);
    }
    void update_forces(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
      // Update forces applied to the boat (respect saturation of the boat)
      
      if (msg->data[0] >= 0) {
        this->tau_u = min(msg->data[0], this->tau_u_limit);

      } else {
        this->tau_u = max(msg->data[0], -this->tau_u_limit);
      }
      if (msg->data[1] >= 0) {
        this->tau_r = min(msg->data[1], this->tau_r_limit);

      } else {
        this->tau_r = max(msg->data[1], -this->tau_r_limit);
      }

      //RCLCPP_INFO(this->get_logger(), "Received new forces: tau_u = %f, tau_r = %f", this->tau_u, this->tau_r);
    }
    void resetBoatCallback(const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response)
    {
      // Reset boat to initial state
      this->init_boat();

      // Publish transformation
      this->publish_transformation();

      // Publish odometry message
      this->publish_odometry();

      // Prevent unused parameter warning
      (void)request;

      // Send sucess response
      response->success = true;
      response->message = "Boat reseted";

      RCLCPP_INFO(this->get_logger(), "Boat reseted");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service;

    double x, y, psi, u, v, r;
    double deltat;
    double tau_u, tau_r;
    double tau_u_limit, tau_r_limit;
    double m_u, m_v, m_r, m_u_v;
    double d_u, d_v, d_r, d_u_u, d_v_v, d_r_r;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
  return 0;
}
