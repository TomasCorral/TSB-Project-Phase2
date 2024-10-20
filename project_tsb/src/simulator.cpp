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
class Simulator : public rclcpp::Node
{
  public:
    Simulator()
    : Node("simulator")
    {
      // TODO: Get initial robot state from parameters
      this->x = 0.0;
      this->y = 0.0;
      this->psi = 0;
      this->u = 0.0;
      this->v = 0.0;
      this->r = 0.0;

      
      // TODO: Get time constant from parameter
      this->deltat = 2; // 2s

      // Initialize forces applied
      this->tau_u = 1.0;
      this->tau_v = 0.0; //This one is always zero
      this->tau_r = 0.05;

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

      // Initialize internal timer used for state update
      timer_ = this->create_wall_timer(std::chrono::duration<double>(this->deltat), std::bind(&Simulator::update_state, this));

      // Initialize transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      // Setup publishers and subscribers
      publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("topic3", 10); //Publisher used to publish the odometry
      subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("topic2", 10, std::bind(&Simulator::update_forces, this, std::placeholders::_1));

      // Publish transformation
      this->publish_transformation();

      // Publish the odometry message with the new state
      this->publish_odometry();
    }

  private:
    void update_state()
    {
      // Update robot state
      // TODO: Trocar dinamica com cinematica? Probably not, perguntar
      this->u += (this->tau_u + this->m_v*this->v*this->r - this->d_u*this->u - this->d_u_u*this->u*abs(this->u)) * this->deltat / this->m_u;
      this->v += (this->tau_v -this->m_u*this->u*this->r - this->d_v*this->v - this->d_v_v*this->u*abs(this->u) + this->tau_v) * this->deltat / this->m_v;
      this->r += (this->tau_r + this->m_u_v*this->u*this->v - this->d_r*this->r - this->d_r_r*this->r*abs(this->r)) * this->deltat / this->m_r;

      this->x += (this->u*cos(this->psi) - this->v*sin(this->psi)) * this->deltat;
      this->y += (this->u*sin(this->psi) + this->v*cos(this->psi)) * this->deltat;
      this->psi += this->r * this->deltat;
      

      // Publish transformation
      this->publish_transformation();

      // Publish the odometry message with the new state
      this->publish_odometry();


      //RCLCPP_INFO(this->get_logger(), "Publishing new robot position");
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
      odometry.pose.pose.orientation = tf2::toMsg(q);
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
      // Update forces applied to the robot
      this->tau_u = msg->data[0];
      this->tau_r = msg->data[1];
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double x, y, psi, u, v, r;
    double deltat;
    double tau_u, tau_v, tau_r;
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
