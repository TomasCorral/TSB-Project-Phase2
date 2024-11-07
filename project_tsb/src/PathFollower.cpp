#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <deque>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "project_tsb_msgs/msg/boat_position.hpp"
#include "project_tsb_msgs/msg/boat_reference.hpp"

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
class PathFollower : public rclcpp::Node
{
  public:
    PathFollower()
    : Node("Path_Follower")
    {
      // Declare parameters
      this->declare_parameter("deltat", 2.0);
      this->declare_parameter("default_speed", 1.0);
      this->declare_parameter("point_radius", 2.0);
      this->declare_parameter("look_ahead", 1.0);

      // Read sampling time
      deltat_ = this->get_parameter("deltat").as_double();

      // Read default speed and path radius
      desired_speed_ = this->get_parameter("default_speed").as_double();
      point_radius_ = this->get_parameter("point_radius").as_double();
      look_ahead_ = this->get_parameter("look_ahead").as_double();

      // Setup publishers and subscribers
      publisher_reference_ = this->create_publisher<project_tsb_msgs::msg::BoatReference>("boat_reference", 10);
      subscriber_path_ = this->create_subscription<nav_msgs::msg::Path>("path", 10, std::bind(&PathFollower::update_path, this, std::placeholders::_1));
      subscriber_state_ = this->create_subscription<project_tsb_msgs::msg::BoatPosition>("boat_state", 10, std::bind(&PathFollower::update_state, this, std::placeholders::_1));

      // Setup state update timer
      timer_ = this->create_wall_timer(std::chrono::duration<double>(deltat_), std::bind(&PathFollower::update_reference, this));

      RCLCPP_INFO(this->get_logger(), "Path Follower node started");
    }

  private:
    void update_reference()
    {
      // Check if state has been received
      if (!state_received_) {
        RCLCPP_INFO(this->get_logger(), "No boat state received yet, skipping.");
        return;
      }

      // If no path, add current position as the only path
      if (desired_path_.empty()) {
        desired_path_.clear(); //Just in case

        // Add current position as the only path point
        geometry_msgs::msg::Point current_position;
        current_position.x = x_;
        current_position.y = y_;
        desired_path_.push_back(current_position);
        RCLCPP_INFO(this->get_logger(), "Path empty, holding current position");
      }

      // Get next waypoint
      geometry_msgs::msg::Point next_waypoint;
      double distance;
      for (next_waypoint = desired_path_.front(), distance = sqrt(pow(next_waypoint.x - x_, 2) + pow(next_waypoint.y - y_, 2)); desired_path_.size() > 1; next_waypoint = desired_path_.front()) {
        distance = sqrt(pow(next_waypoint.x - x_, 2) + pow(next_waypoint.y - y_, 2));
        if (distance < point_radius_) {
          RCLCPP_INFO(this->get_logger(), "Waypoint reached");
          desired_path_.pop_front();
        } else {
          break;
        }
      }

      // Calculate desired yaw to reach point
      double teta_path = atan2(next_waypoint.y - y_, next_waypoint.x - x_);
      double x_los = next_waypoint.x + look_ahead_ * cos(teta_path);
      double y_los = next_waypoint.y + look_ahead_ * sin(teta_path);

      desired_yaw_ = atan2(y_los - y_, x_los - x_);
      
      RCLCPP_INFO(this->get_logger(), "Number of waypoints %d", (int)desired_path_.size());
      RCLCPP_INFO(this->get_logger(), "Distance to next waypoint %f", distance);

      if (distance < point_radius_ && desired_path_.size() == 1) {
        publish_reference(0.0, desired_yaw_);
        RCLCPP_INFO(this->get_logger(), "Desired Speed/Yaw: %f/%f", 0.0, desired_yaw_);
      } else {
        publish_reference(desired_speed_, desired_yaw_);
        RCLCPP_INFO(this->get_logger(), "Desired Speed/Yaw: %f/%f", desired_speed_, desired_yaw_);
      }

    }

    void publish_reference(double u, double yaw)
    {
      project_tsb_msgs::msg::BoatReference reference;
      reference.u = u;
      reference.yaw = radiansToDegrees(yaw);
      publisher_reference_->publish(reference);
    }

    void update_path(const nav_msgs::msg::Path::SharedPtr msg)
    {
      // Clear previous path
      desired_path_.clear();

      // Extract path points
      for (const auto & pose_stamped : msg->poses) {
        desired_path_.push_back(pose_stamped.pose.position);
      }

      RCLCPP_INFO(this->get_logger(), "Received new path with %d waypoints", (int)desired_path_.size());
    }

    void update_state(const project_tsb_msgs::msg::BoatPosition::SharedPtr msg)
    {
      // Extract current state from msg
      x_ = msg->x;
      y_ = msg->y;
      yaw_ = degreesToRadians(msg->yaw);
      state_received_ = true;

      RCLCPP_INFO(this->get_logger(), "Received new state: x = %f, y = %f, yaw = %f", x_, y_, yaw_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<project_tsb_msgs::msg::BoatReference>::SharedPtr publisher_reference_; 
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscriber_path_;
    rclcpp::Subscription<project_tsb_msgs::msg::BoatPosition>::SharedPtr subscriber_state_;

    // Boat State
    double x_, y_, yaw_;
    bool state_received_ = false;

    // Sampling time
    double deltat_;

    // Desired path, speed, radius, look-ahead
    double desired_speed_;
    double desired_yaw_;
    double point_radius_;
    double look_ahead_;
    std::deque<geometry_msgs::msg::Point> desired_path_;
    geometry_msgs::msg::Point next_waypoint_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
