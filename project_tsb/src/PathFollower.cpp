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
      this->declare_parameter("cruising_speed", 1.0);
      this->declare_parameter("minimum_speed", 0.2);
      this->declare_parameter("reach_radius", 5.0);
      this->declare_parameter("reach_radius_last", 1.0);
      this->declare_parameter("slowdown_distance", 10.0);
      this->declare_parameter("look_ahead", 1.0);

      // Read sampling time
      deltat_ = this->get_parameter("deltat").as_double();

      // Read default speed and path radius
      cruising_speed_ = this->get_parameter("cruising_speed").as_double();
      minimum_speed_ = this->get_parameter("minimum_speed").as_double();
      point_radius_ = this->get_parameter("reach_radius").as_double();
      point_radius_last_ = this->get_parameter("reach_radius_last").as_double();
      slowdown_distance_ = this->get_parameter("slowdown_distance").as_double();
      look_ahead_ = this->get_parameter("look_ahead").as_double();

      // Setup publishers and subscribers
      publisher_reference_ = this->create_publisher<project_tsb_msgs::msg::BoatReference>("boat_reference", 10);
      subscriber_path_ = this->create_subscription<nav_msgs::msg::Path>("path", 10, std::bind(&PathFollower::update_path, this, std::placeholders::_1));
      subscriber_state_ = this->create_subscription<project_tsb_msgs::msg::BoatPosition>("boat_state", 10, std::bind(&PathFollower::update_state, this, std::placeholders::_1));

      // Setup callback for live parameter updates
      param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&PathFollower::on_parameters_change, this, std::placeholders::_1));

      // Setup state update timer
      timer_ = this->create_wall_timer(std::chrono::duration<double>(deltat_), std::bind(&PathFollower::update_reference, this));

      RCLCPP_INFO(this->get_logger(), "Path Follower node started");
    }

  private:
    void update_reference()
    {
      // Check if state has been received
      if (!state_received_) {
        //RCLCPP_INFO(this->get_logger(), "No boat state received yet, skipping.");
        return;
      }
      if (!path_received_) {
        //RCLCPP_INFO(this->get_logger(), "No path received yet, skipping.");
        return;
      }
      if (desired_path_.empty()) {
        //RCLCPP_INFO(this->get_logger(), "Last waypoint reached, skipping.");
        return;
      }

      // Get next waypoint
      geometry_msgs::msg::Point next_waypoint = desired_path_.front();
      double distance = sqrt(pow(next_waypoint.x - x_, 2) + pow(next_waypoint.y - y_, 2));
      while ((distance < point_radius_ && desired_path_.size() > 1) || (distance < point_radius_last_ && desired_path_.size() == 1))  {
        RCLCPP_INFO(this->get_logger(), "Waypoint reached");
        desired_path_.pop_front();
        if (desired_path_.empty()) {
          RCLCPP_INFO(this->get_logger(), "Reached last waypoint");
          publish_reference(0.0, yaw_);
          return;
        }
        next_waypoint = desired_path_.front();
        distance = sqrt(pow(next_waypoint.x - x_, 2) + pow(next_waypoint.y - y_, 2));
      }

      // Calculate desired speed
      if (desired_path_.size() == 1) {
        desired_speed_ = cruising_speed_ * (distance / slowdown_distance_);
        desired_speed_ = std::max(desired_speed_, minimum_speed_);
        desired_speed_ = std::min(desired_speed_, cruising_speed_);
      } else {
        desired_speed_ = cruising_speed_;
      }

      // Calculate desired yaw
      double teta_path = atan2(next_waypoint.y - y_, next_waypoint.x - x_);
      double x_los = next_waypoint.x + look_ahead_ * cos(teta_path);
      double y_los = next_waypoint.y + look_ahead_ * sin(teta_path);
      desired_yaw_ = atan2(y_los - y_, x_los - x_);
      

      //RCLCPP_INFO(this->get_logger(), "Current state: x = %f, y = %f, yaw = %f", x_, y_, radiansToDegrees(yaw_));
      //RCLCPP_INFO(this->get_logger(), "Desired Speed/Yaw: %f/%f", desired_speed_, desired_yaw_);
      //RCLCPP_INFO(this->get_logger(), "Distance to next waypoint %f", distance);
      //RCLCPP_INFO(this->get_logger(), "Number of waypoints %d", (int)desired_path_.size());

      // Publish reference
      publish_reference(desired_speed_, desired_yaw_);

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

      // Set path received flag
      path_received_ = true;

      RCLCPP_INFO(this->get_logger(), "Received new path with %d waypoints", (int)desired_path_.size());
      
      // Update reference right away instead of waiting for timer
      update_reference();
    }

    void update_state(const project_tsb_msgs::msg::BoatPosition::SharedPtr msg)
    {
      // Extract current state from msg
      x_ = msg->x;
      y_ = msg->y;
      yaw_ = degreesToRadians(msg->yaw);

      // Set state received flag
      state_received_ = true;

      //RCLCPP_INFO(this->get_logger(), "Received new state: x = %f, y = %f, yaw = %f", x_, y_, yaw_);
    }
    rcl_interfaces::msg::SetParametersResult on_parameters_change(const std::vector<rclcpp::Parameter> &parameters)
    {
      // Update parameters
      for (const auto &parameter : parameters) {
        if (parameter.get_name() == "deltat") {
          deltat_ = parameter.as_double();
          timer_->cancel();
          timer_ = this->create_wall_timer(std::chrono::duration<double>(deltat_), std::bind(&PathFollower::update_reference, this));
        } else if (parameter.get_name() == "cruising_speed") {
          cruising_speed_ = parameter.as_double();
        } else if (parameter.get_name() == "minimum_speed") {
          minimum_speed_ = parameter.as_double();
        } else if (parameter.get_name() == "reach_radius") {
          point_radius_ = parameter.as_double();
        } else if (parameter.get_name() == "reach_radius_last") {
          point_radius_last_ = parameter.as_double();
        } else if (parameter.get_name() == "slowdown_distance") {
          slowdown_distance_ = parameter.as_double();
        } else if (parameter.get_name() == "look_ahead") {
          look_ahead_ = parameter.as_double();
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
    rclcpp::Publisher<project_tsb_msgs::msg::BoatReference>::SharedPtr publisher_reference_; 
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscriber_path_;
    rclcpp::Subscription<project_tsb_msgs::msg::BoatPosition>::SharedPtr subscriber_state_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // Boat State
    double x_, y_, yaw_;
    bool state_received_ = false;

    // Sampling time
    double deltat_;

    // Speeds, path and path following parameters
    bool path_received_ = false;
    double cruising_speed_, minimum_speed_;
    double desired_speed_, desired_yaw_;
    double point_radius_, point_radius_last_, slowdown_distance_;
    double look_ahead_;
    std::deque<geometry_msgs::msg::Point> desired_path_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
