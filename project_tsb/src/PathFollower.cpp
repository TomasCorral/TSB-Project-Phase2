#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <deque>
#include <string>
#include <spline.h>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "project_tsb_msgs/msg/boat_state.hpp"
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

// Distance between two points
double calculateDistance(double point1_x, double point1_y, double point2_x, double point2_y) {
  return sqrt(pow(point2_x - point1_x, 2) + pow(point2_y - point1_y, 2));
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
      this->declare_parameter("reach_radius", 10.0);
      this->declare_parameter("reach_radius_last", 1.0);
      this->declare_parameter("slowdown_distance", 6.0);
      this->declare_parameter("look_ahead", 8.0);
      this->declare_parameter("generate_spline", false);
      this->declare_parameter("min_spacing", 10.0);

      // Read sampling time
      deltat_ = this->get_parameter("deltat").as_double();

      // Read default speed and path radius
      cruising_speed_ = this->get_parameter("cruising_speed").as_double();
      minimum_speed_ = this->get_parameter("minimum_speed").as_double();
      point_radius_ = this->get_parameter("reach_radius").as_double();
      point_radius_last_ = this->get_parameter("reach_radius_last").as_double();
      slowdown_distance_ = this->get_parameter("slowdown_distance").as_double();
      look_ahead_ = this->get_parameter("look_ahead").as_double();
      generate_spline_ = this->get_parameter("generate_spline").as_bool();
      min_spacing_ = this->get_parameter("min_spacing").as_double();

      // Setup publishers and subscribers
      publisher_reference_ = this->create_publisher<project_tsb_msgs::msg::BoatReference>("boat_reference", 10);
      subscriber_path_ = this->create_subscription<nav_msgs::msg::Path>("path", 10, std::bind(&PathFollower::update_path, this, std::placeholders::_1));
      subscriber_state_ = this->create_subscription<project_tsb_msgs::msg::BoatState>("boat_state", 10, std::bind(&PathFollower::update_state, this, std::placeholders::_1));
      publisher_path_generated_ = this->create_publisher<geometry_msgs::msg::Point>("path_generated", 10);

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
      if (path_finished_) {
        //RCLCPP_INFO(this->get_logger(), "Last waypoint reached, skipping.");
        return;
      }

      // Get next waypoint
      double distance = calculateDistance(next_waypoint_.x, next_waypoint_.y, x_, y_);
      if (distance < point_radius_ && desired_path_.size() > 0) {
        previous_waypoint_ = next_waypoint_;
        next_waypoint_ = desired_path_.front();
        desired_path_.pop_front();
        RCLCPP_INFO(this->get_logger(), "Waypoint reached");
      }

      if (distance < point_radius_last_ && desired_path_.size() == 0) {
        desired_path_.pop_front();
        RCLCPP_INFO(this->get_logger(), "Reached last waypoint");
        publish_reference(0.0, yaw_);
        path_finished_ = true;
        return;
      }

      // Calculate desired speed
      if (desired_path_.size() == 0) {
        desired_speed_ = cruising_speed_ * (distance / slowdown_distance_);
        desired_speed_ = std::max(desired_speed_, minimum_speed_);
        desired_speed_ = std::min(desired_speed_, cruising_speed_);
      } else {
        desired_speed_ = cruising_speed_;
      }

      // Calculate desired yaw
      double delta_x = next_waypoint_.x - previous_waypoint_.x;
      double delta_y = next_waypoint_.y - previous_waypoint_.y;
      double s = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
      double e_x = (delta_x) / s;
      double e_y = (delta_y) / s;
      double x_diff = x_ - previous_waypoint_.x;
      double y_diff = y_ - previous_waypoint_.y;
      double s_proj = e_x * x_diff + e_y * y_diff;
      double s_los = std::min(s_proj + look_ahead_, s);
      double x_los = previous_waypoint_.x + s_los * e_x;
      double y_los = previous_waypoint_.y + s_los * e_y;
      desired_yaw_ = atan2(y_los - y_, x_los - x_);

      //RCLCPP_INFO(this->get_logger(), "Current state: x = %f, y = %f, yaw = %f", x_, y_, radiansToDegrees(yaw_));
      //RCLCPP_INFO(this->get_logger(), "Desired Speed/Yaw: %f/%f", desired_speed_, radiansToDegrees(desired_yaw_));
      //RCLCPP_INFO(this->get_logger(), "Distance to next waypoint %f", distance);
      //RCLCPP_INFO(this->get_logger(), "Previous Waypoint: x = %f, y = %f Next waypoint: x = %f, y = %f", previous_waypoint_.x, previous_waypoint_.y, next_waypoint_.x, next_waypoint_.y);

      // Publish reference
      publish_reference(desired_speed_, desired_yaw_);

    }

    void publish_reference(double u, double yaw)
    {
      project_tsb_msgs::msg::BoatReference reference;
      reference.surge = u;
      reference.yaw = radiansToDegrees(yaw);
      publisher_reference_->publish(reference);
    }

    void update_path(const nav_msgs::msg::Path::SharedPtr msg)
    {
      // Clear previous path
      desired_path_.clear();

      // Extract waypoints from msg
      std::deque<geometry_msgs::msg::Point> waypoints;
      for (const auto & pose_stamped : msg->poses) {
        waypoints.push_back(pose_stamped.pose.position);
      }
      RCLCPP_INFO(this->get_logger(), "Received new path with %ld waypoints", waypoints.size());

      // Generate spline if flag is set
      if (generate_spline_) {
        // Create spline
        std::vector<double> x, y, t;
        double dist = 0.0;
        for (size_t i = 0; i < waypoints.size(); i++) {
          if (i > 0) {
            dist += calculateDistance(waypoints[i].x, waypoints[i].y, waypoints[i-1].x, waypoints[i-1].y);
          }
          x.push_back(waypoints[i].x);
          y.push_back(waypoints[i].y);
          t.push_back(dist);
        }

        tk::spline sx, sy;
        sx.set_points(t, x);
        sy.set_points(t, y);

        // Generate path points and then filter out
        double t_step = 0.1;
        double distance;
        std::vector<geometry_msgs::msg::Point> generated_points;
        for (double ti = 0.0; ti < t.back(); ti += t_step) {
          geometry_msgs::msg::Point point;
          point.x = sx(ti);
          point.y = sy(ti);
          generated_points.push_back(point);
        }

        desired_path_.push_back(generated_points.front());
        for (size_t i = 1; i < generated_points.size(); i++) {
          distance = calculateDistance(generated_points[i].x, generated_points[i].y, desired_path_.back().x, desired_path_.back().y);
          if (distance > min_spacing_) {
            desired_path_.push_back(generated_points[i]);
          }
        }

        RCLCPP_INFO(this->get_logger(), "Generated path with %d waypoints", (int)desired_path_.size());

      } else {
        desired_path_ = waypoints;
      }
      
      // Publish path for visualization
      for (size_t i = 0; i < desired_path_.size(); i++) {
        geometry_msgs::msg::Point msg;
        msg.x = desired_path_[i].x;
        msg.y = desired_path_[i].y;
        publisher_path_generated_->publish(msg);
        std::this_thread::sleep_for(std::chrono::milliseconds(50)); // Prevent spamming
        //RCLCPP_INFO(this->get_logger(), "Waypoint %ld: x = %f, y = %f", i, desired_path_[i].x, desired_path_[i].y);
      }

      // Set flags
      path_received_ = true;
      path_finished_ = false;

      // Set next and preivous waypoints
      previous_waypoint_ = desired_path_.front();
      desired_path_.pop_front();
      next_waypoint_ = desired_path_.front();
      desired_path_.pop_front();

      // Update reference right away instead of waiting for timer
      update_reference();
    }

    void update_state(const project_tsb_msgs::msg::BoatState::SharedPtr msg)
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
        }  else if (parameter.get_name() == "look_ahead") {
          look_ahead_ = parameter.as_double();
        } else if (parameter.get_name() == "generate_spline") {
          generate_spline_ = parameter.as_bool();
        } else if (parameter.get_name() == "min_spacing") {
          min_spacing_ = parameter.as_double();
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
    rclcpp::Subscription<project_tsb_msgs::msg::BoatState>::SharedPtr subscriber_state_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_path_generated_;

    // Boat State
    double x_, y_, yaw_;
    bool state_received_ = false;

    // Update rate
    double deltat_;

    // Path flags
    bool path_received_ = false;
    bool path_finished_ = false;
    
    // Path following output
    double desired_speed_, desired_yaw_;

    // Path following configs
    double cruising_speed_, minimum_speed_;
    double point_radius_, point_radius_last_, slowdown_distance_;
    double look_ahead_;
    bool generate_spline_;
    double min_spacing_;

    // Path and waypoints
    std::deque<geometry_msgs::msg::Point> desired_path_;
    geometry_msgs::msg::Point next_waypoint_;
    geometry_msgs::msg::Point previous_waypoint_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
