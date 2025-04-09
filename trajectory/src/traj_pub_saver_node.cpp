// Imports
#include "trajectory/srv/save_trajectory.hpp"
#include <chrono>
#include <fstream>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

// Struct for keeping track of Trajectory
struct TimedPose {
  double x, y, timestamp;
};

class TrajectorySaverNode : public rclcpp::Node {
public:
  TrajectorySaverNode() : Node("trajectory_saver_node") {
    this->declare_parameter<std::string>("topic", "/pub_trajectory_markers");

    publish_topic = this->get_parameter("topic").as_string();
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&TrajectorySaverNode::odom_callback, this, _1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        publish_topic, 10);

    save_srv_ = this->create_service<trajectory::srv::SaveTrajectory>(
        "save_trajectory",
        std::bind(&TrajectorySaverNode::save_callback, this, _1, _2));

    timer_ = this->create_wall_timer(
        1s, std::bind(&TrajectorySaverNode::publish_markers, this));

    RCLCPP_INFO(this->get_logger(), "Trajectory saver node running.");
  }

private:
  std::vector<TimedPose> trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Service<trajectory::srv::SaveTrajectory>::SharedPtr save_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string file_name, file_format, publish_topic;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    TimedPose pose;
    pose.x = msg->pose.pose.position.x;
    pose.y = msg->pose.pose.position.y;
    pose.timestamp = this->now().seconds();
    trajectory_.push_back(pose);
  }

  void publish_markers() {
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    for (const auto &pose : trajectory_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "odom";
      marker.header.stamp = this->now();
      marker.ns = "trajectory";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = marker.ADD;
      marker.pose.position.x = pose.x;
      marker.pose.position.y = pose.y;
      marker.pose.position.z = 0.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      array.markers.push_back(marker);
    }
    marker_pub_->publish(array);
  }

  void save_callback(
      const std::shared_ptr<trajectory::srv::SaveTrajectory::Request> request,
      std::shared_ptr<trajectory::srv::SaveTrajectory::Response> response) {
    double now = this->now().seconds();
    std::vector<TimedPose> filtered;
    for (const auto &pose : trajectory_) {
      if (now - pose.timestamp <= request->duration) {
        filtered.push_back(pose);
      }
    }
    RCLCPP_INFO(this->get_logger(), "Saving trajectory for %.2f seconds",
                request->duration);
    try {
      std::ofstream file(request->filename);
      if (!file.is_open()) {
        throw std::runtime_error("Cannot open file.");
      }

      if (request->format == "csv") {
        file << "x,y,timestamp\n";
        for (const auto &pose : filtered) {
          file << pose.x << "," << pose.y << "," << pose.timestamp << "\n";
        }
      } else if (request->format == "json") {
        file << "[\n";
        for (size_t i = 0; i < filtered.size(); ++i) {
          file << "  {\"x\": " << filtered[i].x << ", \"y\": " << filtered[i].y
               << ", \"timestamp\": " << filtered[i].timestamp << "}";
          if (i != filtered.size() - 1)
            file << ",";
          file << "\n";
        }
        file << "]";
      } else if (request->format == "yaml") {
        for (const auto &pose : filtered) {
          file << "- x: " << pose.x << "\n";
          file << "  y: " << pose.y << "\n";
          file << "  timestamp: " << pose.timestamp << "\n";
        }
      } else {
        throw std::runtime_error("Unsupported format: " + request->format);
      }

      response->success = true;
      response->message = "Saved successfully.";
      RCLCPP_INFO(this->get_logger(), "Trajectory saved!!!!");
    } catch (const std::exception &e) {
      response->success = false;
      response->message = e.what();
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectorySaverNode>());
  rclcpp::shutdown();
  return 0;
}
