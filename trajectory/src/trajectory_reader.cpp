#include "trajectory/json.hpp"
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;
using json = nlohmann::json;

struct PoseData {
  double x, y, timestamp;
};

class TrajectoryReaderNode : public rclcpp::Node {
public:
  TrajectoryReaderNode() : Node("trajectory_reader_node") {
    this->declare_parameter<std::string>("file_path",
                                         "my_path.json");
    this->declare_parameter<std::string>("format", "json");
    this->declare_parameter<std::string>("topic", "/read_trajectory_markers");
    pub_marker = this->get_parameter("topic").as_string();
    file_path = this->get_parameter("file_path").as_string();
    file_format = this->get_parameter("format").as_string();

    publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        pub_marker, 10);
    timer_ = this->create_wall_timer(
        1s, std::bind(&TrajectoryReaderNode::publish_markers, this));

    if (load_trajectory()) {
      RCLCPP_INFO(this->get_logger(),
                  "Trajectory loaded and ready to publish.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to load trajectory.");
    }
  }

private:
  std::vector<PoseData> trajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string file_path, file_format, pub_marker;
  bool load_trajectory() {
    RCLCPP_INFO(this->get_logger(), "Loading trajectory from: %s (format: %s)",
                file_path.c_str(), file_format.c_str());

    try {
      std::ifstream file(file_path);
      if (!file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open file: %s", file_path.c_str());
        return false;
      }

      if (file_format == "csv") {
        std::string line;
        std::getline(file, line); // skip header
        while (std::getline(file, line)) {
          std::stringstream ss(line);
          std::string x_str, y_str, t_str;
          std::getline(ss, x_str, ',');
          std::getline(ss, y_str, ',');
          std::getline(ss, t_str, ',');
          trajectory_.push_back(
              {std::stod(x_str), std::stod(y_str), std::stod(t_str)});
        }
      } else if (file_format == "json") {
        json j;
        file >> j;

        if (!j.is_array()) {
          RCLCPP_ERROR(this->get_logger(), "JSON root is not an array.");
          return false;
        }

        for (const auto &item : j) {
          if (item.contains("x") && item.contains("y") &&
              item.contains("timestamp")) {
            trajectory_.push_back({item["x"].get<double>(),
                                   item["y"].get<double>(),
                                   item["timestamp"].get<double>()});
          } else {
            RCLCPP_WARN(this->get_logger(), "Skipping invalid JSON entry.");
          }
        }
      } else if (file_format == "yaml") {
        YAML::Node yaml = YAML::Load(file);
        for (const auto &node : yaml) {
          trajectory_.push_back({node["x"].as<double>(), node["y"].as<double>(),
                                 node["timestamp"].as<double>()});
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Unsupported format: %s",
                     file_format.c_str());
        return false;
      }

      return true;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Exception while loading trajectory: %s",
                   e.what());
      return false;
    }
  }

  void publish_markers() {
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    for (const auto &pose : trajectory_) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "odom";
      marker.header.stamp = this->now();
      marker.ns = "read_trajectory";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = marker.ADD;
      marker.pose.position.x = pose.x;
      marker.pose.position.y = pose.y;
      marker.pose.position.z = 0.0;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 1.0;
      array.markers.push_back(marker);
    }
    publisher_->publish(array);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryReaderNode>());
  rclcpp::shutdown();
  return 0;
}
