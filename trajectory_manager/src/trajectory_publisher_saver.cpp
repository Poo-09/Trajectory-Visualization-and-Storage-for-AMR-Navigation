#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "trajectory_manager/srv/save_trajectory.hpp"
#include <deque>
#include <vector>
#include <fstream>
#include <sstream>
#include <string>

struct TrajectoryPoint {
  rclcpp::Time timestamp;
  geometry_msgs::msg::Point point;
};

class TrajectoryPublisherSaver : public rclcpp::Node
{
public:
  TrajectoryPublisherSaver() : Node("trajectory_publisher_saver")
  {
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&TrajectoryPublisherSaver::odom_callback, this, std::placeholders::_1));

    marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);

    save_service_ = this->create_service<trajectory_manager::srv::SaveTrajectory>(
        "save_trajectory",
        std::bind(&TrajectoryPublisherSaver::save_trajectory_callback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Trajectory Publisher and Saver Node started.");
  }

private:
  std::deque<TrajectoryPoint> trajectory_points_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Service<trajectory_manager::srv::SaveTrajectory>::SharedPtr save_service_;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    TrajectoryPoint tp;
    tp.timestamp = this->now();
    tp.point = msg->pose.pose.position;
    trajectory_points_.push_back(tp);

    publish_marker_array();
  }

  void publish_marker_array()
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "odom";
    marker.header.stamp = this->now();
    marker.ns = "trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.pose.orientation.w = 1.0;

    for (const auto &pt : trajectory_points_) {
      marker.points.push_back(pt.point);
    }
    marker_array.markers.push_back(marker);
    marker_publisher_->publish(marker_array);
  }

  void save_trajectory_callback(
      const std::shared_ptr<trajectory_manager::srv::SaveTrajectory::Request> request,
      std::shared_ptr<trajectory_manager::srv::SaveTrajectory::Response> response)
  {
    rclcpp::Time now = this->now();
    double duration = request->duration;
    std::vector<TrajectoryPoint> points_to_save;

    // Filter trajectory points based on time duration
    for (const auto &pt : trajectory_points_) {
      if ((now - pt.timestamp).seconds() <= duration) {
        points_to_save.push_back(pt);
      }
    }

    std::string filename = request->filename;
    std::ofstream file(filename);
    if (!file.is_open()) {
      response->success = false;
      response->message = "Failed to open file: " + filename;
      return;
    }

    // Save in different formats based on filename extension
    if (filename.find(".json") != std::string::npos) {
      file << "{\n  \"trajectory\": [\n";
      for (size_t i = 0; i < points_to_save.size(); ++i) {
        file << "    { \"x\": " << points_to_save[i].point.x
             << ", \"y\": " << points_to_save[i].point.y
             << ", \"timestamp\": " << points_to_save[i].timestamp.seconds() << " }";
        if (i < points_to_save.size() - 1) {
          file << ",";
        }
        file << "\n";
      }
      file << "  ]\n}";
    } else if (filename.find(".yaml") != std::string::npos) {
      file << "trajectory:\n";
      for (const auto &pt : points_to_save) {
        file << "  - { x: " << pt.point.x << ", y: " << pt.point.y
             << ", timestamp: " << pt.timestamp.seconds() << " }\n";
      }
    } else {
      file << "x,y,timestamp\n";
      for (const auto &pt : points_to_save) {
        file << pt.point.x << "," << pt.point.y << "," << pt.timestamp.seconds() << "\n";
      }
    }

    file.close();
    response->success = true;
    response->message = "Trajectory saved successfully to " + filename;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryPublisherSaver>());
  rclcpp::shutdown();
  return 0;
}

