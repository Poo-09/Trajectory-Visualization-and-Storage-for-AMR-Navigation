#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose.hpp"
#include <fstream>
#include <sstream>
#include <vector>

// Reuse the TrajectoryPoint structure defined earlier
struct TrajectoryPoint {
  geometry_msgs::msg::Pose pose;
  double timestamp;
};

class TrajectoryReaderPublisher : public rclcpp::Node
{
public:
  TrajectoryReaderPublisher()
  : Node("trajectory_reader_publisher"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/transformed_trajectory", 10);
    
    // Get parameter for trajectory file name (default: "trajectory.csv")
    this->declare_parameter<std::string>("trajectory_file", "trajectory.csv");
    this->get_parameter("trajectory_file", trajectory_file_);

    // Read and publish the trajectory
    readAndPublishTrajectory(trajectory_file_);
  }

private:
  void readAndPublishTrajectory(const std::string &filename)
  {
    // Load trajectory data from file (implement CSV reading)
    std::vector<TrajectoryPoint> trajectory = loadTrajectoryFromFile(filename);
    visualization_msgs::msg::MarkerArray marker_array;
    
    // For each point, perform transformation to 'odom' frame using tf2
    for (size_t i = 0; i < trajectory.size(); ++i) {
      geometry_msgs::msg::Pose transformed_pose;
      try {
        // Transform pose: adjust the parameters as needed. This is an example.
        transformed_pose = tf_buffer_.transform(trajectory[i].pose, "odom", rclcpp::Duration::from_seconds(0.1));
      } catch (const tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
        continue;
      }
      // Create a marker for each transformed point
      // Set marker properties (id, type, position, scale, color, etc.)
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "odom";
      marker.header.stamp = this->now();
      marker.ns = "trajectory";
      marker.id = i;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = transformed_pose;
      marker.scale.x = 0.05;
      marker.scale.y = 0.05;
      marker.scale.z = 0.05;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      
      marker_array.markers.push_back(marker);
    }
    marker_pub_->publish(marker_array);
  }

  std::vector<TrajectoryPoint> loadTrajectoryFromFile(const std::string &filename)
  {
    std::vector<TrajectoryPoint> data;
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open file: %s", filename.c_str());
      return data;
    }
    
    std::string line;
    // Skip CSV header
    std::getline(file, line);
    while (std::getline(file, line)) {
      std::istringstream ss(line);
      TrajectoryPoint point;
      char comma;
      ss >> point.timestamp >> comma
         >> point.pose.position.x >> comma
         >> point.pose.position.y >> comma
         >> point.pose.position.z >> comma
         >> point.pose.orientation.x >> comma
         >> point.pose.orientation.y >> comma
         >> point.pose.orientation.z >> comma
         >> point.pose.orientation.w;
      data.push_back(point);
    }
    file.close();
    return data;
  }
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string trajectory_file_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryReaderPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

