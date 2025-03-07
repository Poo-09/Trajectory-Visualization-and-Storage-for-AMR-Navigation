#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "amr_trajectory/srv/save_trajectory.hpp"
#include <vector>
#include <fstream>
#include <chrono>

// Structure to store pose and timestamp
struct TrajectoryPoint {
  geometry_msgs::msg::Pose pose;
  rclcpp::Time timestamp;
};

class TrajectoryPublisherSaver : public rclcpp::Node
{
public:
  TrajectoryPublisherSaver() : Node("trajectory_publisher_saver")
  {
    // Subscribe to the robot's odometry topic
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&TrajectoryPublisherSaver::odomCallback, this, std::placeholders::_1));

    // Publisher for MarkerArray (for visualization in RViz2)
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/trajectory_marker", 10);

    // Service to save the trajectory data to file
    save_service_ = this->create_service<amr_trajectory::srv::SaveTrajectory>(
      "save_trajectory",
      std::bind(&TrajectoryPublisherSaver::saveTrajectoryCallback, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Trajectory Publisher and Saver Node started.");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Record current pose and timestamp
    trajectory_.push_back({msg->pose.pose, this->now()});
    
    // (Optional) Create MarkerArray from trajectory_ and publish for visualization
    visualization_msgs::msg::MarkerArray marker_array;
    // Populate marker_array with markers corresponding to stored trajectory points
    // [You would typically create markers with unique IDs and appropriate geometry settings]
    
    marker_pub_->publish(marker_array);
  }

  void saveTrajectoryCallback(
    const std::shared_ptr<amr_trajectory::srv::SaveTrajectory::Request> request,
    std::shared_ptr<amr_trajectory::srv::SaveTrajectory::Response> response)
  {
    auto current_time = this->now();
    std::vector<TrajectoryPoint> filtered;

    // Filter trajectory: only include points within the requested duration
    for (const auto &point : trajectory_) {
      if ((current_time - point.timestamp).seconds() <= request->duration) {
        filtered.push_back(point);
      }
    }

    // Serialize filtered trajectory to file based on the filename extension
    // Here you would implement serialization (e.g., JSON using nlohmann/json)
    bool write_success = writeTrajectoryToFile(request->filename, filtered);

    response->success = write_success;
    response->message = write_success ? "Trajectory saved successfully." : "Failed to save trajectory.";
  }
  
  bool writeTrajectoryToFile(const std::string &filename, const std::vector<TrajectoryPoint> &data)
  {
    // Simple example: Write data to a CSV file
    std::ofstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s", filename.c_str());
      return false;
    }
    // CSV Header
    file << "timestamp,x,y,z,orientation_x,orientation_y,orientation_z,orientation_w\n";
    for (const auto &point : data) {
      file << point.timestamp.seconds() << ","
           << point.pose.position.x << ","
           << point.pose.position.y << ","
           << point.pose.position.z << ","
           << point.pose.orientation.x << ","
           << point.pose.orientation.y << ","
           << point.pose.orientation.z << ","
           << point.pose.orientation.w << "\n";
    }
    file.close();
    return true;
  }

  // Member variables
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Service<amr_trajectory::srv::SaveTrajectory>::SharedPtr save_service_;
  std::vector<TrajectoryPoint> trajectory_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TrajectoryPublisherSaver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

