#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <vector>
#include <sstream>

class TrajectoryReaderPublisher : public rclcpp::Node {
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
    TrajectoryReaderPublisher() : Node("trajectory_reader_publisher") {
        trajectory_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("trajectory_markers", 10);
        
        // Initialize TF2 Buffer and Listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "Trajectory Reader and Publisher Node started.");
    }

    void read_and_publish_trajectory(const std::string &filename, const std::string &input_frame) {
        std::ifstream file(filename);
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filename.c_str());
            return;
        }

        visualization_msgs::msg::MarkerArray marker_array;
        std::string line;
        int id = 0;
        rclcpp::Time now = this->get_clock()->now();

        // Get transform from input_frame to "odom"
        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_->lookupTransform("odom", input_frame, tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform from %s to odom: %s", input_frame.c_str(), ex.what());
            return;
        }

        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string val;
            std::vector<double> values;

            while (std::getline(ss, val, ',')) {
                values.push_back(std::stod(val));
            }

            if (values.size() < 3) continue; // Ensure valid data

            // Create a geometry point in the saved frame
            geometry_msgs::msg::PointStamped point_in, point_out;
            point_in.header.frame_id = input_frame;
            point_in.header.stamp = now;
            point_in.point.x = values[0];
            point_in.point.y = values[1];
            point_in.point.z = values[2];

            // Transform to "odom" frame
            try {
                tf2::doTransform(point_in, point_out, transform_stamped);
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "TF2 transform failed: %s", ex.what());
                continue;
            }

            // Create a marker for visualization
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "odom"; // Now in odom frame
            marker.header.stamp = now;
            marker.ns = "trajectory";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.1;
            marker.scale.y = 0.1;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.a = 1.0;
            marker.pose.position = point_out.point;

            marker_array.markers.push_back(marker);
        }

        trajectory_pub_->publish(marker_array);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryReaderPublisher>();

    std::string filename = "trajectory.csv"; // Update with the actual file name
    std::string input_frame = "map"; // Set the actual frame where the trajectory was recorded

    node->read_and_publish_trajectory(filename, input_frame);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

