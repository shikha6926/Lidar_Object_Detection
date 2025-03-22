#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <filesystem>
#include <vector>

namespace fs = std::filesystem;

class PointCloudPublisher : public rclcpp::Node {

public:
    PointCloudPublisher() : Node("point_cloud_publisher"), current_file_idx_(0) {

        // Create publisher for Pointcloud2
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("my_pcd", 10);

        // Get the list of PCD files from the directory
        std::string pcd_directory = "/home/shikha/Workspace/Sensor_Fusion_Course/repos/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/";
        for (const auto &entry : fs::directory_iterator(pcd_directory)) {
            if (entry.path().extension() == ".pcd") {
                pcd_files_.push_back(entry.path().string());
            }
        }

        // Check if there are any PCD files
        if (pcd_files_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No PCD files found in the directory");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Found %lu PCD files", pcd_files_.size());

        // Timer to publish pointcloud at 1 Hz
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PointCloudPublisher::publishPointCloud, this)
        );
    }

private:
    void publishPointCloud() {
        // Check if we have exhausted all PCD files
        if (current_file_idx_ >= pcd_files_.size()) {
            current_file_idx_ = 0;  // Loop back to the first file
        }

        // Load the next PCD file
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_files_[current_file_idx_], *cloud) == -1) {
            RCLCPP_ERROR(this->get_logger(), "Could not load the PCD file: %s", pcd_files_[current_file_idx_].c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %d data_points from PCD file: %s", cloud->width * cloud->height, pcd_files_[current_file_idx_].c_str());

        // Convert PCL to ROS2 msg
        pcl::toROSMsg(*cloud, output_);
        output_.header.stamp = this->now();
         // Set the frame of reference
        output_.header.frame_id = "map"; 

        // Publish the point cloud
        pub_->publish(output_);

        // Move to the next PCD file
        current_file_idx_++;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    sensor_msgs::msg::PointCloud2 output_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<std::string> pcd_files_; 
    size_t current_file_idx_; 
};

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
}
