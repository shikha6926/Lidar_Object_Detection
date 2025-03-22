#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <filesystem>

#include "lidar_object_detection/processPointClouds.h"


namespace fs = std::filesystem;

class LidarNode : public rclcpp::Node
{
public:
    LidarNode() : Node("lidar_node"), current_pcd_index_(0)
    {
        pcl_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_cloud", 10);
        cluster_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_cloud", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("bounding_boxes", 10);

        timer_ = this->create_wall_timer(std::chrono::seconds(2), std::bind(&LidarNode::processNextPCD, this));

        std::string pcd_directory = "/home/shikha/Workspace/Sensor_Fusion_Course/repos/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/";
        for (const auto &entry : fs::directory_iterator(pcd_directory)) {
            if (entry.path().extension() == ".pcd") {
                pcd_files_.push_back(entry.path().string());
            }
        }

        if (pcd_files_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No PCD files found in the directory");
            rclcpp::shutdown();
        }
    }

private:
    
    void processNextPCD()
    {

        if (current_pcd_index_ >= pcd_files_.size()) {
            RCLCPP_INFO(this->get_logger(), "All PCD files processed. Restarting from the beginning...");
            current_pcd_index_ = 0;  // Reset index to start over
        }    

        std::string pcd_file = pcd_files_[current_pcd_index_++];
        RCLCPP_INFO(this->get_logger(), "Processing PCD file: %s", pcd_file.c_str());

        pPointCloudProcessor  = new ProcessPointClouds<pcl::PointXYZI>();

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud = pPointCloudProcessor->loadPcd(pcd_file);
        if (!cloud || cloud->empty()) {
            RCLCPP_WARN(this->get_logger(), "Failed to load PCD file or it's empty.");
            return;
        }

        inputCloud_ = pPointCloudProcessor->FilterCloud(cloud, 0.2f, Eigen::Vector4f(-10, -5, -5, 1), Eigen::Vector4f(30, 6, 5, 1));
        processPointCloud();
    }

    void processPointCloud()
    {
        // Step 1: Plane Segmentation (RANSAC)
        auto segmentCloud = pPointCloudProcessor->Ransac3D(inputCloud_, 300, 0.2);

        // Step 2: Object Clustering
        auto cloudClusters = pPointCloudProcessor->ClusteringFromScratch(segmentCloud.first, 0.4, 10, 600);

        // Step 3: Publish segmented point cloud (road surface)
        publishPointCloud(segmentCloud.second, "filtered_cloud");

        // Step 4: Publish clustered objects and bounding boxes
        publishClusters(cloudClusters);
    }

    void publishPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::string& topic)
    {
        sensor_msgs::msg::PointCloud2 ros_cloud;
        pcl::toROSMsg(*cloud, ros_cloud);
        ros_cloud.header.frame_id = "map";

        if (topic == "filtered_cloud")
            pcl_publisher_->publish(ros_cloud);
        else
            cluster_publisher_->publish(ros_cloud);
    }

    void publishClusters(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters)
    {
        sensor_msgs::msg::PointCloud2 clustered_cloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr allClusters(new pcl::PointCloud<pcl::PointXYZI>());

        visualization_msgs::msg::MarkerArray markerArray;
        int clusterId = 0;

        for (const auto& cluster : clusters)
        {
            *allClusters += *cluster;
            Box box = pPointCloudProcessor->BoundingBox(cluster);
            visualization_msgs::msg::Marker marker = createBoundingBoxMarker(box, clusterId);
            markerArray.markers.push_back(marker);
            clusterId++;
        }

        publishPointCloud(allClusters, "clustered_cloud");
        marker_publisher_->publish(markerArray);
    }

    visualization_msgs::msg::Marker createBoundingBoxMarker(Box box, int id)
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = rclcpp::Clock().now();
        marker.ns = "bounding_boxes";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = (box.x_min + box.x_max) / 2.0;
        marker.pose.position.y = (box.y_min + box.y_max) / 2.0;
        marker.pose.position.z = (box.z_min + box.z_max) / 2.0;
        marker.scale.x = box.x_max - box.x_min;
        marker.scale.y = box.y_max - box.y_min;
        marker.scale.z = box.z_max - box.z_min;
        marker.color.a = 0.5;
        marker.color.r = 0.5;
        marker.color.g = 0.0;
        marker.color.b = 0.0;

        return marker;
    }

    ProcessPointClouds<pcl::PointXYZI>* pPointCloudProcessor;
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud_;
    std::vector<std::string> pcd_files_;
    size_t current_pcd_index_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarNode>());
    rclcpp::shutdown();
    return 0;
}
