#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>   // ← use .h, not .hpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// For brevity:
using std::placeholders::_1;

/*
 * This node:
 *  1) Subscribes to a PointCloud2 (e.g. "/camera/rgbd/points")
 *  2) Runs RANSAC plane segmentation (SACSegmentation) to find the ground plane
 *  3) Extracts all points NOT on that plane
 *  4) Republishes the “no-ground” cloud on "/cloud_no_ground_ransac"
 *
 * RANSAC parameters are hardcoded here (3 cm threshold, 50 iterations).
 * Feel free to expose them as ROS 2 parameters if needed.
 */

class GroundRemovalNode : public rclcpp::Node
{
public:
  GroundRemovalNode()
  : Node("ground_removal_node")
  {
    // Hardcoded: any point within 3 cm of plane = “ground”
    dist_thresh_ = 0.03;   // meters
    max_iter_   = 50;      // RANSAC iterations

    // Use best_effort QoS, depth=1
    rclcpp::QoS qos(1);
    qos.best_effort();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points",  // ← adjust to your actual input topic
      qos,
      std::bind(&GroundRemovalNode::cloudCallback, this, _1)
    );

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/cloud_no_ground_ransac",
      10
    );

    RCLCPP_INFO(this->get_logger(),
      "GroundRemovalNode started: dist_thresh=%.3f m, max_iter=%d",
      dist_thresh_, max_iter_
    );
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Convert incoming ROS2 PointCloud2 → PCL PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_in(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *pcl_in);

    if (pcl_in->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty cloud; skipping.");
      return;
    }

    // 1) RANSAC plane segmentation setup
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_thresh_);
    seg.setMaxIterations(max_iter_);
    seg.setInputCloud(pcl_in);

    // 2) Run segmentation
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
      RCLCPP_WARN(this->get_logger(),
        "RANSAC found no plane; publishing original cloud.");
      // Publish original unmodified cloud:
      sensor_msgs::msg::PointCloud2 out_msg;
      pcl::toROSMsg(*pcl_in, out_msg);
      out_msg.header = msg->header;
      publisher_->publish(out_msg);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
      "Plane inliers: %zu points", inliers->indices.size());

    // 3) Extract points not on that plane
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(pcl_in);
    extract.setIndices(inliers);
    extract.setNegative(true);  // keep points that are NOT inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*no_ground);

    RCLCPP_INFO(this->get_logger(),
      "After removing plane: %zu points remain", no_ground->size());

    // 4) Convert back to ROS2 PointCloud2 and publish
    sensor_msgs::msg::PointCloud2 out_msg;
    pcl::toROSMsg(*no_ground, out_msg);
    out_msg.header = msg->header;  // preserve original timestamp & frame
    publisher_->publish(out_msg);
  }

  // RANSAC parameters
  double dist_thresh_;
  int    max_iter_;

  // ROS 2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr  publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundRemovalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
