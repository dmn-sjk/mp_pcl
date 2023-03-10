#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher cones_pub;
ros::Publisher processed_cloud_pub;

void filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // TODO
}

void remove_ground(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // TODO
}

void get_cluster_indices(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                         std::vector<pcl::PointIndices> &output_cluster_indices) {
    // TODO
}

void get_cones_centers_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                             const std::vector<pcl::PointIndices> &cluster_indices,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cones_cloud) {
    // TODO
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
    // Create a container for the data.
    sensor_msgs::PointCloud2 output;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromROSMsg(*input, *pcl_cloud);

    int cloud_size = pcl_cloud->points.size();
    ROS_INFO("Cloud size before filtering: %d", cloud_size);

    filter(pcl_cloud);

    cloud_size = pcl_cloud->points.size();
    ROS_INFO("Cloud size after filtering: %d\n", cloud_size);

    remove_ground(pcl_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    get_cluster_indices(pcl_cloud, cluster_indices);
    get_cones_centers_cloud(pcl_cloud, cluster_indices, centroid_cloud);

    // Publish processed cloud
    pcl::toROSMsg(*pcl_cloud, output);
    output.header = input->header;
    output.fields = input->fields;
    processed_cloud_pub.publish(output);

    // Publish cone centers
    pcl::toROSMsg(*centroid_cloud, output);
    output.header = input->header;
    output.fields = input->fields;
    cones_pub.publish(output);
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "cones_detector");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("fsds/lidar/Lidar", 1, cloud_cb);

    // Create a ROS publisher for the output point clouds
    cones_pub = nh.advertise<sensor_msgs::PointCloud2>("cones_cloud", 1);
    processed_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("processed_cloud", 1);

    // Spin
    ros::spin();
}