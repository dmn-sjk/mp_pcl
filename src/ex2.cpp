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
#include <pcl/common/centroid.h>

ros::Publisher cones_pub;
ros::Publisher processed_cloud_pub;

void filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::VoxelGrid<pcl::PointXYZ> filter;

    // TODO: Set input cloud and leaf size

    filter.filter(*cloud);
}

void remove_ground(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // indices of point clusters inside point cloud
    pcl::SACSegmentation<pcl::PointXYZ> sac;

    // TODO: Set model type (plane), set method type (RANSAC), set distance threshold, set input cloud

    sac.segment(*inliers, *coefficients);


    pcl::ExtractIndices<pcl::PointXYZ> extract;

    // TODO: Set input cloud, set indices, use setNegative function to retrieve only the points that are not inliers (not on the ground plane)

    extract.filter(*cloud);
}

void get_cluster_indices(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                         std::vector<pcl::PointIndices> &output_cluster_indices) {

    // fast data structure for search and retrieval of point inside the point cloud
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setSearchMethod(kdtree);

    // TODO: Set input cloud, set cluster tolerance, set min cluster size, set max cluster size

    ec.extract(output_cluster_indices);
}

void get_cones_centers_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                             const std::vector<pcl::PointIndices> &cluster_indices,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cones_cloud) {
    // for each cluster of points
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        pcl::CentroidPoint<pcl::PointXYZ> centroid;
        pcl::PointXYZ centroid_point;

        // for each point inside the cluster
        for (const auto& idx : it->indices) {
            pcl::PointXYZ point = (*cloud)[idx];

            // TODO: add point to the centroid object

        }

        // TODO: assign a centroid point to centroid_point variable


        output_cones_cloud->push_back(centroid_point);
    }
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