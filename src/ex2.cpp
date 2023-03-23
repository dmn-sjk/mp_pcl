#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>


ros::Publisher cones_pub;
ros::Publisher processed_cloud_pub;

void filter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // TODO
    pcl::VoxelGrid<pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);
    filter.setLeafSize (0.1, 0.1, 0.1);
    filter.filter(*cloud);
}

void remove_ground(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    // TODO
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    pcl::SACSegmentation<pcl::PointXYZ> sac;
    sac.setModelType(pcl::SACMODEL_PLANE);
    sac.setMethodType(pcl::SAC_RANSAC);
    sac.setDistanceThreshold (0.01);

    sac.setInputCloud(cloud);
    sac.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud);
}

void get_cluster_indices(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                         std::vector<pcl::PointIndices> &output_cluster_indices) {
    // TODO
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
    kdtree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.4);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(cloud);
    ec.extract(output_cluster_indices);
}

void get_cones_centers_cloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &cloud,
                             const std::vector<pcl::PointIndices> &cluster_indices,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cones_cloud) {
    // TODO
    pcl::PointXYZ p;
    long j = 0;
    float x, y = 0.0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        for (const auto& idx : it->indices) {
            x += (*cloud)[idx].x;
            y += (*cloud)[idx].y;
            j++;
        }

        p.x = x / j;
        p.y = y / j;
        p.z = 0.0; // z-axis info not needed to follow the track

        output_cones_cloud->push_back(p);

        j = 0;
        x = 0.0;
        y = 0.0;
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