#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <math.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input) {
    // Create a container for the data.
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);

    uint16_t w = 1024;
    uint16_t h = 128;
    float phi_up = 22.5 * M_PI / 180.0; // radians
    float phi_down = -22.5 * M_PI / 180.0; // radians
    
    // Create range image
    cv::Mat range_img = cv::Mat(h, w, CV_32FC1);
    range_img = 0;

    for (int idx = 0; idx < cloud->points.size(); idx++) {
        // Get point from cloud
        pcl::PointXYZI point = cloud->points[idx];

        // Remove invalid points
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z))
            continue;

        // TODO: Calculate the range value
        float range;

        // TODO: Calculate horizontal and vertical angle of the laser beam
        float theta;
        float phi;

        // TODO: Calculate the pixel location in the image                                                 
        uint16_t u;
        uint16_t v;

        // TODO: Limit the range value


        // TODO: Reject points/pixels outside image size 


        // Set the range value in the pixel of an image
        range_img.at<float>(v, u) = range;
    }

    // TODO: Normalize the pixel values of range_img to 0 - 255
    // cv::normalize();

    // Convert image to CV_8UC1
    range_img.convertTo(range_img, CV_8UC1);

    cv_bridge::CvImage out_msg;
    out_msg.header   = input->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    out_msg.image    = range_img;

    pub.publish(out_msg.toImageMsg()); // publish range image
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init(argc, argv, "pc_to_range_image");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe("os_cloud_node/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::Image>("range_image", 1);

    // Spin
    ros::spin();
}