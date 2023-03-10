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

    float SCANNER_COLS = 1024;
    float SCANNER_ROWS = 128;
    
    // Create range image
    cv::Mat range_img = cv::Mat(SCANNER_ROWS, SCANNER_COLS, CV_32FC1);
    range_img = 0;

    for (int idx = 0; idx < cloud->points.size(); idx++) {
        // Get point from cloud
        pcl::PointXYZI point = cloud->points[idx];

        // Remove invalid points
        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.z))
            continue;
            
        // TODO: Calculate horizontal and vertical angle of the laser beam


        // TODO: Calculate the range value
        float range = 0;


        // TODO: Calculate the pixel location in the image                                                 
        float height = 0;
        float width = 0;


        // TODO: Limit the range value

        // TODO: Reject points/pixels outside image size 
    

        // Set the range value in the pixel of an image
        range_img.at<float>(height, width) = range;
    }

    // TODO: Normalize range_img to 0 - 255
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