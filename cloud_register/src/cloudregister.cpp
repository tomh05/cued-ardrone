#include <iostream>
#include <cmath>
// ROS specific includes
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <custom_msgs/StampedStampedInt.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

ros::Publisher pub;

// Vector of pointers is used to prevent recreation of buffer everytime we get a new cloud
// (C++ vectors are contiguous so adding large amounts means they are reallocated 
//  and the old data copied)
std::vector<sensor_msgs::PointCloud2::Ptr> cloud2_buffer = std::vector<sensor_msgs::PointCloud2::Ptr>();
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudT_buffer = std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>();

// This stores the index for clouds that are believed to be linked due to feature matches
// This could arguable be done with pointers (to the clouds) instead
std::vector<std::vector<int> > cloud_links = std::vector<std::vector<int> >();


/* Note: This code makes extensive use of pcl tutorial code */

/// This function combines the passed cloud with the combined cloud buffer
void accumulate_point_cloud_buffer(sensor_msgs::PointCloud2 cloud2)
{
}

void register_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    setVerbosityLevel(pcl::console::L_DEBUG); 
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout<< "Has converged:"<< icp.hasConverged() <<"| score: "<<icp.getFitnessScore()<<std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    
    std::cout<<"Point Cloud Registration Complete"<<std::endl<<std::endl<<std::endl<<std::endl;
}

void on_got_matches(custom_msgs::StampedStampedInt s_s_int)
{
    std::cout<<"Receiving new match"<<std::endl;
    int a = -1;
    int b = -1;
    for (int i = 0; i < cloud2_buffer.size(); i++)
    {
        if (cloud2_buffer[i]->header.frame_id == s_s_int.header1.frame_id && cloud2_buffer[i]->header.stamp == s_s_int.header1.stamp)
        {
            if (a != -1) std::cout<<"WARNING: Ambiguous headers"<<std::endl;
            a = i;
        }
        if (cloud2_buffer[i]->header.frame_id == s_s_int.header2.frame_id && cloud2_buffer[i]->header.stamp == s_s_int.header2.stamp)
        {
            if (b != -1) std::cout<<"WARNING: Ambiguous headers"<<std::endl;
            b = i;
        }
    }
    if (a == -1 || b == -1)
        std::cout<<"WARNING: Point Cloud Dropped"<<std::endl;
    else if (a == b)
        std::cout<<"WARNING: Self Matches Detected"<<std::endl;
    else
    {
        std::cout<<"Added Match from "<<a<<" to "<<b<<std::endl<<std::endl<<std::endl<<std::endl;
        std::vector<int> matches = std::vector<int>();
        matches.push_back(a);
        matches.push_back(b);
        cloud_links.push_back(matches);
        std::cout<<"Attempting to register "<<a<<" with "<<b<<std::endl;
        register_clouds(cloudT_buffer[a], cloudT_buffer[b]);
    }
}

void on_got_cloud(const sensor_msgs::PointCloudConstPtr& cloud1)
{
    std::cout<<"Receiving new cloud"<<std::endl;
    sensor_msgs::PointCloud2::Ptr cloud2 (new sensor_msgs::PointCloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudT (new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::convertPointCloudToPointCloud2(*cloud1, *cloud2);
    cloud2_buffer.push_back(cloud2);
    pcl::fromROSMsg (*cloud2, *cloudT);
    cloudT_buffer.push_back(cloudT);
    std::cout<<"Added cloud to buffer"<<std::endl<<std::endl<<std::endl<<std::endl;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "Cloud_Register");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/scan/relative_cloud", 2, on_got_cloud);
    ros::Subscriber sub2 = nh.subscribe ("/cloud_compare/matches", 8, on_got_matches);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_register/absolute_cloud", 1);
    
    // Spin
    ros::spin ();
}

