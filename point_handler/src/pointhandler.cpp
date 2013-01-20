#include <iostream>
// ROS specific includes
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

ros::Publisher pub;
ros::Publisher polygon_pub;

void cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud1)
{
    
    // Convert to PointCloudT for pcl
    sensor_msgs::PointCloud2 cloud2;
    sensor_msgs::convertPointCloudToPointCloud2( *cloud1, cloud2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (cloud2, *cloud);

    // Normal estimation
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    // normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.5);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    for (int i = 0; i<triangles.polygons.size(); i++)
    {
        /*std::cout<<triangles.polygons[i]<<std::endl;
        std::cout<<triangles.polygons[i].vertices[0]<<std::endl;
        std::cout<<triangles.polygons[i].vertices[1]<<std::endl;
        std::cout<<triangles.polygons[i].vertices[2]<<std::endl;
        std::cout<<(*cloud1).points[triangles.polygons[i].vertices[0]]<<std::endl;
        std::cout<<(*cloud1).points[triangles.polygons[i].vertices[1]]<<std::endl;
        std::cout<<(*cloud1).points[triangles.polygons[i].vertices[2]]<<std::endl;*/
        geometry_msgs::PolygonStamped polygonStamped;
        geometry_msgs::Polygon polygon;
        geometry_msgs::Point32 point32;
        
        point32.x = (*cloud1).points[triangles.polygons[i].vertices[0]].x;
        point32.y = (*cloud1).points[triangles.polygons[i].vertices[0]].y;
        point32.z = (*cloud1).points[triangles.polygons[i].vertices[0]].z;
        polygon.points.push_back(point32);
        
        point32.x = (*cloud1).points[triangles.polygons[i].vertices[1]].x;
        point32.y = (*cloud1).points[triangles.polygons[i].vertices[1]].y;
        point32.z = (*cloud1).points[triangles.polygons[i].vertices[1]].z;
        polygon.points.push_back(point32);
        
        point32.x = (*cloud1).points[triangles.polygons[i].vertices[2]].x;
        point32.y = (*cloud1).points[triangles.polygons[i].vertices[2]].y;
        point32.z = (*cloud1).points[triangles.polygons[i].vertices[2]].z;
        polygon.points.push_back(point32);
        
        polygonStamped.polygon = polygon;
        polygonStamped.header.frame_id = "/world";
        
        polygon_pub.publish(polygonStamped);
    }
    
    
    
    
    
    
    std::cout<<"Done"<<std::endl;
    
    // Publish the data
    pub.publish (cloud2);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "Point_Handler");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("pointCloud", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    
    // Create polygon publisher
    polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("polygons", 512);

    // Spin
    ros::spin ();
}
