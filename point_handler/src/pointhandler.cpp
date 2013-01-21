#include <iostream>
// ROS specific includes
#include <ros/ros.h>
#include <std_msgs/Empty.h>
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

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

ros::Publisher pub;
ros::Publisher polygon_pub;
ros::Publisher poly_clear_pub;
sensor_msgs::PointCloud2 cloud2_buffer;
bool first_time = true;

/* Note: This code makes extensive use of pcl tutorial code */

void floor_plane_intersect(float a, float b, float c, float d)
{
    // Floor coefficients (plane at y = 0)
    // Note: zeros skipped
    float b_f = 1;
    
    // Find shared normal
    // ie. n1 x n2
    // Note: Cross product calc skips 0 vars
    std::vector<float> n; // This compiler doesn't appear to support specific vector intialization
    n.push_back(-c*b_f);
    n.push_back(0);
    n.push_back(a*b_f);
    
    // Find point on both planes
    // We know both must have one at y=0
    // let x = 0 or z = 0 depending on coefficients
    // therefore cz = d or ax = d
    std::vector<float> p;
    if (a == 0)
    {
        // let x = 0        
        p.push_back(0);
        p.push_back(0);
        p.push_back(d/c);
    }   
    else
    {
        // let y = 0
        p.push_back(d/a);
        p.push_back(0);
        p.push_back(0);
    }
    std::cout<<"Line Equation of:"<<std::endl;
    std::cout<<p[0]<<" + "<<n[0]<<"x"<<std::endl;
    std::cout<<p[1]<<" + "<<n[1]<<"y"<<std::endl;
    std::cout<<p[2]<<" + "<<n[2]<<"y"<<std::endl;
}


void cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud1)
{
    std::cout<<"Receiving new cloud"<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    
    if (first_time)
    {      
        sensor_msgs::convertPointCloudToPointCloud2( *cloud1, cloud2_buffer);
        cloud2_buffer.header.frame_id = "/world";
        first_time = false;
    }
    else
    {
        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::convertPointCloudToPointCloud2( *cloud1, cloud2);        
        // Merge new cloud into old
        /*
        std::cout<<cloud2_buffer.height<<", "<<cloud2.height<<std::endl;
        std::cout<<cloud2_buffer.width<<", "<<cloud2.width<<std::endl;
        std::cout<<cloud2_buffer.fields.size()<<", "<<cloud2.fields.size()<<std::endl;
        std::cout<<cloud2_buffer.point_step<<", "<<cloud2.point_step<<std::endl;
        std::cout<<cloud2_buffer.row_step<<", "<<cloud2.row_step<<std::endl;
        std::cout<<cloud2_buffer.data.size()<<", "<<cloud2.data.size()<<std::endl;
        */
        
        /*
        cloud2_buffer.data.reserve(cloud2_buffer.data.size()+cloud2.data.size());
        // Shift existing y to start of new y section
        cloud2_buffer.data.insert(cloud2_buffer.data[cloud2_buffer.row_step+cloud2.row_step], cloud2_buffer.data[cloud2_buffer.row_step], cloud2_buffer.data[2*cloud2_buffer.row_step]);
        // Append new y
        cloud2_buffer.data.insert(cloud2_buffer.data[2*cloud2_buffer.row_step+cloud2.row_step], cloud2.data[cloud2.row_step], cloud2.data[2*cloud2.row_step]);
        // Shift existing z
        cloud2_buffer.data.insert(cloud2_buffer.data[2*(cloud2_buffer.row_step+cloud2.row_step)], cloud2_buffer.data[2*cloud2_buffer.row_step], cloud2_buffer.data[3*cloud2_buffer.row_step]);
        // Append new z
        cloud2_buffer.data.insert(cloud2_buffer.data[3*cloud2_buffer.row_step+2*cloud2.row_step], cloud2.data[2*cloud2.row_step], cloud2.data[3*cloud2.row_step]);
        // Overwrite old existing y with new extended x
        cloud2_buffer.data.insert(cloud2_buffer.data[cloud2_buffer.row_step], cloud2.data.begin(), cloud2.data[cloud2.row_step]);
        */
        
        std::vector<uint8_t> data;
        data.reserve(cloud2_buffer.data.size()+cloud2.data.size());
        
        // Note: The values passed here are iterators, so the '+' is infact shifting the iterator rather than adding to it
        
        data.insert(data.end(), cloud2_buffer.data.begin(), cloud2_buffer.data.begin()+cloud2_buffer.row_step);
        data.insert(data.end(), cloud2.data.begin(), cloud2.data.begin()+cloud2.row_step);
        
        data.insert(data.end(), cloud2_buffer.data.begin()+cloud2_buffer.row_step, cloud2_buffer.data.begin()+2*cloud2_buffer.row_step);
        data.insert(data.end(), cloud2.data.begin()+cloud2.row_step, cloud2.data.begin()+2*cloud2.row_step);
        
        data.insert(data.end(), cloud2_buffer.data.begin()+2*cloud2_buffer.row_step, cloud2_buffer.data.begin()+3*cloud2_buffer.row_step);
        data.insert(data.end(), cloud2.data.begin()+2*cloud2.row_step, cloud2.data.begin()+3*cloud2.row_step);
        
        cloud2_buffer.data = data;
        
        
        cloud2_buffer.width += cloud2.width;
        cloud2_buffer.row_step += cloud2.row_step;
        
        
        /*
        for (int i = 0; i< cloud2.data.size(); i++)
        {
            cloud2_buffer.data.push_back(cloud2.data[i]);
        }
        */
        
        
        /*
        
        
        sensor_msgs::PointCloud2 test_cloud2;
        pcl::toROSMsg(*cloud, test_cloud2);
        
        sensor_msgs::PointCloud test_cloud;
        sensor_msgs::convertPointCloud2ToPointCloud(test_cloud2, test_cloud);
        for (int i = 0; i<test_cloud.points.size(); i++)
        {
          std::cout<<test_cloud.points[i].x<<", "<<test_cloud.points[i].y<<", "<<test_cloud.points[i].z<<std::endl;
        }
        */
        std::cout<<"Merged new cloud"<<std::endl;
        
    }
    
    pcl::fromROSMsg (cloud2_buffer, *cloud);

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
    
    poly_clear_pub.publish(std_msgs::Empty());

    sensor_msgs::PointCloud cloud_buffer;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud2_buffer, cloud_buffer);
    
    std::cout<<"Producing polygons"<<std::endl;
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
        
        point32.x = cloud_buffer.points[triangles.polygons[i].vertices[0]].x;
        point32.y = cloud_buffer.points[triangles.polygons[i].vertices[0]].y;
        point32.z = cloud_buffer.points[triangles.polygons[i].vertices[0]].z;
        polygon.points.push_back(point32);
        
        point32.x = cloud_buffer.points[triangles.polygons[i].vertices[1]].x;
        point32.y = cloud_buffer.points[triangles.polygons[i].vertices[1]].y;
        point32.z = cloud_buffer.points[triangles.polygons[i].vertices[1]].z;
        polygon.points.push_back(point32);
        
        point32.x = cloud_buffer.points[triangles.polygons[i].vertices[2]].x;
        point32.y = cloud_buffer.points[triangles.polygons[i].vertices[2]].y;
        point32.z = cloud_buffer.points[triangles.polygons[i].vertices[2]].z;
        polygon.points.push_back(point32);
        
        polygonStamped.polygon = polygon;
        polygonStamped.header.frame_id = "/world";
        //std::cout<<i<<std::endl;
        polygon_pub.publish(polygonStamped);
    }
    
    
    
    
    
    
    std::cout<<"Done meshing"<<std::endl;
    
    
    std::cout<<"Plane Fitting"<<std::endl;
    // Begin plane fitting
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.1);
    
    
    // Prepare temporary Clouds
    // cloud_p contains points supporting currently tested plane
    // cloud_f contains points not yet fitted to a plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    int i = 0, nr_points = (int) (cloud)->points.size ();
    // While 10% of the original cloud is still there
    while (cloud->points.size () > 0.1 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the inliers
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_p);
        std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
        std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;
        floor_plane_intersect(coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        i++;
    }
    
    
    
    /*
    seg.setInputCloud ((*cloud).makeShared ());
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (size_t i = 0; i < inliers->indices.size (); ++i)
    {
        std::cerr << inliers->indices[i] << "    " << (*cloud).points[inliers->indices[i]].x << " "
                                                << (*cloud).points[inliers->indices[i]].y << " "
                                                << (*cloud).points[inliers->indices[i]].z << std::endl;
    }
    */
        
    pub.publish(cloud2_buffer);
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "Point_Handler");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/scan/absolute_cloud", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("/point_handler/absolute_cloud", 1);
    
    // Create polygon publisher
    polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("/point_handler/polygons", 512);

    // Create polygon publisher
    poly_clear_pub = nh.advertise<std_msgs::Empty>("/point_handler/poly_clear", 512);

    // Spin
    ros::spin ();
}

