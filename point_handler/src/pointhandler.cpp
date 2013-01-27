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
#include <custom_msgs/RendererPolyLineTri.h>
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
#include <pcl/common/angles.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;
ros::Publisher polygon_pub;
ros::Publisher triangle_pub;
ros::Publisher poly_clear_pub;
ros::Publisher line_pub;
ros::Publisher render_pub;

custom_msgs::RendererPolyLineTri renderer_poly_line_tri;

// Is declaring this globally a very bad idea? (put on stack not heap?)
sensor_msgs::PointCloud2 cloud2_buffer;
bool first_time = true;



/* Note: This code makes extensive use of pcl tutorial code */



void publish_ordered_convex_hull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull)
{
    // Order points
    // 1) Create index vector
    // 2) Find top-most point
    //std::cout<<"Cycling points"<<std::endl;
    std::vector<int> index;
    std::vector<int> ordered_index;
    int current;
    int target;
    float max_z = -999999999;
    ordered_index.push_back(0);
    for (int i = 0; i<cloud_hull->size(); i++)
    {
        //std::cout<<cloud_hull->points[i].x<<", "<<cloud_hull->points[i].y<<", "<<cloud_hull->points[i].z<<std::endl;
        index.push_back(i);
        if (cloud_hull->points[i].z > max_z)
        {
            max_z = cloud_hull->points[i].z;
            ordered_index[ordered_index.size()-1] = index[i];
            target = i;
        }        
    }
    //std::cout<<"Max z picked : "<<cloud_hull->points[ordered_index[0]].x<<", "<<cloud_hull->points[ordered_index[0]].y<<", "<<cloud_hull->points[ordered_index[0]].z<<std::endl;
    
    while(index.size() > 2)
    {
        //std::cout<<"Removing "<<target<<std::endl;
        index.erase(index.begin()+target);
        
        
        ordered_index.push_back(0);
        // 3) Search for point with lowest angle
        float min_angle = 2*3.141592654;
        for (int i = 0; i < index.size(); i++)
        {
            //std::cout<<"Comparing to : "<<std::endl;
            //std::cout<<cloud_hull->points[index[i]].x<<", "<<cloud_hull->points[index[i]].y<<", "<<cloud_hull->points[index[i]].z<<std::endl;
    
            float dxtest = 0.707106;
            float dytest = 0.707106;
            float dztest = 0.;
            
            float dx = cloud_hull->points[index[i]].x-cloud_hull->points[ordered_index[0]].x;
            float dy = cloud_hull->points[index[i]].y-cloud_hull->points[ordered_index[0]].y;
            float dz = cloud_hull->points[index[i]].z-cloud_hull->points[ordered_index[0]].z;
            float mag = std::sqrt(dx*dx+dy*dy+dz*dz);

            //std::cout<<"dx, dy, dz : "<<dx<<", "<<dy<<", "<<dz<<std::endl;
            float angle = std::acos((dxtest*dx+dytest*dy)/mag);
            //std::cout<<"angle"<<angle<<std::endl;
            if (angle < 0)
            {
                angle += 2*3.141592654;
            }
            if (angle<min_angle)
            {
                min_angle = angle;
                ordered_index[ordered_index.size()-1] = index[i];
                target = i;
            }
        }
        //std::cout<<"min_angle: "<<min_angle<<std::endl;
        //std::cout<<"Point picked : "<<cloud_hull->points[ordered_index[ordered_index.size()-1]].x<<", "<<cloud_hull->points[ordered_index.size()-1].y<<", "<<cloud_hull->points[ordered_index.size()-1].z<<std::endl;
    }
    ordered_index.push_back(index[0]);
    
    
    geometry_msgs::PolygonStamped polygonStamped;
    geometry_msgs::Polygon polygon;
    geometry_msgs::Point32 point32;

    std::cout<<"            Producing polygons"<<std::endl;
    for (int i = 0; i<ordered_index.size(); i++)
    {
        //std::cout<<"ordered index : "<<ordered_index[i]<<std::endl; 
        point32.x = cloud_hull->points[ordered_index[i]].x;
        point32.y = cloud_hull->points[ordered_index[i]].y;
        point32.z = cloud_hull->points[ordered_index[i]].z;
        //std::cout<<point32.x<<", "<<point32.y<<", "<<point32.z<<std::endl;
        polygon.points.push_back(point32);            
        
    }

    polygonStamped.polygon = polygon;
    polygonStamped.header.frame_id = "/world";
    polygon_pub.publish(polygonStamped);
    
    renderer_poly_line_tri.polygons.push_back(polygon);
}

void publish_floor_ends(std::vector<float> floor_lines)
{
    // Note ros arrays are similar to c++ vectors in functionality
    // c++ vectors will not work as they lack ros specific functionality
    // arrays will not work as they have no object functionality
    std_msgs::Float32MultiArray floor_array;
    
    for (int i = 0; i < floor_lines.size(); i++)
    {
        floor_array.data.push_back(floor_lines[i]);
        renderer_poly_line_tri.floorlines.push_back(floor_lines[i]);
    }
    line_pub.publish(floor_array);
}


void get_floor_ends(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected, float end_xyxy[])
{
    end_xyxy[0] = 9999;
    end_xyxy[1] = 9999;
    end_xyxy[2] = -9999;
    end_xyxy[3] = -9999;
    for (int i = 0; i<cloud_projected->size(); i++)
    {
        if (cloud_projected->points[i].x < end_xyxy[0])
        {
            end_xyxy[0] = cloud_projected->points[i].x;
        }
        if (cloud_projected->points[i].x > end_xyxy[2])
        {
            end_xyxy[2] = cloud_projected->points[i].x;
        }
        if (cloud_projected->points[i].y < end_xyxy[1])
        {
            end_xyxy[1] = cloud_projected->points[i].y;
        }
        if (cloud_projected->points[i].y > end_xyxy[3])
        {
            end_xyxy[3] = cloud_projected->points[i].y;
        }
    }
}

void triangulate_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
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
    gp3.setSearchRadius (0.2);

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
    

    sensor_msgs::PointCloud cloud_buffer;
    sensor_msgs::convertPointCloud2ToPointCloud(cloud2_buffer, cloud_buffer);
    
    std::cout<<"    Producing triangles"<<std::endl;
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
        
        renderer_poly_line_tri.triangles.push_back(polygon);
        polygonStamped.polygon = polygon;
        polygonStamped.header.frame_id = "/world";
        
        //std::cout<<i<<std::endl;
        triangle_pub.publish(polygonStamped);
    }
}


void cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud1)
{
    std::vector<float> floor_lines;
    renderer_poly_line_tri = custom_msgs::RendererPolyLineTri();
    
    std::cout<<"Receiving new cloud"<<std::endl;
    poly_clear_pub.publish(std_msgs::Empty());
    
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
        std::cout<<"Merged new cloud"<<std::endl<<std::endl;
        
    }
    
    std::cout<<"Beginning Meshing"<<std::endl;
    
    pcl::fromROSMsg (cloud2_buffer, *cloud);

    
    
    
    
    
    
    
    std::cout<<"Done meshing"<<std::endl;
    
    
    std::cout<<"Beginning Plane Fitting"<<std::endl<<std::endl;
    // Begin plane fitting
    
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    //seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setAxis (Eigen::Vector3f (1.0, 1.0, 0.0));
    seg.setEpsAngle (pcl::deg2rad (10.));
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
        std::cerr << "  PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
        //std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
        //                              << coefficients->values[1] << " "
        //                              << coefficients->values[2] << " " 
        //                             << coefficients->values[3] << std::endl;
                                      
        
        std::cout<<"    Beginning Sub-plane clustering"<<std::endl;
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloud_p);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.45); // 2cm
        ec.setMinClusterSize (8);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_p);
        ec.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            {
                cloud_cluster->points.push_back (cloud_p->points[*pit]); 
            }
            cloud_cluster->width = cloud_cluster->points.size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "      PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
            
            
            
            
            
            ///================================================================
            /// Get convex hull
            ///================================================================
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::ProjectInliers<pcl::PointXYZ> proj;
            pcl::ConvexHull<pcl::PointXYZ> chull;
            chull.setDimension(2);
            
            // Project the model inliers         
            proj.setModelType (pcl::SACMODEL_PLANE);
            proj.setInputCloud (cloud_cluster);
            proj.setModelCoefficients (coefficients);
            proj.filter (*cloud_projected);

            // Create a Convex Hull representation of the projected inliers
            chull.setInputCloud (cloud_projected);
            //chull.setAlpha (0.5);
            chull.reconstruct (*cloud_hull);

            std::cerr << "          Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
            
            // Order convex hull and publish
            publish_ordered_convex_hull(cloud_hull);
            
            
            ///----------------------------------------------------------------
            
            
            
            ///================================================================
            /// Get floor plan
            ///================================================================                                     
            float end_xyxy[4];
            get_floor_ends(cloud_projected, end_xyxy);
            for (int k = 0; k<4; k++)
            {
                floor_lines.push_back(end_xyxy[k]);
            }
            ///----------------------------------------------------------------
            
            j++;
        }
        std::cout<<"    Sub-Plane clustering complete"<<std::endl;    
        // Create the filtering object
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        i++;
    }
    std::cout<<"    Plane fitting complete"<<std::endl<<std::endl;
    
    publish_floor_ends(floor_lines);
    
    render_pub.publish(renderer_poly_line_tri);
    
    pub.publish(cloud2_buffer);
    
    std::cout<<"Point Cloud Handling Complete"<<std::endl<<std::endl<<std::endl<<std::endl;
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
    polygon_pub = nh.advertise<geometry_msgs::PolygonStamped>("/point_handler/polygon", 512);
    
    // Create triangle publisher    
    triangle_pub = nh.advertise<geometry_msgs::PolygonStamped>("/point_handler/triangle", 512);

    // Create polygon clear trigger publisher
    poly_clear_pub = nh.advertise<std_msgs::Empty>("/point_handler/poly_clear", 512);
    
    // Create floorplan line publisher
    line_pub = nh.advertise<std_msgs::Float32MultiArray>("/point_handler/lines", 64);
    
    // Create render publisher
    render_pub = nh.advertise<custom_msgs::RendererPolyLineTri>("/point_hander/renderer_data", 16);

    // Spin
    ros::spin ();
}

