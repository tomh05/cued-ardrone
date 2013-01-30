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

#include <pcl/filters/statistical_outlier_removal.h>

ros::Publisher pub;
ros::Publisher render_pub;
ros::Publisher clean_pub;

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
    
    renderer_poly_line_tri.polygons.push_back(polygon);
}


void get_floor_ends(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected)
{
    float end_xyxy[4];
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
    
    for (int i = 0; i<4; i++)
    {
        renderer_poly_line_tri.floorlines.push_back(end_xyxy[i]);
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
    }
}

/// This function combines the passed cloud with the combined cloud buffer
void accumulate_point_cloud_buffer(sensor_msgs::PointCloud2 cloud2)
{
    // Create temporary data store
    std::vector<uint8_t> data;    
    // Pre-allocate memory to avoid mutiple auto resizes during copying
    data.reserve(cloud2_buffer.data.size()+cloud2.data.size());
    
    // Note: The values passed here are iterators, so the '+' is infact shifting the iterator rather than adding to it
    
    // Copy old x values followed by new
    data.insert(data.end(), cloud2_buffer.data.begin(), cloud2_buffer.data.begin()+cloud2_buffer.row_step);
    data.insert(data.end(), cloud2.data.begin(), cloud2.data.begin()+cloud2.row_step);
    
    // Copy old y values followed by new
    data.insert(data.end(), cloud2_buffer.data.begin()+cloud2_buffer.row_step, cloud2_buffer.data.begin()+2*cloud2_buffer.row_step);
    data.insert(data.end(), cloud2.data.begin()+cloud2.row_step, cloud2.data.begin()+2*cloud2.row_step);
    
    // Copy old z values followed by new
    data.insert(data.end(), cloud2_buffer.data.begin()+2*cloud2_buffer.row_step, cloud2_buffer.data.begin()+3*cloud2_buffer.row_step);
    data.insert(data.end(), cloud2.data.begin()+2*cloud2.row_step, cloud2.data.begin()+3*cloud2.row_step);
    
    // Set buffer to temp data store
    cloud2_buffer.data = data;
    
    // Update stride info
    cloud2_buffer.width += cloud2.width;
    cloud2_buffer.row_step += cloud2.row_step;
    
    // Publish accumulated point cloud
    pub.publish(cloud2_buffer);
}

void cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud1)
{
    // Clear output message for renderer
    renderer_poly_line_tri = custom_msgs::RendererPolyLineTri();
    
    std::cout<<"Receiving new cloud"<<std::endl;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);    
    if (first_time)
    {
        // On first data, set accumulated point cloud to received cloud      
        sensor_msgs::convertPointCloudToPointCloud2( *cloud1, cloud2_buffer);
        cloud2_buffer.header.frame_id = "/world";
        first_time = false;
    }
    else
    {
        // Convert to PointCloud2 format
        sensor_msgs::PointCloud2 cloud2;
        sensor_msgs::convertPointCloudToPointCloud2( *cloud1, cloud2);
        
        // Merge new points with buffer of all points
        accumulate_point_cloud_buffer(cloud2);
        
        std::cout<<"Merged new cloud"<<std::endl<<std::endl;        
    }
    
    std::cout<<"Beginning Meshing"<<std::endl;
    
    // Carry out a greedy triangulation on total point cloud
    pcl::fromROSMsg (cloud2_buffer, *cloud);
    triangulate_point_cloud(cloud);
    
    std::cout<<"Done meshing"<<std::endl<<std::endl;
    
    std::cout<<"Removing statistical outliers"<<std::endl;
    // At present points are just removed from the analysis cycle
    // It is unclear whether it will be feasible to remove permanently as this
    // may remove too much 'good' data as out data set it very sparse


    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);


    std::cerr << "  Cloud before filtering: "<<cloud->points.size () << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    // Consider 50 nearest neighbours
    sor.setMeanK (50);
    // Filter those at > 3 standard deviations above the mean sparseness
    sor.setStddevMulThresh (3.0);
    sor.filter (*cloud_filtered);
    
    std::cerr << "  Cloud after filtering: "<<cloud_filtered->points.size () << std::endl;
    cloud = cloud_filtered;
    
    sensor_msgs::PointCloud2 temp_cloud2;
    sensor_msgs::PointCloud temp_cloud;
    pcl::toROSMsg(*cloud, temp_cloud2);
    sensor_msgs::convertPointCloud2ToPointCloud(temp_cloud2, temp_cloud);
    clean_pub.publish(temp_cloud);
    
    std::cout<<"Done Cleaning"<<std::endl<<std::endl;
    
    std::cout<<"Beginning Plane Fitting"<<std::endl<<std::endl;
    // Begin plane fitting/segmentation routine:
    // 1) Vertical planes are fitted to the point cloud using the following:
    //      a) RANSAC fit vertical plane
    //      b) Remove supporting data points
    //      c) Repeat a) to b) until no planes, or 90% of points fitted
    // 2) Separate planes into sections using euclidian clustering (i.e. based
    //    on distance)
    // 3) Find Convex hull for each plane section
    // 4) Find floorplan line by getting the projected extremes of each section
    
    
    // Create necessary objects
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Prepare temporary Clouds
    // cloud_p contains points supporting currently tested plane
    // cloud_f contains points not yet fitted to a plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    
    // Set segmentation settings
    
    seg.setOptimizeCoefficients (true);
    seg.setMaxIterations(1000);
    
    // Only accept planes within +/-10deg of vertical
    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setAxis (Eigen::Vector3f (1.0, 1.0, 0.0));
    seg.setEpsAngle (pcl::deg2rad (10.));
    
    // Use RANSAC search approach
    seg.setMethodType (pcl::SAC_RANSAC);
       
    // Allow for +/-10cm distance from plane for inliers
    seg.setDistanceThreshold (0.1);
    
    
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
        
        // Setup euclidean cluster parameters
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.45); // 45cm
        ec.setMinClusterSize (8);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloud_p);
        ec.extract (cluster_indices);

        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            // Iterate through planar cloud section, clustering
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
            chull.reconstruct (*cloud_hull);

            std::cerr << "          Convex hull has: " << cloud_hull->points.size () << " data points." << std::endl;
            
            // Order convex hull and publish
            publish_ordered_convex_hull(cloud_hull);
            
            
            ///----------------------------------------------------------------
            
            
            
            ///================================================================
            /// Get floor plan
            ///================================================================                                     
            get_floor_ends(cloud_projected);            
            ///----------------------------------------------------------------
            
            j++;
        }
        std::cout<<"    Sub-Plane clustering complete"<<std::endl;    
        // Create the filtering object and remove fitted points
        extract.setNegative (true);
        extract.filter (*cloud_f);
        cloud.swap (cloud_f);
        i++;
    }
    std::cout<<"    Plane fitting complete"<<std::endl<<std::endl;
    
    // Publish data for renderer
    render_pub.publish(renderer_poly_line_tri);
    
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
    
    clean_pub = nh.advertise<sensor_msgs::PointCloud>("/point_handler/clean_absolute_cloud", 1);
    
    // Create render publisher
    render_pub = nh.advertise<custom_msgs::RendererPolyLineTri>("/point_hander/renderer_data", 16);

    // Spin
    ros::spin ();
}

