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

#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

ros::Publisher pub;
ros::Publisher pub_merge;
ros::Publisher pub_a;
ros::Publisher pub_b;

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



class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500)
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

// Align a collection of object templates to a sample point cloud
void align(pcl::PointCloud<pcl::PointXYZ>::ptr target,pcl::PointCloud<pcl::PointXYZ>::ptr source)
{

  // Load the target cloud PCD file
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (argv[2], *cloud);

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud);
  

    // Compute the surface normals
    normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
    norm_est.setInputCloud (xyz_);
    norm_est.setSearchMethod (search_method_xyz_);
    norm_est.setRadiusSearch (normal_radius_);
    norm_est.compute (*normals_);

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  template_align.setTargetCloud (target_cloud);

  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  // Save the aligned template for visualization
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
  pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);

  return (0);
}

void register_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    setVerbosityLevel(pcl::console::L_DEBUG); 
    icp.setMaxCorrespondenceDistance(10.);
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout<< "Has converged:"<< icp.hasConverged() <<"| score: "<<icp.getFitnessScore()<<std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    
    std::cout<<"Point Cloud Registration Complete"<<std::endl<<std::endl<<std::endl<<std::endl;
    
    sensor_msgs::PointCloud2 temp = sensor_msgs::PointCloud2();
    pcl::toROSMsg (Final, temp);
    pub_merge.publish(temp);
    pcl::toROSMsg (*cloud_in, temp);
    pub_a.publish(temp);
    pcl::toROSMsg (*cloud_out, temp);
    pub_b.publish(temp);
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
    
    pub_merge = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_register/merged_pair", 1);
    pub_a = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_register/a", 1);
    pub_b = nh.advertise<sensor_msgs::PointCloud2> ("/cloud_register/b", 1);
    
    // Spin
    ros::spin ();
}

