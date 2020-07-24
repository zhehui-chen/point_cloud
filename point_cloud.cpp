#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PointStamped.h>

// PCL specific includes
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/icp.h>

//filter
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>


typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

std::string pcd_file_path = "/home/ncrl/cloud_ws/src/point_cloud/src/ITRI/map.pcd";
ros::Publisher cloud_pub, map_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map (new pcl::PointCloud<pcl::PointXYZ>);

bool read_gps_first_data = false, operation = false;

struct position
{
  float x;
  float y;
  float z;
}GPS;

void gps_info(const geometry_msgs::PointStamped::ConstPtr& gps)
{
  if(read_gps_first_data == false)
  {
    GPS.x = gps->point.x;
    GPS.y = gps->point.y;
    GPS.z = gps->point.z;
    read_gps_first_data = true;
  }
}

void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
  printf ("Rotation matrix :\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
  printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
  printf ("Translation vector :\n");
  printf ("t = < %6.3f, %6.3f, %6.3f >", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

void transform_point(const sensor_msgs::PointCloud2::ConstPtr& point)
{
  if (read_gps_first_data == true && operation == false)
  {
    pcl::PCLPointCloud2::Ptr pcl_point_unfiltered (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr pcl_point_filtered (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr pcl_map_unfiltered (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr pcl_map_filtered (new pcl::PCLPointCloud2 ());

    // trans from sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl_conversions::toPCL(*point, *pcl_point_unfiltered);
    // trans from pcl::PCLPointCloud to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2(*pcl_map, *pcl_map_unfiltered);

//    // the filtering object
//    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
//    sor.setInputCloud(pcl_map_unfiltered);
//    sor.setLeafSize(0.5f, 0.5f, 0.5f);
//    sor.filter(*pcl_map_filtered);

//    sor.setInputCloud(pcl_point_unfiltered);
//    sor.setLeafSize(0.5f, 0.5f, 0.5f);
//    sor.filter(*pcl_point_filtered);

    // The point clouds we will be using
    PointCloudT::Ptr cloud_in (new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr (new PointCloudT);  // Transformed point cloud // used for backup data
    PointCloudT::Ptr cloud_icp (new PointCloudT);  // ICP output point cloud

    // trans from pcl::PCLPointCloud2 to pcl::PCLPointCloud
    pcl::fromPCLPointCloud2(*pcl_map_unfiltered, *cloud_in);
    pcl::fromPCLPointCloud2(*pcl_point_unfiltered, *cloud_icp);

    // filter for z direction
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("z");
//    pass.setFilterLimits(-8.0, -5.0);
    pass.setFilterLimits(-2.0, 1.0);
    pass.setFilterLimitsNegative(false);//设置过滤器限制负//设置保留范围内false
    pass.filter(*cloud_in);

    pass.setInputCloud(cloud_icp);
//    pass.setFilterLimits(-0.5, 2.0);
    pass.setFilterLimits(-2.0, 1.0);
    pass.setFilterLimitsNegative(false);//设置过滤器限制负//设置保留范围内false
    pass.filter(*cloud_icp);

//    // A translation on XYZ axis from body frame to map frame
//    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
//    transformation_matrix (0, 3) = GPS.x;
//    transformation_matrix (1, 3) = GPS.y;
//    transformation_matrix (2, 3) = GPS.z;
//    std::cout << "GPS: " << GPS.x << " " << GPS.y << " "<< GPS.z << " "<< std::endl;
//    print4x4Matrix(transformation_matrix);

//    // Executing the transformation
//    pcl::transformPointCloud (*cloud_icp, *cloud_icp, transformation_matrix);
    *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
    std::cout << "\ntransformPointCloud: " << cloud_icp->points[0] << std::endl;

    // opti transformation
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    Eigen::Matrix4d opti_transformation = Eigen::Matrix4d::Identity();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    int iterations = 100;
    float rotation_angle = 15;
    float theta = (2*M_PI)/(360/rotation_angle);
    float score = FLT_MAX;
    float opti_score = FLT_MAX;
    float opti_angle = 0;
    sensor_msgs::PointCloud2 output_cloud, output_map;


    // rotation per 15 degree
//    for (int i=0; i < 360/rotation_angle; i++)
    for (int i=7; i < 14; i++)
    {
      *cloud_icp = *cloud_tr;
      transformation_matrix = Eigen::Matrix4d::Identity();
      transformation_matrix (0, 0) = cos (theta*i);
      transformation_matrix (0, 1) = -sin (theta*i);
      transformation_matrix (1, 0) = sin (theta*i);
      transformation_matrix (1, 1) = cos (theta*i);
      transformation_matrix (0, 3) = GPS.x;
      transformation_matrix (1, 3) = GPS.y;
      transformation_matrix (2, 3) = GPS.z;

      pcl::transformPointCloud(*cloud_icp, *cloud_icp, transformation_matrix);

      icp.setMaximumIterations (iterations);
      icp.setInputSource (cloud_icp);
      icp.setInputTarget (cloud_in);
      icp.align (*cloud_icp);

      if (icp.hasConverged ())
      {
        score = icp.getFitnessScore ();
        printf ("\nIn loop %d ICP has converged, score is %+.0e\n", i, icp.getFitnessScore ());

        std::cout << "Angle is " << rotation_angle*i << std::endl;
        transformation_matrix = icp.getFinalTransformation ().cast<double>()*transformation_matrix;  // WARNING /!\ This is not accurate! For "educational" purpose only!
        print4x4Matrix (transformation_matrix);  // Print the transformation between original pose and current pose
      }
      else
      {
        PCL_ERROR ("\nIn loop %d ICP has not converged.\n", i);
      }

      if (score < opti_score)
      {
        opti_score = score;
        opti_angle = rotation_angle*i;
        opti_transformation = transformation_matrix;
      }
      std::cout << "===============================" << std::endl;
//      sensor_msgs::PointCloud2 output_cloud;
      pcl::toROSMsg(*cloud_icp, output_cloud);
      output_cloud.header.frame_id = "point_cloud";
      cloud_pub.publish(output_cloud);
    }

    // output the optimal result
    std::cout << "\nThe optimized angle is: " << opti_angle << std::endl;
    std::cout << "\nThe optimized score is: " << opti_score << std::endl;
    print4x4Matrix (opti_transformation);
    pcl::transformPointCloud (*cloud_tr, *cloud_icp, opti_transformation);

    std::cout << "\noutput to rviz" << std::endl;
//    sensor_msgs::PointCloud2 output_cloud, output_map;
    pcl::toROSMsg(*cloud_icp, output_cloud);
    pcl::toROSMsg(*cloud_in, output_map);
    output_cloud.header.frame_id = "point_cloud";
    output_map.header.frame_id = "point_cloud";

    cloud_pub.publish(output_cloud);
    map_pub.publish(output_map);

    operation = true;
  }
//  read_lidar_first_data = true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud");
  ros::NodeHandle n;
  cloud_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud", 1000);
  map_pub = n.advertise<sensor_msgs::PointCloud2>("point_cloud_map", 1000);

  pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file_path, *pcl_map); //* load the file

  ros::Subscriber fix_sub = n.subscribe("fix", 1000, gps_info);
  ros::Subscriber point_sub = n.subscribe("lidar_points", 1000, transform_point);

  ros::spin();
  return 0;
}
