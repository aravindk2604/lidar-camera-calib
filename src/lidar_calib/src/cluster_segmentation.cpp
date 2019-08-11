#include <ros/ros.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  // Container for original & filtered data
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud(cloudPtr);
  sor.setLeafSize(0.001, 0.001, 0.001);
  sor.filter(cloud_filtered);

  pcl::PointCloud<pcl::PointXYZ> point_cloud;
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloudPtr(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_filtered, point_cloud);
  pcl::copyPointCloud(point_cloud, *point_cloudPtr);

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(point_cloudPtr);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1); // 2cm
  ec.setMinClusterSize(100);   // 100
  ec.setMaxClusterSize(99000000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(point_cloudPtr);
  ec.extract(cluster_indices);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_segmented(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  int j = 0;

  for (std::vector<pcl::PointIndices>::const_iterator it =
           cluster_indices.begin();
       it != cluster_indices.end(); ++it) {
    for (std::vector<int>::const_iterator pit = it->indices.begin();
         pit != it->indices.end(); ++pit) {
      pcl::PointXYZRGB point;
      point.x = point_cloudPtr->points[*pit].x;
      point.y = point_cloudPtr->points[*pit].y;
      point.z = point_cloudPtr->points[*pit].z;

      if (j == 0) // Red	#FF0000	(255,0,0)
      {
        point.r = 0;
        point.g = 0;
        point.b = 255;
      } else if (j == 1) // Lime	#00FF00	(0,255,0)
      {
        point.r = 0;
        point.g = 255;
        point.b = 0;
      } else if (j == 2) // Blue	#0000FF	(0,0,255)
      {
        point.r = 255;
        point.g = 0;
        point.b = 0;
      } else if (j == 3) // Yellow	#FFFF00	(255,255,0)
      {
        point.r = 255;
        point.g = 255;
        point.b = 0;
      } else if (j == 4) // Cyan	#00FFFF	(0,255,255)
      {
        point.r = 0;
        point.g = 255;
        point.b = 255;
      } else if (j == 5) // Magenta	#FF00FF	(255,0,255)
      {
        point.r = 255;
        point.g = 0;
        point.b = 255;
      } else if (j == 6) // Olive	#808000	(128,128,0)
      {
        point.r = 128;
        point.g = 128;
        point.b = 0;
      } else if (j == 7) // Teal	#008080	(0,128,128)
      {
        point.r = 0;
        point.g = 128;
        point.b = 128;
      } else if (j == 8) // Purple	#800080	(128,0,128)
      {
        point.r = 128;
        point.g = 0;
        point.b = 128;
      } else {
        if (j % 2 == 0) {
          point.r = 255 * j / (cluster_indices.size());
          point.g = 128;
          point.b = 50;
        } else {
          point.r = 0;
          point.g = 255 * j / (cluster_indices.size());
          point.b = 128;
        }
      }
      point_cloud_segmented->push_back(point);
    }
    j++;
  }
  std::cerr << "segemnted:  " << (int)point_cloud_segmented->size() << "\n";
  std::cerr << "origin:     " << (int)point_cloudPtr->size() << "\n";
  // Convert to ROS data type
  point_cloud_segmented->header.frame_id = point_cloudPtr->header.frame_id;
  if (point_cloud_segmented->size())
    pcl::toPCLPointCloud2(*point_cloud_segmented, cloud_filtered);
  else
    pcl::toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_filtered, output);

  // Publish the data
  pub.publish(output);
}

int main(int argc, char **argv) {
  // Initialize ROS
  ros::init(argc, argv, "cluster_segmentation");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("/sensors/velodyne_points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub =
      nh.advertise<sensor_msgs::PointCloud2>("/sensors/new_velodyne_output", 1);

  // Spin
  ros::spin();
}