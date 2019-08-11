/*
 * PCL Example using ROS and CPP
 */

// Include the ROS library
#include <ros/ros.h>

// Include pcl
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// Include PointCloud2 message
#include <iostream>
#include <sensor_msgs/PointCloud2.h>

// Topics
static const std::string IMAGE_TOPIC = "/sensors/velodyne_points";
// static const std::string PUBLISH_TOPIC = "/pcl/points";

// ROS Publisher
// ros::Publisher pub;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZI> pass_x;
  // pcl::PassThrough<pcl::PointXYZI> pass_y;
  // pcl::PassThrough<pcl::PointXYZI> pass_z;

  pass_x.setInputCloud(cloud);
  // pass_y.setInputCloud (cloud);
  // pass_z.setInputCloud (cloud);

  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(
      0.0, 4.5); // zero to 4.5 meters on the +ve X-axis which is into the world
  pass_x.filter(*cloud_filtered);
  /*
          pass_y.setFilterFieldName ("y");
          pass_y.setFilterLimits(-4.5, 4.5); // limiting between - and + 4.5
     metres of data pass_y.filter (*cloud_filtered);

          pass_z.setFilterFieldName ("z");
          pass_z.setFilterLimits(-4.5, 4.5); // limiting between - and + 4.5
     metres of data pass_z.filter (*cloud_filtered);
  */

  // std::cout << "size: " << cloud->size() << std::endl; //"  hei: " <<
  // cloud->height << std::endl;

  for (size_t i = 0; i < cloud_filtered->size(); ++i) {
    ROS_INFO_STREAM("X: " << cloud_filtered->points[i].x
                          << "  Y: " << cloud_filtered->points[i].y
                          << "  Z: " << cloud_filtered->points[i].z);
    // std::cout << "    " << cloud_msg->data[i] << std::endl;
    //" "    << cloud->data[i].y << " "    << cloud->data[i].z << std::endl;
  }

  // Perform the actual filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.1, 0.1, 0.1);
  // sor.filter (cloud_filtered);

  // Convert to ROS data type
  // sensor_msgs::PointCloud2 output;
  // pcl_conversions::moveFromPCL(cloud_filtered, output);

  // Publish the data
  //	pub.publish (output);
}

int main(int argc, char **argv) {
  // Initialize the ROS Node "roscpp_pcl_example"
  ros::init(argc, argv, "roscpp_pcl_example");
  ros::NodeHandle nh;

  // Print "Hello" message with node name to the terminal and ROS log file
  //	ROS_INFO_STREAM("Hello from ROS Node: " << ros::this_node::getName());

  // Create a ROS Subscriber to IMAGE_TOPIC with a queue_size of 1 and a
  // callback function to cloud_cb
  ros::Subscriber sub = nh.subscribe(IMAGE_TOPIC, 1, cloud_cb);

  // Create a ROS publisher to PUBLISH_TOPIC with a queue_size of 1
  //	pub = nh.advertise<sensor_msgs::PointCloud2>(PUBLISH_TOPIC, 1);

  // Spin
  ros::spin();

  // Success
  return 0;
}