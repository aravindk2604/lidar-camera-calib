// ROS includes
#include "image_geometry/pinhole_camera_model.h"
#include <boost/foreach.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>

// OpenCV includes
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Include pcl
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// Include PointCloud2 ROS message
#include "pcl_ros/impl/transforms.hpp"
#include "pcl_ros/transforms.h"
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <string>

#define IMAGE_WIDTH 964
#define IMAGE_HEIGHT 724

// Topics
static const std::string LIDAR_TOPIC = "/sensors/velodyne_points";
static const std::string IMG_TOPIC = "/sensors/camera/image_rect_color";
static const std::string CAMERA_INFO = "/sensors/camera/camera_info";
static const std::string COMPOSITE_IMG_OUT =
    "/sensors/camera/lidar_image"; // lidar points on camera image

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter {

  ros::NodeHandle nh;

  ros::Subscriber image_sub_;
  ros::Subscriber info_sub_;
  ros::Subscriber lidar_sub;
  ros::Time cloudHeader;

  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  image_geometry::PinholeCameraModel cam_model_;

  const tf::TransformListener tf_listener_;
  tf::StampedTransform transform;

  std::vector<tf::Vector3> objectPoints;
  tf::Vector3 pt_cv;
  std::vector<cv::Point3d> pt_transformed;

public:
  ImageConverter() : it_(nh) {

    lidar_sub = nh.subscribe(LIDAR_TOPIC, 1, &ImageConverter::lidarCb, this);

    // Subscribe to the camera video
    image_sub_ = nh.subscribe(IMG_TOPIC, 1, &ImageConverter::imageCb, this);
    info_sub_ =
        nh.subscribe(CAMERA_INFO, 1, &ImageConverter::cameraCallback, this);
    image_pub_ = it_.advertise(COMPOSITE_IMG_OUT, 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  // Destructor
  ~ImageConverter() { cv::destroyWindow(OPENCV_WINDOW); }

  void cameraCallback(const sensor_msgs::CameraInfoConstPtr &info_msg) {

    cam_model_.fromCameraInfo(info_msg);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &image_msg) {

    cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

    try {
      cv_ptr =
          cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // cam_model_.fromCameraInfo(info_msg);

    /*	try
            {
                    tf_listener_.lookupTransform("/camera_optical", "/velodyne",
       ros::Time(0), transform);
            }
            catch (tf::TransformException& ex) {
                    ROS_ERROR("%s", ex.what());
                    ros::Duration(1.0).sleep();
                    //continue;
            }
*/
    ros::Time acquisition_time = ros::Time(0);
    ros::Duration timeout(1.0 / 30);

    try {
      //	ros::Time acquisition_time = cloudHeader;
      // ros::Duration timeout(1.0 / 30);
      tf_listener_.waitForTransform("/camera_optical", "/velodyne",
                                    acquisition_time, timeout);
      tf_listener_.lookupTransform("/camera_optical", "/velodyne",
                                   acquisition_time, transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      // ros::Duration(1.0).sleep();
    }

    // tranform the xyz point from pointcoud
    for (size_t i = 0; i < objectPoints.size(); ++i) {

      pt_cv = transform(objectPoints[i]);

      pt_transformed.push_back(cv::Point3d(pt_cv.x(), pt_cv.y(), pt_cv.z()));
      cv::Point2d uv;
      uv = cam_model_.project3dToPixel(pt_transformed[i]);

      if (uv.x >= 0 && uv.x <= IMAGE_WIDTH && uv.y >= 0 &&
          uv.y <= IMAGE_HEIGHT) {
        static const int RADIUS = 3;
        cv::circle(cv_ptr->image, uv, RADIUS, CV_RGB(255, 0, 0));
      }
    }
    pt_cv.setZero();
    pt_transformed.clear();
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(1);

    sensor_msgs::ImagePtr convertedMsg =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image)
            .toImageMsg();
    image_pub_.publish(convertedMsg);
  }
  ///////////////////////////////////// LIDAR CALLBACK
  /////////////////////////////////

  void lidarCb(const sensor_msgs::PointCloud2ConstPtr &pointCloudMsg) {
    cloudHeader = pointCloudMsg->header.stamp;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*pointCloudMsg, *cloud);

    // Create the  filtering object
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(0.0, 4.5); // 0 to 4.5 mts limitation
    pass_x.filter(*cloud_filtered);

    objectPoints.clear();
    for (size_t i = 0; i < cloud_filtered->size(); ++i) {
      objectPoints.push_back(tf::Vector3(cloud_filtered->points[i].x,
                                         cloud_filtered->points[i].y,
                                         cloud_filtered->points[i].z));
      // ROS_INFO_STREAM("X: " << objectPoints[i].x() << "  Y: "<<
      // objectPoints[i].y() << "  Z: "<< objectPoints[i].z());
    }
  }
};

int main(int argc, char **argv) {

  ROS_INFO("Starting LiDAR node");
  ros::init(argc, argv, "lidar_calibration");

  ImageConverter ic;
  ros::spin();
  return 0;
}