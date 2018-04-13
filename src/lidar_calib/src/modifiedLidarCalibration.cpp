// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include "image_geometry/pinhole_camera_model.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_listener.h>
#include <boost/foreach.hpp>

// OpenCV includes
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

// Include pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/passthrough.h>

// Include PointCloud2 ROS message
#include <sensor_msgs/PointCloud2.h>

#include <iostream>
#include <string>

#define IMAGE_WIDTH 964
#define IMAGE_HEIGHT 724

// Topics
static const std::string LIDAR_TOPIC = "/sensors/velodyne_points";
static const std::string IMG_TOPIC = "/sensors/camera/image_color";
static const std::string CAMERA_INFO = "/sensors/camera/camera_info";
static const std::string COMPOSITE_IMG_OUT = "/sensors/camera/lidar_image"; // lidar points on camera image

//static const std::string OPENCV_WINDOW = "Image window";



class ImageConverter {

    ros::NodeHandle nh;

    image_transport::ImageTransport it_;
    image_transport::CameraSubscriber image_sub_; 
    image_transport::Publisher image_pub_;
    tf::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    std::vector<std::string> frame_ids_;



    cv::Mat cameraMatrix, rotationMatrix;
    cv::Mat distCoeffs, rvec, tvec;
    ros::Subscriber lidar_sub;

    std::vector<cv::Point3d> objectPoints;
    // std::vector<cv::Point2d> projectedPoints;

public:
    ImageConverter(const std::vector<std::string>& frame_ids) : it_(nh), frame_ids_(frame_ids) { 

        lidar_sub = nh.subscribe(LIDAR_TOPIC, 1, &ImageConverter::lidarCb, this);
        
        // Subscibe to the camera video
        image_sub_ = it_.subscribeCamera(IMG_TOPIC, 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise(COMPOSITE_IMG_OUT, 1);

        cameraMatrix = cv::Mat(3,3,cv::DataType<double>::type);
        distCoeffs= cv::Mat(5,1,cv::DataType<double>::type);
        rvec = cv::Mat(3,1,cv::DataType<double>::type); // rotation vector      
        tvec = cv::Mat(3,1,cv::DataType<double>::type); // translation vector
        rotationMatrix = cv::Mat(3,3,cv::DataType<double>::type);
// values obtained from task #1 camera calibration
        cameraMatrix.at<double>(0, 0) = 4.8500013227778845e+02;
        cameraMatrix.at<double>(0, 1) = 0.;
        cameraMatrix.at<double>(0, 2) = 4.6048439978206324e+02;
        cameraMatrix.at<double>(1, 0) = 0.;
        cameraMatrix.at<double>(1, 1) = 4.8446529771202120e+02;
        cameraMatrix.at<double>(1, 2) = 3.6882717135520573e+02;
        cameraMatrix.at<double>(2, 0) = 0.;
        cameraMatrix.at<double>(2, 1) = 0.;
        cameraMatrix.at<double>(2, 2) = 1.;


// values obtained from task #1 camera calibration
        distCoeffs.at<double>(0) = -2.1897207538791941e-01;
        distCoeffs.at<double>(1) = 1.1378088445810178e-01;
        distCoeffs.at<double>(2) = 2.7963672438432903e-03;
        distCoeffs.at<double>(3) = 1.2647206581812528e-03;
        distCoeffs.at<double>(4) = -2.7036330701899484e-02;

        tvec.at<double>(0) = -0.05937507;
        tvec.at<double>(1) = -0.48187289; 
        tvec.at<double>(2) = -0.26464405;

        // tvec.at<double>(0) = 0.05937507;
        // tvec.at<double>(1) = -0.48187289; 
        // tvec.at<double>(2) = -0.26464405; 

        rvec.at<double>(0) = 5.41868013;
        rvec.at<double>(1) = 4.49854285;
        rvec.at<double>(2) = 4.46979746;

        cv::Rodrigues(rvec, )

        

      //  cv::namedWindow(OPENCV_WINDOW);

// values obtained from task #1 camera calibration  


    } 

// Destructor
    ~ImageConverter() {
      // cv::destroyWindow(OPENCV_WINDOW);
    }

//////////////////////// Camera Image handling //////////////////////////////////////////
    void imageCb(const sensor_msgs::ImageConstPtr& msg, 
       const sensor_msgs::CameraInfoConstPtr& info_msg) {
        cv_bridge::CvImagePtr cv_ptr;
        cv::Mat image;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            image = cv_ptr->image;
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cam_model_.fromCameraInfo(info_msg);

        BOOST_FOREACH(const std::string& frame_id, frame_ids_) {
          tf::StampedTransform transform;
          try {
            ros::Time acquisition_time = info_msg->header.stamp;
            ros::Duration timeout(1.0 / 30);
            tf_listener_.waitForTransform(cam_model_.tfFrame(), frame_id,
              acquisition_time, timeout);
            tf_listener_.lookupTransform(cam_model_.tfFrame(), frame_id,
               acquisition_time, transform);
        }
        catch (tf::TransformException& ex) {
            ROS_WARN("[draw_frames] TF exception:\n%s", ex.what());
            return;
        }

//        tf::Point pt = transform.getOrigin();
  //      cv::Point3d pt_cv(pt.x(), pt.y(), pt.z());
        cv::Point2d uv;
        uv = cam_model_.project3dToPixel(objectPoints);

        static const int RADIUS = 3;
        cv::circle(image, uv, RADIUS, CV_RGB(255,0,0), -1);
    }

    sensor_msgs::ImagePtr convertedMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
    image_pub_.publish(convertedMsg);
   

}

        //sensor_msgs::ImagePtr convertedMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
        //image_pub_.publish(convertedMsg);


////////////////////////////// LiDAR Point Cloud processing //////////////////////////////////
void lidarCb(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg (*pointCloudMsg, *cloud);


    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZI> pass_x;
    pass_x.setInputCloud (cloud);
    pass_x.setFilterFieldName ("x");
    pass_x.setFilterLimits (0.0, 4.5); // zero to 4.5 meters on the +ve X-axis which is into the world
    pass_x.filter (*cloud_filtered);

    //objectPoints(cloud_filtered->points);
    for (size_t i = 0; i < cloud_filtered->size(); ++i)
    {
        objectPoints(cv::Point3d(cloud_filtered->points[i].x), cv::Point3d(cloud_filtered->points[i].y), cv::Point3d(cloud_filtered->points[i].z));
        //objectPoints(cv::Point3d(cloud_filtered->points.x()), cv::Point3d(cloud_filtered->points.y()), cv::Point3d(cloud_filtered->points.z()));

        //ROS_INFO_STREAM("X: " << cloud_filtered->points[i].x << "  Y: "<< cloud_filtered->points[i].y << "  Z: "<< cloud_filtered->points[i].z);
    }
}

}; // end of class ImageConverter




int main(int argc, char** argv) {
    ROS_INFO("Starting LiDAR node");
    ros::init(argc, argv, "lidar_calibration");
    std::vector<std::string> frame_ids(argv + 1, argv + argc);

    ImageConverter ic(frame_ids);

    ros::spin();
    return 0;
}
