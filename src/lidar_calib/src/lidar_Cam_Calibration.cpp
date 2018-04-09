// ROS includes
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include "image_geometry/pinhole_camera_model.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

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
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs, rvec, tvec;
    ros::Subscriber lidar_sub;

    std::vector<cv::Point3d> objectPoints;
    std::vector<cv::Point2d> projectedPoints;

public:
    ImageConverter() : it_(nh) { 

        lidar_sub = nh.subscribe(LIDAR_TOPIC, 1, &ImageConverter::lidarCb, this);
        
        // Subscibe to the camera video
        image_sub_ = it_.subscribeCamera(IMG_TOPIC, 1, &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise(COMPOSITE_IMG_OUT, 1);


        

        cameraMatrix = cv::Mat(3,3,cv::DataType<double>::type);
        distCoeffs= cv::Mat(5,1,cv::DataType<double>::type);
        rvec = cv::Mat(3,1,cv::DataType<double>::type); // rotation vector      
        tvec = cv::Mat(3,1,cv::DataType<double>::type); // translation vector

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

        // rvec.at<double>(0) = 0;
        // rvec.at<double>(1) = 0;
        // rvec.at<double>(2) = 0;

        // tvec.at<double>(0) = 0.4240347073463082;
        // tvec.at<double>(1) = -0.09119832617429594; 
        // tvec.at<double>(2) = -1.764738961137057; 

        // rvec.at<double>(0) = 0.9533470811110965;
        // rvec.at<double>(1) = 1.501583131508996;
        // rvec.at<double>(2) = 1.513657927546317;

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

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);

        for(size_t i = 0; i< projectedPoints.size(); ++i) {
            //ROS_INFO_STREAM ("PPsize: " << projectedPoints.size());
            if(projectedPoints[i].x >= 0 && projectedPoints[i].x < IMAGE_WIDTH && projectedPoints[i].y >= 0 && projectedPoints[i].y <IMAGE_HEIGHT)
            {   
                // draw circles on all the projectedPoints to represent the lidar data
                cv::circle(cv_ptr->image, cv::Point((int)projectedPoints[i].x,(int)projectedPoints[i].y), 4, CV_RGB(255,0,0));
                //ROS_INFO_STREAM(" x " << projectedPoints[i].x << " y " << projectedPoints[i].y);
                //cv::imshow(OPENCV_WINDOW,cv_ptr->image);
                //cv::waitKey(3);
                sensor_msgs::ImagePtr convertedMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
                image_pub_.publish(convertedMsg);
            }   

        }

        //sensor_msgs::ImagePtr convertedMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
        //image_pub_.publish(convertedMsg);

    }
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


    for (size_t i = 0; i < cloud_filtered->size(); ++i)
    {
        objectPoints.push_back(cv::Point3f(cloud_filtered->points[i].x, cloud_filtered->points[i].y,
           cloud_filtered->points[i].z));

        //ROS_INFO_STREAM("X: " << cloud_filtered->points[i].x << "  Y: "<< cloud_filtered->points[i].y << "  Z: "<< cloud_filtered->points[i].z);
    }
}

}; // end of class ImageConverter




int main(int argc, char** argv) {
    ROS_INFO("Starting LiDAR node");
    ros::init(argc, argv, "lidar_calibration");

    ImageConverter ic;

    ros::spin();
    return 0;
}
