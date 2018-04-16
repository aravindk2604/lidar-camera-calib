/**
 * This program takes a set of corresponding 2D and 3D points and finds the transformation matrix 
 * that best brings the 3D points to their corresponding 2D points.
 */
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
 
#include <iostream>
#include <string>
 
std::vector<cv::Point2f> Generate2DPoints();
std::vector<cv::Point3f> Generate3DPoints();
 
int main( int argc, char* argv[])
{
  // Read points
  std::vector<cv::Point2f> imagePoints = Generate2DPoints();
  std::vector<cv::Point3f> objectPoints = Generate3DPoints();
 
  std::cout << "There are " << imagePoints.size() << " imagePoints and " << objectPoints.size() << " objectPoints." << std::endl;
  cv::Mat cameraMatrix(3,3,cv::DataType<double>::type);
  cv::Mat xyz(3,3,cv::DataType<double>::type);

  //cv::setIdentity(cameraMatrix);

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


 
  std::cout << "Initial cameraMatrix: \n" << cameraMatrix << std::endl;
 
  cv::Mat distCoeffs(5,1,cv::DataType<double>::type);

// values obtained from task #1 camera calibration
  distCoeffs.at<double>(0) = -2.1897207538791941e-01;
  distCoeffs.at<double>(1) = 1.1378088445810178e-01;
  distCoeffs.at<double>(2) = 2.7963672438432903e-03;
  distCoeffs.at<double>(3) = 1.2647206581812528e-03;
  distCoeffs.at<double>(4) = -2.7036330701899484e-02;
  

  cv::Mat rvec(3,1,cv::DataType<double>::type);
  cv::Mat abc(3,1,cv::DataType<double>::type);

  cv::Mat tvec(3,1,cv::DataType<double>::type);

// values used from running after one iteration
  // tvec.at<double>(0) = -0.05937507;
  // tvec.at<double>(1) = -0.48187289; 
  // tvec.at<double>(2) = -0.26464405; 
////////////////////////////////////////////////////
  //  rvec.at<double>(0) =   2.46979746;
  // rvec.at<double>(1) =  4.49854285;
  // rvec.at<double>(2) =  5.41868013;


//////////////////////////////////////////////////




  // rvec.at<double>(0) = 5.41868013;
  // rvec.at<double>(1) = 4.49854285;
  // rvec.at<double>(2) = 2.46979746;

  // tvec.at<double>(0) = 0.4240347073463082;
  // tvec.at<double>(1) = -0.09119832617429594; 
  // tvec.at<double>(2) = -1.764738961137057; 

  // rvec.at<double>(0) = 0.9533470811110965;
  // rvec.at<double>(1) = -1.501583131508996;
  // rvec.at<double>(2) = 1.513657927546317;


  ///
  //  tvec.at<double>(0) = -0.4142752012282254;
  // tvec.at<double>(1) =   -0.3466738762524834; 
 
  // tvec.at<double>(2) = -3.365232380983624; 

 // rvec.at<double>(0) =  8.358138515105635;
 //  rvec.at<double>(1) =   5.217670625741007;
 //  rvec.at<double>(2) =  3.545116283922075;


  // rvec.at<double>(0) =     2.203427738539934;
  // rvec.at<double>(1) =   -0.04443768090240761;
  // rvec.at<double>(2) =   -1.886260535251167;


  

  // tvec.at<double>(0) = 0.3192214067855315;
  // tvec.at<double>(1) =  -1.066795755674321; 
  // tvec.at<double>(2) = -3.197155619420194 ;

 // rvec.at<double>(0) =   2.483162023330979;
 //  rvec.at<double>(1) =  6.903274718145067;
 //  rvec.at<double>(2) =  6.810895482760497;


  //  rvec.at<double>(0) =  -1.209199576156146;
  // rvec.at<double>(1) =  -1.209199576156146;
  // rvec.at<double>(2) = -1.209199576156146;



  tvec.at<double>(0) = -0.8;
  tvec.at<double>(1) =  1.2; 
  tvec.at<double>(2) =  -0.5;
 rvec.at<double>(0) =  -1.209199576156146;
  rvec.at<double>(1) =  -1.209199576156146;
  rvec.at<double>(2) = -1.209199576156146;




  xyz.at<double>(0, 0) = 0;
  xyz.at<double>(0, 1) = 1;
  xyz.at<double>(0, 2) = 0;
  xyz.at<double>(1, 0) = 0;
  xyz.at<double>(1, 1) = 0;
  xyz.at<double>(1, 2) = 1;
  xyz.at<double>(2, 0) = 1;
  xyz.at<double>(2, 1) = 0;
  xyz.at<double>(2, 2) = 0;

cv::Rodrigues(xyz, abc);
 
 
  cv::solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec, true, CV_ITERATIVE);
 
  std::cout << "rvec: " << rvec << std::endl;

  std::cout << "tvec: " << tvec << std::endl;
 
  std::cout << "abc: " << abc << std::endl;




  std::vector<cv::Point2f> projectedPoints;
  cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, distCoeffs, projectedPoints);
 
  for(unsigned int i = 0; i < projectedPoints.size(); ++i)
    {
    std::cout << "Image point: " << imagePoints[i] << " Projected to " << projectedPoints[i] << std::endl;
    }


    std::cout << "obj: " << objectPoints.size() << " img: " << projectedPoints.size() << std::endl;

  //cv::Rodrigues()
 
  return 0;
}
 
 
std::vector<cv::Point2f> Generate2DPoints()
{
  std::vector<cv::Point2f> points;
 
  float x,y;
 
  x=309;y=315;
  points.push_back(cv::Point2f(x,y));
 
  x=304;y=433;
  points.push_back(cv::Point2f(x,y));
 
  x=491;y=436;
  points.push_back(cv::Point2f(x,y));
 
  x=490;y=321;
  points.push_back(cv::Point2f(x,y));
 
  x=426;y=286;
  points.push_back(cv::Point2f(x,y));
 
  x=253;y=401;
  points.push_back(cv::Point2f(x,y));
 
  //x=566;y=475;
  //points.push_back(cv::Point2f(x,y));
 
  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
 
  return points;
}
 
 
std::vector<cv::Point3f> Generate3DPoints()
{
  std::vector<cv::Point3f> points;
 
 
  float x,y,z;
 
  x=1.568;y=0.159;z=-0.082;
  points.push_back(cv::Point3f(x,y,z));
 
  x=1.733;y=0.194;z=-0.403;
  points.push_back(cv::Point3f(x,y,z));
 
  x=1.595;y=-0.375;z=-0.378;
  points.push_back(cv::Point3f(x,y,z));
 
  x=1.542;y=-0.379;z=-0.083;
  points.push_back(cv::Point3f(x,y,z));
 
  x=1.729;y=-0.173;z=0.152;
  points.push_back(cv::Point3f(x,y,z));
 
  x=3.276;y=0.876;z=-0.178;
  points.push_back(cv::Point3f(x,y,z));
 
  //x=-.5;y=-.5;z=.5;
  //points.push_back(cv::Point3f(x,y,z));
 
  for(unsigned int i = 0; i < points.size(); ++i)
    {
    std::cout << points[i] << std::endl;
    }
 
  return points;
}