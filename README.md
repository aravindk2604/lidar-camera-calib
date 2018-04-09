# LiDAR-Camera Calibration

There are three tasks to be completed.  

* Monocular Camera Calibration  
* Camera -- LiDAR cross calibration  
* Visualization of data in point cloud  

---
## Task 1: Monocular Camera Calibration  
Perform Camera Calibration for the monocular camera data that is stored in the rosbag provided.  
The ROS wiki provides a good description on  [How to Calibrate a Monocular Camera](https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) .  
By running the `cameracalibrator.py` node in the terminal, I was able to get the calibration parameters saved in a yaml file. Before we get to that we must inspect the rosbag for the type of messages that we are to expect.

###### Run this in the terminal  
```
rosbag info 2016-11-22-14-32-13_test.bag
```


* I get the following output:

```
path:        2016-11-22-14-32-13_test.bag
version:     2.0
duration:    1:53s (113s)
start:       Nov 22 2016 14:32:14.41 (1479853934.41)
end:         Nov 22 2016 14:34:07.88 (1479854047.88)
size:        3.1 GB
messages:    5975
compression: none [1233/1233 chunks]
types:       sensor_msgs/CameraInfo  [c9a58c1b0b154e0e6da7578cb991d214]
             sensor_msgs/Image       [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/PointCloud2 [1158d486dd51d683ce2f1be655c3c181]
topics:      /sensors/camera/camera_info   2500 msgs    : sensor_msgs/CameraInfo 
             /sensors/camera/image_color   1206 msgs    : sensor_msgs/Image      
             /sensors/velodyne_points      2269 msgs    : sensor_msgs/PointCloud2
```


This rosbag data consists of the monocular camera data and velodyne lidar data. The concerned __topics__ for this task are `/sensors/camera/camera_info`  and  `/sensors/camera/image_color` .  

I played the rosbag data and used the `cameracalibrator.py` node to get a GUI that started performing the calibration as the camera data played. I calibrated and saved the parameters.


* The command to play the rosbag is:
```
$ rosbag play 2016-11-22-14-32-13_test.bag
```
  

* The command to run the `cameracalibrator.py` node is:
```
$ rosrun camera_calibration cameracalibrator.py --size=5x7 --square=0.050 image:=/sensors/camera/image_color camera:=/sensors/camera/camera_info  --no-service-check
```

> The checker board pattern had the dimensions 5x7 corners and each square was 5cm.


* The calibration parameters obtained by running the above command is as follows:
```
image_width: 964
image_height: 724
camera_name: narrow_stereo
camera_matrix:
  rows: 3
  cols: 3
  data: [484.093389, 0.000000, 456.852175, 0.000000, 483.969563, 365.996130, 0.000000, 0.000000, 1.000000]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.198634, 0.067562, 0.003059, 0.000139, 0.000000]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.000000, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000, 0.000000, 0.000000, 1.000000]
projection_matrix:
  rows: 3
  cols: 4
  data: [424.457428, 0.000000, 460.282975, 0.000000, 0.000000, 431.685059, 369.980132, 0.000000, 0.000000, 0.000000, 1.000000, 0.000000]
```


I wrote a C++ code to get the camera calibration parameters and used this [OpenCV documentation on Camera Calibration](https://docs.opencv.org/2.4/doc/tutorials/calib3d/camera_calibration/camera_calibration.html
) as a reference.

You can find this file under `src/cam_calib/src/images/camCalibrationParameters.cpp` . I screen-grabbed 30 images from the rosbag and used a yml file to read in the list of images for camera calibration process. 

> It is important to know that a `.jpg` image output will be different from a `.png` output while calibrating the camera. I realised this when I was working with `.png` files and the camera calibration paramenters always showed zeros.

I use the terminal to run my code and the command is:
```
$ g++ -std=c++11 camCalibrationParameters.cpp `pkg-config --libs --cflags opencv` -o output

$ ./output
```

The output of my code is stored in a yml file which is as follows:

```
%YAML:1.0
---
camera_matrix: !!opencv-matrix
   rows: 3
   cols: 3
   dt: d
   data: [ 4.8500013227778845e+02, 0., 4.6048439978206324e+02, 0.,
       4.8446529771202120e+02, 3.6882717135520573e+02, 0., 0., 1. ]
distortion_coefficients: !!opencv-matrix
   rows: 1
   cols: 5
   dt: d
   data: [ -2.1897207538791941e-01, 1.1378088445810178e-01,
       2.7963672438432903e-03, 1.2647206581812528e-03,
       -2.7036330701899484e-02 ]
       
```


##### TODO
- [ ] explain image_proc
- [ ] bag_tools used for recording new rosbag with camera calibration
- [ ] unavailability of bag_tools for ROS Kinetic
- [ ] rectified image output

The [rectified image output](_videos/rectified_camera_output.mp4) video is placed inside the `_videos` directory.
