#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>

using namespace cv;
using namespace std;



double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
	const vector< vector< Point2f > >& imagePoints,
	const vector< Mat >& rvecs, const vector< Mat >& tvecs,
	const Mat& cameraMatrix , const Mat& distCoeffs) {
	vector< Point2f > imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	vector< float > perViewErrors;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i) {
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float) std::sqrt(err*err/n);
		totalErr += err*err;
		totalPoints += n;
	}
	return std::sqrt(totalErr/totalPoints);
}

int main()
{
    // variables
	int boardWidth = 7;
	int boardHeight = 5;
	int numOfImages = 30; // num of images used for calibration
	float squareSize = 0.05; // 50mm or 5cm
	const string filename = "imgList.yml";
	const string out_file = "calib.yml";
	string img_file[numOfImages];

    // matrices and vectors
	Size imageSize;

	Mat clrImage, grayImage;

	vector< vector < Point3f > > objectCoordinates;
	vector< vector < Point2f > > imageCoordinates;
	vector< Point2f > corners;

	Size boardSize = Size(boardWidth, boardHeight);
	int cornerPoints = boardWidth * boardHeight; // 5 * 7 = 35

	// YAML file operations
	FileStorage fs(filename, FileStorage::READ);
	FileNode n = fs["strings"];

	if (n.type() != FileNode::SEQ)
	{
		cerr << "strings is not a sequence! FAIL" << endl;
		return -1;
	}

	static int index = 0;
	FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
	for (; it != it_end; ++it) {
		img_file[index] = (string)*it;
		index++;
	}

	// actual loop that does finding of the corners
	for (int k = 1; k <= numOfImages; k++) {
		clrImage = imread(img_file[k-1], CV_LOAD_IMAGE_COLOR);
		if(! clrImage.data )                              // Check for invalid input
		{
			cout <<  "Could not open or find the image" << std::endl ;
			return -1;
		}


		cv::cvtColor(clrImage, grayImage, CV_BGR2GRAY);
		/*
		if(k == 15 ) {
			namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
			imshow( "Display window", grayImage );
			waitKey(0);

		}
		*/
		bool found = cv::findChessboardCorners(clrImage, boardSize, corners,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
		if (found)
		{
			cornerSubPix(grayImage, corners, cv::Size(11, 11), cv::Size(-1, -1),
				TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(grayImage, boardSize, corners, found);
		}

		vector< Point3f > obj3D;
		for (int i = 0; i < boardHeight; i++)
			for (int j = 0; j < boardWidth; j++)
				obj3D.push_back(Point3f((float)j * squareSize, (float)i * squareSize, 0));

			if (found) {
				cout << k << ". Found corners!" << endl;
				imageCoordinates.push_back(corners);
				objectCoordinates.push_back(obj3D);
			}
        // for debugging, if 3D points corresponded to 2D points
		//	cout << endl << obj3D.size() << "___" << objectCoordinates.size() <<  "___" << corners.size() <<  "___" << imageCoordinates.size() << endl;

		}

		printf("Starting Calibration\n");
		Mat K = Mat(3, 3, CV_32FC1);
		Mat D;
	// camera aspect ratio is assumed 1
		K.ptr<float>(0)[0] = 1;
		K.ptr<float>(1)[1] = 1;
		vector< Mat > rvecs, tvecs;
		int flag = 0;
		flag |= CV_CALIB_FIX_K4;
		flag |= CV_CALIB_FIX_K5;
		calibrateCamera(objectCoordinates, imageCoordinates, clrImage.size(), K, D, rvecs, tvecs, flag);

		cout << "Calibration error: " << computeReprojectionErrors(objectCoordinates, imageCoordinates, rvecs, tvecs, K, D) << endl;

		fs.open(out_file, FileStorage::WRITE);
		// TODO: write image width and height to calib file
		fs << "camera_matrix" << K;
		fs << "distortion_coefficients" << D;
		

		printf("Done Calibration\n");
		fs.release();
		return 0;
	}
