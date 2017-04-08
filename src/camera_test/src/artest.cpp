#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <vector>

/*
static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}*/

using namespace cv;
using namespace cv::aruco;
using namespace std;

#define DISP_SQUARE 0.0873125
#define DISP_CORNER 0.06746875

Point2f marker_disps[] = {
	Point2f(0*DISP_SQUARE, 0*DISP_SQUARE), Point2f(1*DISP_SQUARE, 0*DISP_SQUARE),
	Point2f(0*DISP_SQUARE,-1*DISP_SQUARE), Point2f(1*DISP_SQUARE,-1*DISP_SQUARE),
	Point2f(0*DISP_SQUARE,-2*DISP_SQUARE), Point2f(1*DISP_SQUARE,-2*DISP_SQUARE)
};
Point2f corner_disps[] = {
	Point2f(0*DISP_CORNER, 0*DISP_CORNER), Point2f(1*DISP_CORNER, 0*DISP_CORNER),
	Point2f(1*DISP_CORNER,-1*DISP_CORNER), Point2f(0*DISP_CORNER,-1*DISP_CORNER)
};


bool robertEstimatePose(vector< vector<Point2f> > &markerCorners, vector<int> &markerIds, GridBoard &board, Mat &cameraMatrix, Mat &distCoeffs, Vec3d &rvec, Vec3d &tvec){
	vector< vector<Point3f> > world_coords;
	for(int i = 0; i < 6; i++){
		vector<Point3f>* corners = new vector<Point3f>();
		for(int j = 0; j < 4; j++){
			corners->push_back(Point3f(marker_disps[i].x+corner_disps[j].x,marker_disps[i].y+corner_disps[j].y, 0));
		}
		world_coords.push_back(*corners);
		delete corners;
	}
	vector<Point3f> objectPoints;
	vector<Point2f> imagePoints;
	vector<int>::iterator i;
	vector< vector<Point2f> >::iterator j;
	for(i = markerIds.begin(), j = markerCorners.begin(); i < markerIds.end(), j < markerCorners.end(); i++, j++){
		for(int k = 0; k < 4; k++){
			objectPoints.push_back(world_coords[*i][k]);
			imagePoints.push_back((*j)[k]);
		}
	}
	//cout<<objectPoints<<endl;
	//cout<<imagePoints<<endl;
	return solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
}

int main(int argc, char** argv)
{
	if(argc < 2){
		printf("Supply a calibration file\n");
		return -1;
	}
	VideoCapture cap(0);
	if(!cap.isOpened())
		return -1;

	namedWindow("board",1);
	Ptr<Dictionary> dict = getPredefinedDictionary(DICT_6X6_250);
	Ptr<GridBoard> board = GridBoard::create(2,3,0.10,0.03,dict,0);
	Mat boardImg;
	board->draw(Size(600,500), boardImg, 10, 1);
	imshow("board", boardImg);
	namedWindow("live", WINDOW_NORMAL);

	FileStorage fs(String(argv[1]), FileStorage::READ);
	Mat cameraMatrix, distCoeffs;
	fs["Camera_Matrix"] >> cameraMatrix;
	fs["Distortion_Coefficients"] >> distCoeffs;
	//cout<<"cameraMatrix"<<endl<<cameraMatrix<<endl;
	//cout<<"distCoeffs"<<endl<<distCoeffs<<endl;

	while(true)
	{
		Mat img;
		cap >> img;
		vector<int> markerIds;
		vector< vector<Point2f> > markerCorners;
		detectMarkers(img, board->dictionary, markerCorners, markerIds);
		drawDetectedMarkers(img, markerCorners, markerIds);
		if(markerIds.size() > 0){
			Vec3d rvec, tvec;
			rvec(0) = rvec(1) = rvec(2) = 0;
			tvec(0) = tvec(1) = tvec(2) = 0;
			bool valid = robertEstimatePose(markerCorners, markerIds, *board, cameraMatrix, distCoeffs, rvec, tvec);
			//cout << "rvec" << rvec << endl;
			//cout << "tvec" << tvec << endl;
			/*
			ostringstream ss;
			ss << "Used in pose estimation: " << valid;
			putText(img, ss.str(), cvPoint(30,30),
    			FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
					*/
			if(valid){
				drawAxis(img, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
			}
		}

		imshow("live", img);

		if(waitKey(5) != 255) break;
	}
	return 0;
}
