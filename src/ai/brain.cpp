 /**
 *  This file is part of tum_ardrone.
 *
 *  Copyright 2012 Jakob Engel <jajuengel@gmail.com> (Technical University of Munich)
 *  For more information see <https://vision.in.tum.de/data/software/tum_ardrone>.
 *
 *  tum_ardrone is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  tum_ardrone is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with tum_ardrone.  If not, see <http://www.gnu.org/licenses/>.
 */
 
 
#include "ros/ros.h"
#include "ros/package.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Image.h"
#include "tf/tfMessage.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "../HelperFunctions.h"
#include <ardrone_autonomy/Navdata.h>
#include "deque"
#include "tum_ardrone/filter_state.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include <sys/stat.h>
#include <string>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <cv.h>
#include <math.h>
#include "cvd/thread.h"

 using namespace std;
 using namespace cv;

 #define PI 3.1415     

 class BrainNode
 {	

 	void vidCb(const sensor_msgs::ImageConstPtr img);
 	
 	//static pthread_mutex_t send_CS;
 	//RosThread* rosThread;

 	ros::Subscriber vid_sub;
 	ros::Publisher vel_pub;
 	std::string video_channel;
 	ros::NodeHandle nh_;

 public:
 	BrainNode();


 };

 int maxCorners = 230;
 Mat image_prev;
 vector<Point2f> corners_prev;
 bool firsttime = false;
 // pthread_mutex_t BrainNode::send_CS = PTHREAD_MUTEX_INITIALIZER;

 RNG rng(12345);

 void BrainNode::vidCb(const sensor_msgs::ImageConstPtr img)
 {
		//video callback - do something
 	// printf("received video frame\n");


 	//ROS_INFO("received");
 	//ROS_WARN("received");
 	//ROS_ERROR("received");

 	cv::namedWindow("brain view");

 	cv_bridge::CvImagePtr cv_ptr;
 	cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);

 	//if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
 	//	cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));


  /// Parameters for Shi-Tomasi algorithm
 	vector<Point2f> corners;
 	double qualityLevel = 0.001;
 	double minDistance = 10;
 	int blockSize = 3;
 	bool useHarrisDetector = false;
 	double k = 0.04;

 	Mat src_gray;
 	cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );


  /// Copy the source image
 	Mat copy;
 	copy = src_gray.clone();

  // /// Apply corner detection
 	// goodFeaturesToTrack( src_gray,
 	// 	corners,
 	// 	maxCorners,
 	// 	qualityLevel,
 	// 	minDistance,
 	// 	Mat(),
 	// 	blockSize,
 	// 	useHarrisDetector,
 	// 	k );


 	Mat err; 
 	vector<int> status;
 	//ROS_ERROR("here1");
 	//ROS_ERROR("here2");
 	Mat imgA = image_prev;
 	Mat imgB = src_gray;
 	int win_size = 15;
 	Size img_sz = imgA.size();



 	if(firsttime){

 		goodFeaturesToTrack( imgA,corners_prev,maxCorners,qualityLevel,minDistance,Mat());
 		goodFeaturesToTrack( imgB,corners,maxCorners,qualityLevel,minDistance,Mat());

 		cornerSubPix( imgA, corners_prev, Size( win_size, win_size ), Size( -1, -1 ), 
 			TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

 		cornerSubPix( imgB, corners, Size( win_size, win_size ), Size( -1, -1 ), 
 			TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );

	// Call Lucas Kanade algorithm

 		CvSize pyr_sz = Size( img_sz.width+8, img_sz.height/3 );

 		std::vector<uchar> features_found; 
 		features_found.reserve(maxCorners);
 		std::vector<float> feature_errors; 
 		feature_errors.reserve(maxCorners);

 		calcOpticalFlowPyrLK( imgA, imgB, corners_prev, corners, features_found, feature_errors ,
 			Size( win_size, win_size ), 5,
 			cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0 );

	// Make an image of the results

 		for( int i=0; i < features_found.size(); i++ ){
 			cout<<"Error is "<<feature_errors[i]<<endl;
			//continue;

 			cout<<"Got it"<<endl;
 			Point p0( ceil( corners_prev[i].x ), ceil( corners_prev[i].y ) );
 			Point p1( ceil( corners[i].x ), ceil( corners[i].y ) );
 			line( copy, p0, p1, CV_RGB(255,255,255), 2 );
 		}
 	}


 	cv::imshow("brain view", copy);

 	cv::waitKey(3);

	 image_prev = src_gray;//cv_ptr->image;
	 corners_prev.clear();
	 for(int i=0; i<corners.size() ; i++){
	 	corners_prev.push_back(corners[i]);
	 }

	 firsttime = true;


	 //deliberation for motion
	 // TODO: check converstion (!)
	 //ControlCommand c;
	 double sensGaz, sensYaw, sensRP;
	 sensGaz = sensYaw = sensRP = 1;

	// if(isPressed[0]) c.roll = -sensRP; // j
	// if(isPressed[1]) c.pitch = sensRP; // k
	// if(isPressed[2]) c.roll = sensRP; // l
	// if(isPressed[3]) c.pitch = -sensRP; // i
	// if(isPressed[4]) c.yaw = -sensYaw; // u
	// if(isPressed[5]) c.yaw = sensYaw; // o
	// if(isPressed[6]) c.gaz = sensRP; // q
	// if(isPressed[7]) c.gaz = -sensRP; // a

	 //rosThread->sendControlToDrone(c);
	 // pthread_mutex_lock(&send_CS);
	 geometry_msgs::Twist cmdT;
	 // cmdT.angular.z = -cmd.yaw;
	 // cmdT.linear.z = cmd.gaz;
	 // cmdT.linear.x = -cmd.pitch;
	 // cmdT.linear.y = -cmd.roll;

	 // cmdT.angular.x = cmdT.angular.y = gui->useHovering ? 0 : 1;
	 ROS_ERROR("publishing command!");
	 //vel_pub.publish(cmdT);
	 // pthread_mutex_unlock(&send_CS);

	}


	BrainNode::BrainNode(){
		video_channel = nh_.resolveName("ardrone/image_raw");
		vid_sub       = nh_.subscribe(video_channel,10, &BrainNode::vidCb, this);
		vel_pub	   = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("cmd_vel"),1);

		//rosThread = NULL;

	}

	int main(int argc, char **argv)
	{
		ros::init(argc, argv, "drone_brain");

		ROS_INFO("Started RU ArDrone Brain Node.");

		BrainNode brainNode;

		ros::spin();

		return 0;
	}
