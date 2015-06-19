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
 	static int loopCount;
 	static float avg_flow_a;
 	static float avg_flow_c;
 	static float avg_flow_b;



 };

 int maxCorners = 230;
 Mat image_prev;
 vector<Point2f> corners_prev;
 bool firsttime = false;

 //divide image up into 3rds
 int width = 640;
 int height = 320;
 int a_left = 0;
 int a_right = width/3;
 int b_left = width/3;
 int b_right = (width/3) * 2;
 int c_left = (width/3) * 2;
 int c_right = width;
 // pthread_mutex_t BrainNode::send_CS = PTHREAD_MUTEX_INITIALIZER;

 RNG rng(12345);

 float euc_dist( float x1, float y1, float x2, float y2)
 {
 	float deltaX = fabs(x2 - x1);
 	float deltaY = (y2 - y1);
 	return sqrt(deltaX * deltaX + deltaY * deltaY);
 } 

 bool sentCommand=false;
 int BrainNode::loopCount = 0;
 float BrainNode::avg_flow_a=0, BrainNode::avg_flow_b=0, BrainNode::avg_flow_c=0;


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
 		vector<float> x_coords_a,x_coords_b,x_coords_c;
 		float flow_mag_a, flow_mag_b, flow_mag_c;
 		float sum_flow_a=0, sum_flow_b=0, sum_flow_c=0;
 		int count_flow_a = 0, count_flow_b = 0, count_flow_c = 0;
 		float distance;
 		for( int i=0; i < features_found.size(); i++ ){
 			cout<<"Error is "<<feature_errors[i]<<endl;

 			cout<<"Got it"<<endl;
 			Point p0( ceil( corners_prev[i].x ), ceil( corners_prev[i].y ) );
 			Point p1( ceil( corners[i].x ), ceil( corners[i].y ) );
 			line( copy, p0, p1, CV_RGB(255,255,255), 2 );

 			// accumulate flow in left third region
 			if(corners[i].x < a_right){
 				x_coords_a.push_back(corners[i].x);
 				flow_mag_a = euc_dist(corners_prev[i].x,corners_prev[i].y,corners[i].x,corners[i].y);
 				sum_flow_a+=flow_mag_a;
 				count_flow_a++;
 			}
 			else if(corners[i].x > a_right && corners[i].x < b_right){//accumulate in center third region
 				x_coords_b.push_back(corners[i].x);
 				flow_mag_b = euc_dist(corners_prev[i].x,corners_prev[i].y,corners[i].x,corners[i].y);
 				sum_flow_b+=flow_mag_b;
 				count_flow_b++;
 			}
 			else if(corners[i].x > b_right && corners[i].x < c_right){//accumulate in right third region
 				x_coords_c.push_back(corners[i].x);
 				flow_mag_c = euc_dist(corners_prev[i].x,corners_prev[i].y,corners[i].x,corners[i].y);
 				sum_flow_c+=flow_mag_c;
 				count_flow_c++;
 			}
 		}

 		// take average flow magnitude
 		avg_flow_a += sum_flow_a / count_flow_a;
 		avg_flow_b += sum_flow_b / count_flow_b;
 		avg_flow_c += sum_flow_c / count_flow_c;


 		Point p00( width/3,  height/2);
 		Point p10( width/3,  height/2 + avg_flow_a*2);
 		line( copy, p00, p10, CV_RGB(255,255,255), 2 );
 		Point p01( width/2,  height/2);
 		Point p11( width/2,  height/2 + avg_flow_b*2);
 		line( copy, p01, p11, CV_RGB(255,255,255), 2 );
 		Point p02( (width/3)*2,  height/2);
 		Point p12( (width/3)*2,  height/2 +avg_flow_c*2);
 		line( copy, p02, p12, CV_RGB(255,255,255), 2 );
 	}


	 image_prev = src_gray;//cv_ptr->image;
	 corners_prev.clear();
	 for(int i=0; i<corners.size() ; i++){
	 	corners_prev.push_back(corners[i]);
	 }

	 firsttime = true;


	 //deliberation for motion
	 // TODO: check converstion (!)
	 //ControlCommand c;
	 //double sensGaz, sensYaw, sensRP;
	 //sensGaz = sensYaw = sensRP = 1;

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
	 float motionAmount = 0.85;
	 int numFrames=5;
	 string msg;
	 geometry_msgs::Twist cmdT;

	 if(!sentCommand && loopCount%numFrames==0){
	 	 cmdT.angular.z = 0;//-cmd.yaw;
	 	cmdT.linear.z = 0;//cmd.gaz;
	 	cmdT.linear.x=0;
	 	cmdT.linear.y=0;

		 if(avg_flow_a > 1.2*avg_flow_c && avg_flow_a > 1.2*avg_flow_b){ //obstacle on left
		 	cmdT.angular.z = +motionAmount;
		 	msg = "turn right";
		 	sentCommand = true;
		 	ROS_ERROR(">> RIGHT");
		 }
		 else if(avg_flow_a*1.2 < avg_flow_c && avg_flow_c > 1.2*avg_flow_b){ //obstacle on right
		 	cmdT.angular.z = -motionAmount;
		 	msg = "turn left";
		 	sentCommand = true;
		 	ROS_ERROR(">> LEFT");
		 }
		 else if(avg_flow_b < 20){
		 	cmdT.linear.x=motionAmount;
		 	sentCommand = true;
		 	ROS_ERROR(">> FWD");
		 }
		 else if(avg_flow_b > 20){
		 	cmdT.linear.x=-motionAmount;
		 	if(avg_flow_a > avg_flow_c)
		 		cmdT.angular.z = +4*motionAmount;
		 	else
		 		cmdT.angular.z = -4*motionAmount;

		 	
		 	sentCommand = true;
		 	ROS_ERROR(">> BACK");
		 }
		 avg_flow_c=0;
		 avg_flow_a=0;
		 avg_flow_b=0;
		}
		else{// if(loopCount%numFrames==0){
			sentCommand=false;
			//loopCount=-1;
			//return;
		}
	 // if(avg_flow_b < 2)// no obstacle in front
	 // {
	 // 	cmdT.linear.x = -0.5;
	 // 	msg = "go fwd";
	 // }
		if(sentCommand){
			putText(copy, msg, cvPoint(30,30), 
				FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);

		 // cmdT.angular.x = cmdT.angular.y = gui->useHovering ? 0 : 1;
			//ROS_ERROR("avoid command!");
			vel_pub.publish(cmdT);
			loopCount=-1;
		}
		else{
			putText(copy, "NONE", cvPoint(30,30), 
				FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
		}

		cv::imshow("brain view", copy);

		cv::waitKey(3);

		loopCount++;

		//ROS_ERROR("%d %d",loopCount, sentCommand);
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