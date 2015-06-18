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


 using namespace std;

 class BrainNode
 {	

 	void vidCb(const sensor_msgs::ImageConstPtr img);
 	
 	ros::Subscriber vid_sub;
 	std::string video_channel;
 	ros::NodeHandle nh_;

 public:
 	BrainNode();


 };

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

 	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
 		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

 	cv::imshow("brain view", cv_ptr->image);

 	cv::waitKey(3);
 	//cout<<"received!!!"<<endl;


 }


 BrainNode::BrainNode(){
 	video_channel = nh_.resolveName("ardrone/image_raw");
 	vid_sub       = nh_.subscribe(video_channel,10, &BrainNode::vidCb, this);

 }

 int main(int argc, char **argv)
 {
 	ros::init(argc, argv, "drone_brain");

 	ROS_INFO("Started RU ArDrone Brain Node.");

 	BrainNode brainNode;

 	ros::spin();

 	return 0;
 }