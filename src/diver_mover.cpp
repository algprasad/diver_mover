#include <iostream>
#include "ros/ros.h"
#include <gazebo_msgs/ModelState.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv-3.3.1-dev/opencv2/imgcodecs.hpp>
#include <string>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <time.h>

void setDiverStateMsg(gazebo_msgs::ModelState& state) {
    srand (time(NULL));
    //pose
    double x_inc = rand()%3;
    state.pose.position.x = 2 + x_inc;
    state.pose.position.y = 0;
    state.pose.position.z = -2;

    //get new orientation

    double roll = 0, pitch = 0, yaw = 0;

    roll = rand()%2 - 1;
    pitch = rand()%2 - 1;
    yaw = rand()%2 - 1;
    std::cout<<roll<<" "<<pitch<<" "<<yaw<<std::endl;


    tf2::Quaternion diver_quat;
    diver_quat.setRPY(roll, pitch, yaw);

    state.pose.orientation.x = diver_quat.x();
    state.pose.orientation.y = diver_quat.y();
    state.pose.orientation.z = diver_quat.z();
    state.pose.orientation.w = diver_quat.w();

    state.model_name = "diver";


}

//save the images to a folder
void imageCallback(sensor_msgs::ImagePtr image){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    static long image_index = 1;
    std::ostringstream ss;
    ss<<image_index;
    std::string image_name = "diver_images/diver_straight/straight_straight/ss"  + ss.str() + ".jpg";
    //cv::imwrite(image_name, cv_ptr->image );
    image_index++;


}

int main(int argc, char **argv){
	ros::init(argc, argv, "diver_mover");
	ros::NodeHandle nh;

	// Publisher and Subsriber stuff
	 ros::Publisher diver_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
	
	 ros::Subscriber sub_image = nh.subscribe("/stereo/left/image_raw", 10, imageCallback);
    ros::Rate rate(50);
	while(ros::ok()){
	    gazebo_msgs::ModelState diver_state_msg;
	    setDiverStateMsg(diver_state_msg);
	    diver_state_pub.publish(diver_state_msg);
	    rate.sleep();
	    ros::spinOnce();
	}
	
	return 0;


}
