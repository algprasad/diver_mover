//
// Created by alg on 03/09/20.
//

#include <iostream>
#include "ros/ros.h"
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv-3.3.1-dev/opencv2/imgcodecs.hpp>
#include <string>
#include <sstream>
#include <tf2/LinearMath/Quaternion.h>
#include <time.h>

#include <geometry_msgs/PoseStamped.h>

geometry_msgs::PoseStamped diver_pose;
double ttime = 5;

//initial values of diver setpoint for sinusoidal wave
double x = 5;
double y = 0;
double z = -2;

void setDiverStateMsg(gazebo_msgs::ModelState& state) {
    srand ((unsigned) time(NULL));
    //pose
    double x_inc = (float) rand()/RAND_MAX;
    x_inc = x_inc/50;
    double y_inc = (float) rand()/RAND_MAX - 0.5;
    y_inc = y_inc/1000;
    double z_inc = (float) rand()/RAND_MAX - 0.5 ;
    z_inc = z_inc/1000;

    state.pose.position.x = diver_pose.pose.position.x + x_inc;
    state.pose.position.y = diver_pose.pose.position.y + y_inc;
    state.pose.position.z = diver_pose.pose.position.z + z_inc;

    //get new orientation

    double roll = 0, pitch = 0, yaw = 0;

    roll = (((float) rand()/RAND_MAX) - 0.5)/10;
    pitch = (((float) rand()/RAND_MAX) - 0.5)/10;
    yaw =  (((float) rand()/RAND_MAX) - 0.5)/10; //(rand()%2 - 1)/10;
    std::cout<<roll<<" "<<pitch<<" "<<yaw<<std::endl;


    tf2::Quaternion diver_quat;
    diver_quat.setRPY(roll, pitch, yaw);

    state.pose.orientation.x = diver_quat.x();
    state.pose.orientation.y = diver_quat.y();
    state.pose.orientation.z = diver_quat.z();
    state.pose.orientation.w = diver_quat.w();

    state.model_name = "diver";


}

void setDiverStateMsgSinusoidal(gazebo_msgs::ModelState& state){
    double time_inc = 0.005;
    x = ttime;
    z = -2 + 0.25*sin(x);
    state.pose.position.x = x;
    state.pose.position.y = y;
    state.pose.position.z = z;

    srand ((unsigned) time(NULL));



    double roll =0, pitch =0, yaw =0;
    roll = (((float) rand()/RAND_MAX) - 0.5)/10;
    pitch = (((float) rand()/RAND_MAX) - 0.5)/10;
    yaw =  (((float) rand()/RAND_MAX) - 0.5)/10; //(rand()%2 - 1)/10;
    tf2::Quaternion diver_quat;
    diver_quat.setRPY(roll, pitch, yaw);

    state.pose.orientation.x = diver_quat.x();
    state.pose.orientation.y = diver_quat.y();
    state.pose.orientation.z = diver_quat.z();
    state.pose.orientation.w = diver_quat.w();

    state.model_name = "diver";



    ttime+= time_inc;

}

void poseCallback(const gazebo_msgs::ModelStatesConstPtr& states_msg){
    diver_pose.pose.position.x = states_msg->pose[2].position.x;
    diver_pose.pose.position.y = states_msg->pose[2].position.y;
    diver_pose.pose.position.z = states_msg->pose[2].position.z;

    diver_pose.pose.orientation.x = states_msg->pose[2].orientation.x;
    diver_pose.pose.orientation.y = states_msg->pose[2].orientation.y;
    diver_pose.pose.orientation.z = states_msg->pose[2].orientation.z;
    diver_pose.pose.orientation.w = states_msg->pose[2].orientation.w;
    std::cout<<"Diver_Pose: "<<diver_pose.pose.position.x<<" "<<diver_pose.pose.position.y<<" "<<diver_pose.pose.position.z<<std::endl;

}



int main(int argc, char **argv){
    ros::init(argc, argv, "diver_mover");
    ros::NodeHandle nh;

    // Publisher and Subsriber stuff
    ros::Publisher diver_state_pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

    ros::Subscriber sub_diver_pose = nh.subscribe("/gazebo/model_states", 10, poseCallback);
    ros::Rate rate(50);


    //First send the diver to x = 5, y= 0, z = -2 to begin the detection and following. Also this would get the latest state of diver pose for the next part to do incremental updates
    int i = 0;
    gazebo_msgs::ModelState initial_diver_state_msg;
    initial_diver_state_msg.model_name = "diver";
    initial_diver_state_msg.pose.position.x = 5;
    initial_diver_state_msg.pose.position.y = 0;
    initial_diver_state_msg.pose.position.z = -2;
    initial_diver_state_msg.pose.orientation.x = 0;
    initial_diver_state_msg.pose.orientation.y = 0;
    initial_diver_state_msg.pose.orientation.z = 0;
    initial_diver_state_msg.pose.orientation.w = 1;

    while(i< 100){
        diver_state_pub.publish(initial_diver_state_msg);
        rate.sleep();
        ros::spinOnce();
        i++;
    }


    while(ros::ok()){
        gazebo_msgs::ModelState diver_state_msg;
        setDiverStateMsgSinusoidal(diver_state_msg);
        //setDiverStateMsg(diver_state_msg);
        diver_state_pub.publish(diver_state_msg);
        rate.sleep();
        ros::spinOnce();
    }

    return 0;


}
