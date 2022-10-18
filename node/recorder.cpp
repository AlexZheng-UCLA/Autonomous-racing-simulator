// Function 
// input: /joy /odom /gt_pose
// output: 

#include <ros/ros.h>

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/ackermann_kinematics.hpp"
#include "f1tenth_simulator/scan_simulator_2d.hpp"

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"
#include "f1tenth_simulator/ks_kinematics.hpp"
#include "f1tenth_simulator/st_kinematics.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <iostream>
#include <math.h>
#include <fstream>
#include <string>
#include <sstream>

using namespace racecar_simulator;

// Data to record *******************************
double x=0.0, y=0.0, theta=0.0;
double gt_x=0.0, gt_y=0.0, gt_theta=0.0;
double throttle=0.0, steering=0.0;
double throttle_num=0, steering_num=0;

void pose_callback(const geometry_msgs::PoseStamped & msg) {
    // x = msg.pose.position.x;
    // y = msg.pose.position.y;
    // geometry_msgs::Quaternion q = msg.pose.orientation;
    // tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    // theta = tf2::impl::getYaw(quat);
    ;
}

void gt_pose_callback(const geometry_msgs::PoseStamped & msg) {
    gt_x = msg.pose.position.x;
    gt_y = msg.pose.position.y;
    geometry_msgs::Quaternion q = msg.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    gt_theta = tf2::impl::getYaw(quat);
}

void odom_callback(const nav_msgs::Odometry & msg) {

    x = msg.pose.pose.position.x;
    y = msg.pose.pose.position.y;
    geometry_msgs::Quaternion q = msg.pose.pose.orientation;
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    theta = tf2::impl::getYaw(quat);
    ;
}

void joy_callback(const sensor_msgs::Joy & msg) {

    throttle = msg.axes[1];
    steering = msg.axes[2];

}

void key_callback(const std_msgs::String & msg) {
    ;
}
    
void drive_callback(const ackermann_msgs::AckermannDriveStamped & msg) {
    // desired_speed = msg.drive.speed;
    // desired_steer_ang = msg.drive.steering_angle;
    ;
}

void ctrl_callback(const std_msgs::Float32MultiArray &msg){
    throttle = msg.data[0];
    steering = msg.data[1];
    throttle_num = msg.data[2];
    steering_num = msg.data[3];

}



int main(int argc, char ** argv) {

    ros::init(argc, argv, "recorder");
    ros::NodeHandle n;

    ros::Subscriber drive_sub = n.subscribe("/drive", 1, drive_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber joy_sub = n.subscribe("/joy", 1, joy_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber key_sub = n.subscribe(key_topic, 1, key_callback);
    // ros::Subscriber pose_sub = n.subscribe(pose_topic, 1, pose_callback);
    ros::Subscriber gt_pose_sub = n.subscribe("/gt_pose", 1, gt_pose_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber ctrl_sub = n.subscribe("/ctrl", 1, ctrl_callback, ros::TransportHints().tcpNoDelay());
        
    std::ofstream file("raw.csv");

    ros::Rate loop_rate(50); 
    
    while(ros::ok())
    {
        ros:: spinOnce();
        
        file << x << "," << y << "," << theta << "," 
        << throttle << "," << steering << "," 
        << throttle_num << "," << steering_num << std::endl ;

        loop_rate.sleep();
    }
    

    file.close();

    return 0;
}


