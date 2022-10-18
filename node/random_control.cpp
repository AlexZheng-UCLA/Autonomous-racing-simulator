// Function 
// input: Odometry  
// output: drive_st_msg 
// random_control generate random control command 

#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

#define NUM 100  // steering_num & throttle_num

bool isRandomcontrol; 
int mode = 1;
bool init = false;

class Randomcontrol {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_throttle, max_steering;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;
    // For printing
    std::vector<bool> prev_mux;

    // Listen for odom messages
    ros::Subscriber odom_sub;

    // Listen for mux messages 
    ros::Subscriber mux_sub;

    // Publish control command 
    ros::Publisher control_pub;

    // previous desired steering angle
    double prev_steering=0.0;
    double prev_throttle=0.0;
    double curr_speed=0.0;

    // variables to record 
    int throttle_num = 0;
    int steering_num = 0;
    double throttle = 0;
    double steering = 0;


public:

    Randomcontrol() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic, mux_topic;
        n.getParam("rand_drive_topic", drive_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("mux_topic", mux_topic);

        // get car parameters
        n.getParam("max_throttle", max_throttle);
        n.getParam("max_steering", max_steering);

        // get size of mux
        n.getParam("mux_size", mux_size);
        
        int random_control_mux_idx;
        n.getParam("random_control_mux_idx", random_control_mux_idx);

        // initialize mux controller
        mux_controller.reserve(mux_size);
        prev_mux.reserve(mux_size);
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = false;
            prev_mux[i] = false;
        }

        // Make a publisher for control messages
        control_pub = n.advertise<std_msgs::Float32MultiArray>("/control", 10);

        // Start a subscriber to listen to mux messages
        mux_sub = n.subscribe(mux_topic, 1, &Randomcontrol::mux_callback, this);
        
        // Start a subscriber to listen to odom messages
        // odom_sub = n.subscribe(mux_topic, 1, &Randomcontrol::odom_callback, this);

    }

    void mux_callback(const std_msgs::Int32MultiArray & msg) {
        // reset mux member variable every time it's published
        for (int i = 0; i < mux_size; i++) {
            mux_controller[i] = bool(msg.data[i]);
        }
        if( mux_controller[2]){
            isRandomcontrol = true;
        }
        else{
            isRandomcontrol = false;
        }
    }

    void odom_callback(const nav_msgs::Odometry & msg) {
        ;
    }

    void mode1() {

        if (!init){
            throttle = 0;steering = 0;
            throttle_num=1;steering_num=1;
            init = true;
        }

        // only change the steering 
        else if (steering_num < NUM && throttle_num <= NUM){
            steering_num += 1;

            // get random 
            double random = ((double) rand() / RAND_MAX);
            double range = max_steering / 2.0;
            double rand_ang = random * range - range / 2.0;  // Maximum 1/4

            // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
            random = ((double) rand() / RAND_MAX); 

            // if ((random > 0.8) && (prev_steering != 0)) {
            // double sign_rand = rand_ang / std::abs(rand_ang);
            // double sign_prev = prev_steering / std::abs(prev_angle);
            // rand_ang *= sign_rand * sign_prev;
            // }   

            // set steering (add random change to previous angle)
            steering = std::min(std::max(prev_steering + rand_ang, -max_steering), max_steering);

             // reset previous desired angle
            prev_steering = steering;
        }
        
        else if (throttle_num <= NUM){
            steering_num = 1;
            throttle_num += 1; 

            double random = ((double) rand() / RAND_MAX);
            double range = max_throttle / 2.0; 
            double rand_throttle  = random * range - range / 2.0;  // Maximum 1/4

            // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
            random = ((double) rand() / RAND_MAX); 

            // if ((random > 0.9) && (prev_throttle != 0)) {
            // double sign_rand = rand_throttle / std::abs(rand_throttle);
            // double sign_prev = prev_throttle / std::abs(prev_throttle);
            // rand_throttle *= sign_rand * sign_prev;
            // }

            // set steering (add random change to previous angle)
            throttle = std::min(std::max(prev_throttle + rand_throttle, -max_throttle), max_throttle);
            
            // reset prev_throttle
            prev_throttle = throttle;
        }
        else{
            init = false;
            ROS_INFO("Mode1 Over !!!!!!!!!!!!!!!!!!!!!!\n");
            mode += 1;
        }


        // publish control command
        publish_control();

    }

    void mode2() {

        if (!init){
            throttle = 0;steering = 0;
            throttle_num=1;steering_num=1;
            init = true;
        }

        // only change the throttle
        else if (throttle_num < NUM && steering_num <= NUM){
            throttle_num += 1;

            double random = ((double) rand() / RAND_MAX);
            double range = max_throttle / 4.0; 
            double rand_throttle  = random * range - range / 2.0;  // Maximum 1/4

            // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
            // random = ((double) rand() / RAND_MAX); 
            // if ((random > 0.8) && (prev_throttle != 0)) {
            // double sign_rand = rand_throttle / std::abs(rand_throttle);
            // double sign_prev = prev_throttle / std::abs(prev_throttle);
            // rand_throttle *= sign_rand * sign_prev;
            // }

            // set steering (add random change to previous angle)
            throttle = std::min(std::max(prev_throttle + rand_throttle, -max_throttle), max_throttle);
            
            // reset prev_throttle
            prev_throttle = throttle;

        
        }
        else if (steering_num <= NUM){
            throttle_num = 1;
            steering_num += 1; 

            // get random 
            double random = ((double) rand() / RAND_MAX);
            double range = max_steering / 1.5;
            double rand_ang = random * range - range / 2.0;  // Maximum 1/3

            // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
            // random = ((double) rand() / RAND_MAX); 
            // if ((random > 0.8) && (prev_steering != 0)) {
            // double sign_rand = rand_ang / std::abs(rand_ang);
            // double sign_prev = prev_steering / std::abs(prev_angle);
            // rand_ang *= sign_rand * sign_prev;
            // }   

            // set steering (add random change to previous angle)
            steering = std::min(std::max(prev_steering + rand_ang, -max_steering), max_steering);

            // reset previous desired angle
            prev_steering = steering;
        }
        else{

            init = false;
            ROS_INFO("Mode2 Over !!!!!!!!!!!!!!!!!!!!!!\n");
            mode += 1;
        }

        publish_control();
    }

    void mode3() {

        if (!init){
            throttle = 0;steering = 0;
            throttle_num=1;steering_num=1;
            init = true;
        }

        else if( throttle_num < NUM * NUM / 10){

            for(int i=0; i<5; i++){

                throttle_num += 1;
                double random = ((double) rand() / RAND_MAX);
                double range = max_throttle / 4.0; 
                double rand_throttle  = random * range - range / 2.0;  // Maximum 1/4

                // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
                // random = ((double) rand() / RAND_MAX); 
                // if ((random > 0.8) && (prev_throttle != 0)) {
                // double sign_rand = rand_throttle / std::abs(rand_throttle);
                // double sign_prev = prev_throttle / std::abs(prev_throttle);
                // rand_throttle *= sign_rand * sign_prev;
                // }

                // set steering (add random change to previous angle)
                throttle = std::min(std::max(prev_throttle + rand_throttle, -max_throttle), max_throttle);
                
                // reset prev_throttle
                prev_throttle = throttle;
            }
            
            for(int i=0; i<5; i++){

                steering_num += 1; 

                // get random 
                double random = ((double) rand() / RAND_MAX);
                double range = max_steering / 1.5;
                double rand_ang = random * range - range / 2.0;  // Maximum 1/3

                // sometimes change sign so it turns more (basically add bias to continue turning in current direction)
                // random = ((double) rand() / RAND_MAX); 
                // if ((random > 0.8) && (prev_steering != 0)) {
                // double sign_rand = rand_ang / std::abs(rand_ang);
                // double sign_prev = prev_steering / std::abs(prev_angle);
                // rand_ang *= sign_rand * sign_prev;
                // }   

                // set steering (add random change to previous angle)
                steering = std::min(std::max(prev_steering + rand_ang, -max_steering), max_steering);

                // reset previous desired angle
                prev_steering = steering;
            }
            
        }
        else{
            isRandomcontrol = false;
            steering_num = -1; throttle_num = -1;
            ROS_INFO("Mode3 Over !!!!!!!!!!!!!!!!!!!!!!\n");
            mode += 1;
        }

        publish_control();
    }

    void publish_control(){
        
        std_msgs::Float32MultiArray control_msg;
        control_msg.data.clear();
        control_msg.data.push_back(throttle);
        control_msg.data.push_back(steering);
        control_msg.data.push_back(throttle_num);
        control_msg.data.push_back(steering_num);
        control_pub.publish(control_msg);
    }

}; // end of class definition


int main(int argc, char ** argv) {
    ros::init(argc, argv, "random_control");
    Randomcontrol rw;

    ros::Rate loop_rate(50); 

    while(ros::ok())
    {   
        ros::spinOnce();
        if(isRandomcontrol){
            
            if(mode == 1) 
                rw.mode1();
            else if(mode == 2) 
                rw.mode2();
            else if(mode == 3)
                rw.mode3();
            else 
                break;
        }
    
        loop_rate.sleep();
    }
    
    return 0;
}