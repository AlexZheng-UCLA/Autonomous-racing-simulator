#include <ros/ros.h> 
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_recorder");
    ros::NodeHandle n;
    // ros::Subscriber pose_mc = n.subscribe("/vicon/zonda/zonda", 1, mcCallback, ros::TransportHints().tcpNoDelay());    
    // ros::Subscriber pose_ekf = n.subscribe("/ekf_odom", 1, ekfCallback, ros::TransportHints().tcpNoDelay());  
    
    std::ofstream file("test.csv");

    // Time ******
    ros::Rate loop_rate(50); 

    while(ros::ok())
    {   
        // Position Command ******
        ros:: spinOnce();
        std::stringstream Px, Py, Psi;
        Px << (float)1.0;
        Py << (float)1.0;
        Psi << (float)1.0;

        file << Px.str() << "," << Py.str() << "," << Psi.str() << "\n";
        loop_rate.sleep();
    }
    
    //关闭串口
    file.close();
 
    return 0;
}