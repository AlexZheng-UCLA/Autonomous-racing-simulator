#pragma once

namespace racecar_simulator {

struct CarState {
    double x;       // x position
    double y;       // y position
    double phi;   // oritention- yaw
    double vx;      // longitudinal speed
    double vy;      // lateral speed
    double r;       // yaw rate
    double s;       // path length 
    double vs;      // set speed

    // double slip_angle;
    // double throttle;           
    // double steer_angle; 
};

}
