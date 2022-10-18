#include <cmath>
#include "stdlib.h"

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/dynamics.hpp"
#include <iostream>

using namespace racecar_simulator;

CarState Dynamics::update(
        const CarState start,
        double throttle,
        double steering,
        CarParams p,
        double dt) {
    
    const double phi = start.phi;
    const double vx = start.vx;
    const double vy = start.vy;
    const double r  = start.r;
    const double s = start.s;
    const double vs = start.vs;

    const TireForces tire_forces_front = getForceFront(start);
    const TireForces tire_forces_rear  = getForceRear(start);
    const double friction_force = getForceFriction(start);
    
    CarState delta;
    delta.x = vx * std::cos(phi) - vy * std::sin(phi);
    delta.y = vy * std::cos(phi) + vx * std::sin(phi);
    delta.phi = r;
    delta.vx = 1.0/ p.m *(tire_forces_rear.F_x + friction_force - tire_forces_front.F_y*std::sin(delta) + param_.m*vy*r);
    delta.vy = 1.0/param_.m*(tire_forces_rear.F_y + tire_forces_front.F_y*std::cos(delta) - param_.m*vx*r);
    delta.r = 1.0/param_.Iz*(tire_forces_front.F_y*param_.lf* std::cos(delta) - tire_forces_rear.F_y*param_.lr);
    delta.s = vs;
    



}

CarState Dynamics:: 