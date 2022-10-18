#pragma once

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"

namespace racecar_simulator {

class STKinemics {

public:

    static CarState update(
            const CarState start,
            double throttle,
            double steering,
            CarParams p,
            double dt);

    static CarState update_T(
            const CarState start,
            double throttle,
            double steering,
            CarParams p,
            double dt);
};

}
