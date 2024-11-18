#ifndef DRONE_APP_SDK_HPP
#define DRONE_APP_SDK_HPP

#include <iostream>
#include "icd.hpp"

namespace drone
{

    class DroneAppSdk
    {
    public:
        DroneAppSdk();
        ~DroneAppSdk();

        drone_sdk::FlightControllerStatus goTo(const drone_sdk::Location &location);
        // drone_sdk::FlightControllerStatus followPath(const std::/queue<drone_sdk::Location> &path);
        drone_sdk::FlightControllerStatus abortMission();
        drone_sdk::FlightControllerStatus hover();

    private:
        void printStatus(const std::string &status);
    };

} // namespace drone

#endif // DRONE_APP_SDK_HPP
