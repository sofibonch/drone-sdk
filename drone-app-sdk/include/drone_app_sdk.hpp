#ifndef DRONE_APP_SDK_HPP
#define DRONE_APP_SDK_HPP

#include <iostream>

namespace drone {

class DroneAppSdk {
public:
    DroneAppSdk();
    ~DroneAppSdk();

    void startMission();
    void abortMission();
    void getStatus();

private:
    void printStatus(const std::string& status);
};

} // namespace drone

#endif // DRONE_APP_SDK_HPP
