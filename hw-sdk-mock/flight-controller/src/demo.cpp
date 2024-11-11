#include "flight-controller/flight_controller.hpp"


int main()
{
    hw_sdk_mock::FlightController fc;
    fc.arm();
    fc.disarm();
    fc.takeOff();
    return 0;
}
