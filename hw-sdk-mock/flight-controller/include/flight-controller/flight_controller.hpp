#pragma once

namespace hw_sdk_mock
{
    class FlightController
    {
    public:
        enum class ResponseCode
        {
            SUCCESS = 0,
            CONNECTION_ERROR,
            HARDWARE_ERROR,
            INVALID_COMMAND
        };

        FlightController() = default;
        virtual ~FlightController() = default;
        FlightController(const FlightController &) = delete;
        FlightController &operator=(const FlightController &) = delete;
        FlightController(FlightController &&) = default;
        FlightController &operator=(FlightController &&) = default;

        ResponseCode arm();
        ResponseCode disarm();
        ResponseCode takeOff();
        ResponseCode land();
        ResponseCode goHome();

        /*
        * @brief Go to a specified location
        * @param latitude Latitude in degrees (-90 to 90)
        * @param longitude Longitude in degrees (-180 to 180)
        * @param altitude Altitude in meters (0 to 10000)
        */
        ResponseCode goTo(double latitude, double longitude, double altitude);
    };
} // namespace hw_sdk_mock
