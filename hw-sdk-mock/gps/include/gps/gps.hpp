#pragma once

namespace hw_sdk_mock
{
    class Gps
    {
    public:
        struct Location
        {
            double latitude; // Latitude in degrees (-90 to 90)
            double longitude; // Longitude in degrees (-180 to 180)
            double altitude; // Altitude in meters (0 to 10000)
        };

        enum class SignalQuality
        {
            NO_SIGNAL = 0,
            POOR,
            FAIR,
            GOOD,
            EXCELLENT
        };

        Gps() = default;
        virtual ~Gps() = default;
        Gps(const Gps &) = delete;
        Gps &operator=(const Gps &) = delete;
        Gps(Gps &&) = default;
        Gps &operator=(Gps &&) = default;

        Location getLocation();
        SignalQuality getSignalQuality();
    };
} // namespace hw_sdk_mock
