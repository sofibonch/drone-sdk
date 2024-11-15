#ifndef MOCK_GPS_HPP
#define MOCK_GPS_HPP

#include "gps_handler.hpp"

#include <cstdint>

namespace hw_sdk_mock
{

    class MockGps
    {
    public:
        // Struct representing a location with latitude, longitude, and altitude
        struct Location
        {
            float latitude;
            float longitude;
            float altitude;
        };

        // Enum representing signal quality levels
        enum class SignalQuality
        {
            NO_SIGNAL = 0,
            POOR = 1,
            FAIR = 2,
            GOOD = 3,
            EXCELLENT = 4
        };

        // Constructor
        MockGps() : m_location({0.0f, 0.0f, 0.0f}), m_signalQuality(SignalQuality::NO_SIGNAL) {}

        // Destructor
        ~MockGps() = default;

        // Mock method to set the GPS location
        void setLocation(float latitude, float longitude, float altitude)
        {
            m_location = {latitude, longitude, altitude};
        }

        // Mock method to get the GPS location
        Location getLocation() const
        {
            return m_location;
        }

        // Mock method to set the GPS signal quality
        void setSignalQuality(SignalQuality quality)
        {
            m_signalQuality = quality;
        }

        // Mock method to get the GPS signal quality
        SignalQuality getSignalQuality() const
        {
            return m_signalQuality;
        }

    private:
        Location m_location;           // Current GPS location
        SignalQuality m_signalQuality; // Current GPS signal quality
    };

} // namespace hw_sdk_mock

#endif // MOCK_GPS_HPP
