#ifndef MOCK_GPS_HPP
#define MOCK_GPS_HPP

#include "icd.hpp"
#include <cstdint>

namespace hw_sdk_mock
{

    class MockGps
    {
    public:
        // Constructor
        MockGps() : m_location({0.0, 0.0, 0.0}), m_signalQuality(drone_sdk::SignalQuality::NO_SIGNAL) {}

        // Destructor
        ~MockGps() = default;

        // Mock method to set the GPS location using the Location struct from ICD
        void setLocation(double latitude, double longitude, double altitude)
        {
            m_location = {latitude, longitude, altitude};
        }

        // Mock method to get the GPS location (returns ICD Location struct)
        drone_sdk::Location getLocation() const
        {
            return m_location;
        }

        // Mock method to set the GPS signal quality (uses ICD SignalQuality enum)
        void setSignalQuality(drone_sdk::SignalQuality quality)
        {
            m_signalQuality = quality;
        }

        // Mock method to get the GPS signal quality (returns ICD SignalQuality enum)
        drone_sdk::SignalQuality getSignalQuality() const
        {
            return m_signalQuality;
        }

        // Method to simulate the callback, which might be used in a subscriber
        void simulateGpsUpdate()
        {
            // Example callback invocation (you can pass a callback function from the subscriber)
            if (m_locationCallback) {
                m_locationCallback(m_location, m_signalQuality);
            }
        }

        // Set callback for GPS location updates
        void setLocationCallback(drone_sdk::GpsCallback::Type callback)
        {
            m_locationCallback = callback;
        }

    private:
        drone_sdk::Location m_location;           // Current GPS location (ICD struct)
        drone_sdk::SignalQuality m_signalQuality; // Current GPS signal quality (ICD enum)
        drone_sdk::GpsCallback::Type m_locationCallback; // Callback function for location updates
    };

} // namespace hw_sdk_mock

#endif // MOCK_GPS_HPP
