#ifndef GPS_HANDLER_HPP
#define GPS_HANDLER_HPP

#include <boost/signals2.hpp>
#include <thread>
#include <chrono>
#include "gps/gps.hpp"
#include "icd.hpp"  // Include the ICD header for Location and SignalQuality

class GpsHandler {
public:
    explicit GpsHandler() = default;
    ~GpsHandler() = default;

    // Define the signal type using ICD types (drone_sdk::Location, drone_sdk::SignalQuality)
    using GpsUpdateSignal = boost::signals2::signal<void(drone_sdk::Location, drone_sdk::SignalQuality)>;

    // Subscribe to GPS update signals
    boost::signals2::connection subscribe(const GpsUpdateSignal::slot_type& slot) {
        return m_gpsUpdateSignal.connect(slot);
    }

    // Update GPS location and signal quality
    void update() {
        auto location = m_gpsDevice.getLocation();         // Get location from hw_sdk_mock::Gps
        auto signalQuality = m_gpsDevice.getSignalQuality(); // Get signal quality from hw_sdk_mock::Gps

        // Convert hw_sdk_mock::Gps::Location to drone_sdk::Location
        drone_sdk::Location icdLocation = {location.latitude, location.longitude, location.altitude};

        // Convert hw_sdk_mock::Gps::SignalQuality to drone_sdk::SignalQuality
        drone_sdk::SignalQuality icdSignalQuality = static_cast<drone_sdk::SignalQuality>(signalQuality);

        // Emit the signal with converted types
        m_gpsUpdateSignal(icdLocation, icdSignalQuality);
    }

private:
    hw_sdk_mock::Gps m_gpsDevice;          // Original GPS device handler
    GpsUpdateSignal m_gpsUpdateSignal;     // Signal to notify subscribers about GPS updates
};

#endif // GPS_HANDLER_HPP
