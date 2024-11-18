
#ifndef MOCK_GPS_HANDLER_HPP
#define MOCK_GPS_HANDLER_HPP

#include <boost/signals2.hpp>
#include "icd.hpp"

class MockGpsHandler {
public:
    explicit MockGpsHandler() = default;
    ~MockGpsHandler() = default;

    // Define the signal type using ICD types (drone_sdk::Location, drone_sdk::SignalQuality)
    using GpsUpdateSignal = boost::signals2::signal<void(drone_sdk::Location, drone_sdk::SignalQuality)>;

    // Subscribe to GPS update signals
    boost::signals2::connection subscribe(const GpsUpdateSignal::slot_type& slot) {
        return m_gpsUpdateSignal.connect(slot);
    }

    // Emit the signal with the mocked data
    void update(drone_sdk::Location location, drone_sdk::SignalQuality signalQuality) {
        m_gpsUpdateSignal(location, signalQuality);
    }

private:
    GpsUpdateSignal m_gpsUpdateSignal;  // Signal to notify subscribers about GPS updates
};

#endif // MOCK_GPS_HANDLER_HPP