#include <boost/signals2.hpp>
#include <thread>
#include <chrono>
#include "gps/gps.hpp"

class GpsHandler {
public:
    // Default constructor and destructor
    explicit GpsHandler() = default;
    ~GpsHandler() = default;
    
    using GpsUpdateSignal = boost::signals2::signal<void(hw_sdk_mock::Gps::Location, hw_sdk_mock::Gps::SignalQuality)>;

    // Registers a callback for GPS updates
    boost::signals2::connection subscribe(const GpsUpdateSignal::slot_type& slot) {
        return m_gpsUpdateSignal.connect(slot);
    }

    void update() {
        auto location = m_gpsDevice.getLocation();
        auto signalQuality = m_gpsDevice.getSignalQuality();
        m_gpsUpdateSignal(location, signalQuality); 
    }

private:
    hw_sdk_mock::Gps m_gpsDevice;
    GpsUpdateSignal m_gpsUpdateSignal;
};
