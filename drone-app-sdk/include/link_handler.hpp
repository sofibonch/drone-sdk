#ifndef LINK_HANDLER_HPP
#define LINK_HANDLER_HPP

#include <boost/signals2.hpp>
#include <thread>
#include <chrono>
#include "link/link.hpp"
#include "icd.hpp"  // Include the ICD header for SignalQuality

class LinkHandler {
public:
    explicit LinkHandler() = default;
    ~LinkHandler() = default;

    // Define the signal type using ICD types (drone_sdk::SignalQuality)
    using LinkUpdateSignal = boost::signals2::signal<void(drone_sdk::SignalQuality)>;

    // Subscribe to Link update signals
    boost::signals2::connection subscribe(const LinkUpdateSignal::slot_type& slot) {
        return m_linkUpdateSignal.connect(slot);
    }

    // Update Link signal quality
    void update() {
        auto signalQuality = m_linkDevice.getSignalQuality(); // Get signal quality from hw_sdk_mock::Link

        // Convert hw_sdk_mock::Link::SignalQuality to drone_sdk::SignalQuality
        drone_sdk::SignalQuality icdSignalQuality = static_cast<drone_sdk::SignalQuality>(signalQuality);

        // Emit the signal with the converted type
        m_linkUpdateSignal(icdSignalQuality);
    }

private:
    hw_sdk_mock::Link m_linkDevice;        // Original Link device handler
    LinkUpdateSignal m_linkUpdateSignal;   // Signal to notify subscribers about Link updates
};

#endif // LINK_HANDLER_HPP
