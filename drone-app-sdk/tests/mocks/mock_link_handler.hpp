
#ifndef MOCK_LINK_HANDLER_HPP
#define MOCK_LINK_HANDLER_HPP

#include <boost/signals2.hpp>
#include "icd.hpp"

class MockLinkHandler {
public:
    explicit MockLinkHandler() = default;
    ~MockLinkHandler() = default;

    // Define the signal type using ICD types (drone_sdk::SignalQuality)
    using LinkUpdateSignal = boost::signals2::signal<void(drone_sdk::SignalQuality)>;

    // Subscribe to Link update signals
    boost::signals2::connection subscribe(const LinkUpdateSignal::slot_type& slot) {
        return m_linkUpdateSignal.connect(slot);
    }

    // Emit the signal with the mocked data
    void update(drone_sdk::SignalQuality signalQuality) {
        m_linkUpdateSignal(signalQuality);
    }

private:
    LinkUpdateSignal m_linkUpdateSignal;   // Signal to notify subscribers about Link updates
};

#endif // MOCK_LINK_HANDLER_HPP
