#ifndef LINK_HANDLER_HPP
#define LINK_HANDLER_HPP

#include <boost/signals2.hpp>
#include <thread>
#include <chrono>
#include "link/link.hpp" 

class LinkHandler {
public:
    explicit LinkHandler() = default;
    ~LinkHandler() = default;

    using LinkUpdateSignal = boost::signals2::signal<void(hw_sdk_mock::Link::SignalQuality)>;

    boost::signals2::connection subscribe(const LinkUpdateSignal::slot_type& slot) {
        return m_linkUpdateSignal.connect(slot);
    }

    void update() {
        auto signalQuality = m_linkDevice.getSignalQuality();
        m_linkUpdateSignal(signalQuality);
    }

private:
    hw_sdk_mock::Link m_linkDevice;
    LinkUpdateSignal m_linkUpdateSignal;
};

#endif // LINK_HANDLER_HPP
