#ifndef SAFETY_STATE_MACHINE_HPP
#define SAFETY_STATE_MACHINE_HPP

#include <boost/sml.hpp>
#include <iostream>
#include "gps/gps.hpp"
#include "link/link.hpp"
#include "icd.hpp"

namespace sml = boost::sml;

struct GpsSignal {
    drone_sdk::SignalQuality quality;  // Updated to use drone_sdk::SignalQuality
};

struct LinkSignal {
    drone_sdk::SignalQuality quality;  // Updated to use drone_sdk::SignalQuality
};

class SafetyStateMachine {
public:
    auto operator()() const {
        using namespace sml;

        // Define states
        const auto gpsHealthy = state<class GpsHealthy>;
        const auto gpsNotHealthy = state<class GpsNotHealthy>;
        const auto connectionConnected = state<class Connected>;
        const auto connectionDisconnected = state<class Disconnected>;

        // Define state machine transitions and actions
        return make_transition_table(
            // GPS signal quality transitions
            *gpsHealthy + event<GpsSignal> [([](const GpsSignal &gs) { return gs.quality == drone_sdk::SignalQuality::NO_SIGNAL; })] / [] {
                std::cout << "Transitioned to GPS Not Healthy" << std::endl;
            } = gpsNotHealthy,

            gpsNotHealthy + event<GpsSignal> [([](const GpsSignal &gs) { return gs.quality != drone_sdk::SignalQuality::NO_SIGNAL; })] / [] {
                std::cout << "Transitioned to GPS Healthy" << std::endl;
            } = gpsHealthy,

            // Connection status transitions
            *connectionConnected + event<LinkSignal> [([](const LinkSignal &ls) { return ls.quality == drone_sdk::SignalQuality::NO_SIGNAL; })] / [] {
                std::cout << "Transitioned to Connection Disconnected" << std::endl;
            } = connectionDisconnected,

            connectionDisconnected + event<LinkSignal> [([](const LinkSignal &ls) { return ls.quality != drone_sdk::SignalQuality::NO_SIGNAL; })] / [] {
                std::cout << "Transitioned to Connection Connected" << std::endl;
            } = connectionConnected,

            // Emergency actions
            gpsNotHealthy + event<GpsSignal> / [] {
                std::cout << "Emergency: Triggering Emergency Landing due to GPS failure" << std::endl;
            },

            connectionDisconnected + event<LinkSignal> / [] {
                std::cout << "Emergency: Triggering Return Home due to disconnection" << std::endl;
            }
        );
    }
};

#endif // SAFETY_STATE_MACHINE_HPP
