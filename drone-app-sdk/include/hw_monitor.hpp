#ifndef HW_MONITOR_HPP
#define HW_MONITOR_HPP
#include <thread>
#include <chrono>
#include <atomic>
#include "gps_handler.hpp"
#include "link_handler.hpp" 
#include "icd.hpp"

class HardwareMonitor {
public:
    // Constructor now initializes both GpsHandler and LinkHandler
    HardwareMonitor()
        : m_gpsHandler(), m_linkHandler(), m_running(false) {}

    ~HardwareMonitor() {
        stop();
    }

    // Start polling at 10 Hz for GPS and link updates
    void start() {
        m_running = true;
        m_pollingThread = std::thread([this]() {
            while (m_running) {
                m_gpsHandler.update();  // Update GPS handler
                m_linkHandler.update();  // Update Link handler
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

    // Stop polling
    void stop() {
        m_running = false;
        if (m_pollingThread.joinable()) {
            m_pollingThread.join();
        }
    }

    // Subscribe to GPS updates
    void subscribeToGpsUpdates(const GpsHandler::GpsUpdateSignal::slot_type& slot) {
        m_gpsHandler.subscribe(slot);
    }

    // Subscribe to Link updates
    void subscribeToLinkUpdates(const LinkHandler::LinkUpdateSignal::slot_type& slot) {
        m_linkHandler.subscribe(slot);
    }

private:
    GpsHandler m_gpsHandler;  // HardwareMonitor owns its own GpsHandler
    LinkHandler m_linkHandler;  // HardwareMonitor owns its own LinkHandler
    std::atomic<bool> m_running;  // Flag to control the polling thread
    std::thread m_pollingThread;  // Thread for polling
};

#endif // HW_MONITOR_HPP
