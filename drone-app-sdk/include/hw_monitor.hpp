// hw_monitor.hpp
#include <thread>
#include <chrono>
#include <atomic>
#include "gps_handler.hpp"

class HardwareMonitor {
public:
    HardwareMonitor(GpsHandler& gpsHandler)
        : m_gpsHandler(gpsHandler), m_running(false) {}

    ~HardwareMonitor() {
        stop();
    }

    // Start polling at 10 Hz
    void start() {
        m_running = true;
        m_pollingThread = std::thread([this]() {
            while (m_running) {
                m_gpsHandler.update();
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

private:
    GpsHandler& m_gpsHandler;
    std::atomic<bool> m_running;      // Flag to control the polling thread
    std::thread m_pollingThread;      // Thread for polling
};
