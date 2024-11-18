#ifndef MOCK_HW_MONITOR_HPP
#define MOCK_HW_MONITOR_HPP

#include <queue>
#include <thread>
#include <atomic>
#include <chrono>
#include "mock_gps_handler.hpp"  // Include the mock GPS handler
#include "mock_link_handler.hpp" // Include the mock Link handler
#include "icd.hpp"

class MockHwMonitor {
public:
    // Constructor initializes the mock handlers
    MockHwMonitor()
        : m_running(false) {}

    ~MockHwMonitor() {
        stop();
    }

    // Start polling at 10 Hz for GPS and link updates from the mock queues
    void start() {
        m_running = true;
        m_pollingThread = std::thread([this]() {
            while (m_running) {
                // Simulate fetching data from the queues
                updateGpsData();
                updateLinkData();
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
    void subscribeToGpsUpdates(const MockGpsHandler::GpsUpdateSignal::slot_type& slot) {
        m_gpsHandler.subscribe(slot);
    }

    // Subscribe to Link updates
    void subscribeToLinkUpdates(const MockLinkHandler::LinkUpdateSignal::slot_type& slot) {
        m_linkHandler.subscribe(slot);
    }

    // Load a set of mock GPS data into the queue
    void loadMockGpsData(const std::queue<drone_sdk::Location>& gpsData) {
        m_mockGpsData = gpsData;
    }

    // Load a set of mock Link data into the queue
    void loadMockLinkData(const std::queue<drone_sdk::SignalQuality>& linkData) {
        m_mockLinkData = linkData;
    }

private:
    // Simulate fetching data from the GPS queue
    void updateGpsData() {
        if (!m_mockGpsData.empty()) {
            // Get the next Location and SignalQuality
            drone_sdk::Location location = m_mockGpsData.front();
            drone_sdk::SignalQuality signalQuality = m_mockSignalQuality.front();
            m_mockGpsData.pop();

            // Trigger the GPS update signal with the mocked data
            m_gpsHandler.update(location, signalQuality);
        }
    }

    // Simulate fetching data from the Link queue
    void updateLinkData() {
        if (!m_mockLinkData.empty()) {
            // Get the next SignalQuality
            drone_sdk::SignalQuality signalQuality = m_mockLinkData.front();
            m_mockLinkData.pop();

            // Trigger the Link update signal with the mocked data
            m_linkHandler.update(signalQuality);
        }
    }

    MockGpsHandler m_gpsHandler;          // Use the MockGpsHandler
    MockLinkHandler m_linkHandler;        // Mocked Link handler
    std::atomic<bool> m_running;          // Flag to control the polling thread
    std::thread m_pollingThread;          // Thread for polling

    // Queues holding mock data
    std::queue<drone_sdk::Location> m_mockGpsData;
    std::queue<drone_sdk::SignalQuality> m_mockLinkData;
    std::queue<drone_sdk::SignalQuality> m_mockSignalQuality;
};

#endif // MOCK_HW_MONITOR_HPP

