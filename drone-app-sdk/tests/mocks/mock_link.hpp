#ifndef MOCK_LINK_HPP
#define MOCK_LINK_HPP

#include <cstdint>

namespace hw_sdk_mock {

class MockLink {
public:
    // Enum representing signal quality levels
    enum class SignalQuality {
        NO_SIGNAL = 0,
        POOR = 1,
        FAIR = 2,
        GOOD = 3,
        EXCELLENT = 4
    };

    // Constructor
    MockLink() : m_signalQuality(SignalQuality::NO_SIGNAL) {}

    // Destructor
    ~MockLink() = default;

    // Mock method to set the Link signal quality
    void setSignalQuality(SignalQuality quality) {
        m_signalQuality = quality;
    }

    // Mock method to get the Link signal quality
    SignalQuality getSignalQuality() const {
        return m_signalQuality;
    }

private:
    SignalQuality m_signalQuality;      // Current Link signal quality
};

} // namespace hw_sdk_mock

#endif // MOCK_LINK_HPP
