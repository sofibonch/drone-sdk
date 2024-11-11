#include "link/link.hpp"
#include <iostream>
#include <random>
#include <string>

namespace hw_sdk_mock
{

    namespace
    {
        // Helper to generate random SignalQuality values
        Link::SignalQuality getRandomSignalQuality()
        {
            static std::random_device rd;
            static std::mt19937 gen(rd());
            std::uniform_int_distribution<int> dis(0, 4);
            return static_cast<Link::SignalQuality>(dis(gen));
        }

        // Converts SignalQuality enum to a readable string
        std::string signalQualityToString(Link::SignalQuality quality)
        {
            switch (quality)
            {
            case Link::SignalQuality::NO_SIGNAL:
                return "NO_SIGNAL";
            case Link::SignalQuality::POOR:
                return "POOR";
            case Link::SignalQuality::FAIR:
                return "FAIR";
            case Link::SignalQuality::GOOD:
                return "GOOD";
            case Link::SignalQuality::EXCELLENT:
                return "EXCELLENT";
            default:
                return "UNKNOWN_SIGNAL_QUALITY";
            }
        }
    }

    Link::SignalQuality Link::getSignalQuality()
    {
        SignalQuality quality = getRandomSignalQuality();
        std::cout << "Link Signal Quality: " << signalQualityToString(quality) << "\n";
        return quality;
    }

} // namespace hw_sdk_mock
