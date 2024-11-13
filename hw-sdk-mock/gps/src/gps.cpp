#include "gps/gps.hpp"

#include <iomanip>
#include <iostream>
#include <random>

namespace hw_sdk_mock
{

    // Helper to generate random double values within a specified range
    double getRandomDouble(double min, double max)
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(min, max);
        return dis(gen);
    }

    // Helper to generate random SignalQuality values
    Gps::SignalQuality getRandomSignalQuality()
    {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_int_distribution<int> dis(0, 4);
        return static_cast<Gps::SignalQuality>(dis(gen));
    }

    Gps::Location Gps::getLocation()
    {
        Location location{
            getRandomDouble(-90.0, 90.0),   // Latitude range
            getRandomDouble(-180.0, 180.0), // Longitude range
            getRandomDouble(0.0, 10000.0)   // Altitude range in meters
        };

        //std::cout << std::fixed << std::setprecision(6)
        //          << "GPS Location - Latitude: " << location.latitude
        //          << ", Longitude: " << location.longitude
        //          << ", Altitude: " << location.altitude << " meters\n";
//
        return location;
    }

    Gps::SignalQuality Gps::getSignalQuality()
    {
        SignalQuality quality = getRandomSignalQuality();

        std::string qualityStr;
        switch (quality)
        {
        case SignalQuality::NO_SIGNAL:
            qualityStr = "NO_SIGNAL";
            break;
        case SignalQuality::POOR:
            qualityStr = "POOR";
            break;
        case SignalQuality::FAIR:
            qualityStr = "FAIR";
            break;
        case SignalQuality::GOOD:
            qualityStr = "GOOD";
            break;
        case SignalQuality::EXCELLENT:
            qualityStr = "EXCELLENT";
            break;
        }

        std::cout << "GPS Signal Quality: " << qualityStr << "\n";
        return quality;
    }

} // namespace hw_sdk_mock
