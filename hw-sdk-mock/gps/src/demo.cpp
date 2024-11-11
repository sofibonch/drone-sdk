#include "gps/gps.hpp"
#include <iostream>

int main()
{
    hw_sdk_mock::Gps gps;

    for (int i = 0; i < 5; ++i)
    {
        auto location = gps.getLocation();
        auto quality = gps.getSignalQuality();

        std::cout << "Location: " << location.latitude << ", " << location.longitude << ", " << location.altitude << " meters\n";
        std::cout << "Signal Quality: " << static_cast<int>(quality) << "\n";
    }

    return 0;
}
