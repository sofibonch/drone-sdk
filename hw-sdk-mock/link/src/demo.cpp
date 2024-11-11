#include "link/link.hpp"
#include <iostream>
int main()
{
    hw_sdk_mock::Link link;

    for (int i = 0; i < 5; ++i)
    {
        auto quality = link.getSignalQuality();

        std::cout << "Signal Quality: " << static_cast<int>(quality) << "\n";
    }

    return 0;
}
