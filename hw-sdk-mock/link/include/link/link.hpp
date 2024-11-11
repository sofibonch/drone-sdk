#pragma once

namespace hw_sdk_mock
{
    class Link
    {
    public:
        enum class SignalQuality
        {
            NO_SIGNAL = 0,
            POOR,
            FAIR,
            GOOD,
            EXCELLENT

        };

        Link() = default;
        virtual ~Link() = default;
        Link(const Link &) = delete;
        Link &operator=(const Link &) = delete;
        Link(Link &&) = default;
        Link &operator=(Link &&) = default;

        SignalQuality getSignalQuality();
    };
} // namespace hw_sdk_mock
