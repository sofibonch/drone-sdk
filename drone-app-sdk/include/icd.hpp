#ifndef ICD_HPP
#define ICD_HPP

namespace drone_sdk {

    enum class LocationField {
        LATITUDE,
        LONGITUDE,
        ALTITUDE
    };

    struct Location {
        double latitude;
        double longitude;
        double altitude;

        double getField(LocationField field) const;
    };

    enum class SignalQuality
    {
        NO_SIGNAL = 0,
        POOR,
        FAIR,
        GOOD,
        EXCELLENT
    };

    struct GpsCallback {
        using Type = void(*)(Location, SignalQuality);
    };

    struct LinkCallback {
        using Type = void(*)(SignalQuality);
    };

} // namespace drone_sdk

#endif // ICD_HPP
