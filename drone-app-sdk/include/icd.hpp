// icd.hpp
#ifndef ICD_HPP
#define ICD_HPP

namespace drone_sdk {

    enum class LocationField {
        Latitude,
        Longitude,
        Altitude
    };

    struct Location {
        double latitude;
        double longitude;
        double altitude;

        double getField(LocationField field) const;
    };

    enum class SignalQuality {
        Excellent = 4,
        Good = 3,
        Fair = 2,
        Poor = 1,
        NoSignal = 0
    };

    struct GpsCallback {
        using Type = void(*)(Location, SignalQuality);
    };

    struct LinkCallback {
        using Type = void(*)(SignalQuality);
    };

} // namespace drone_sdk

#endif // ICD_HPP
