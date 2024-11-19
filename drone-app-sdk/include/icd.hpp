#ifndef ICD_HPP
#define ICD_HPP

namespace drone_sdk
{

    enum class LocationField
    {
        LATITUDE,
        LONGITUDE,
        ALTITUDE
    };

    enum class safetyState
    {
        GPS_HEALTH = 0,
        GPS_NOT_HEALTHY,
        CONNECTED,
        NOT_CONNECTED
    };

struct Location
{
    double latitude;
    double longitude;
    double altitude;

    // Default constructor
    Location() : latitude(0.0), longitude(0.0), altitude(0.0) {}

    // Constructor with parameters
    Location(double lat, double lon, double alt)
        : latitude(lat), longitude(lon), altitude(alt) {}

    // Copy constructor
    Location(const Location &other)
        : latitude(other.latitude), longitude(other.longitude), altitude(other.altitude) {}

    // Assignment operator
    Location &operator=(const Location &other)
    {
        if (this != &other) { // Check for self-assignment
            latitude = other.latitude;
            longitude = other.longitude;
            altitude = other.altitude;
        }
        return *this;
    }

    // Equality operator
    bool operator==(const Location &other) const
    {
        return latitude == other.latitude &&
               longitude == other.longitude &&
               altitude == other.altitude;
    }
};


    enum class SignalQuality
    {
        NO_SIGNAL = 0,
        POOR,
        FAIR,
        GOOD,
        EXCELLENT
    };

    enum class FlightControllerStatus
    {
        SUCCESS = 0,
        EMERGENCY_LAND,
        EMERGENCY_GO_HOME,
        EMERGENCY_ABORTED_MISSION,
        CONNECTION_ERROR,
        HARDWARE_ERROR,
        INVALID_COMMAND,
        UNKNOWN_ERROR
    };

    struct GpsCallback
    {
        using Type = void (*)(Location, SignalQuality);
    };

    struct GpsLocCallback
    {
        using Type = void (*)(Location);
    };

    struct LinkCallback
    {
        using Type = void (*)(SignalQuality);
    };
    enum class FlightState
    {
        LANDED=0,         // Drone is grounded and idle, ready to begin tasks.
        TAKEOFF,        // Drone ascends to a defined operational altitude.
        AIRBORNE,       // Drone is executing tasks in-flight.
        HOVER,          // Drone maintains a stable hover, waiting or observing.
        EMERGENCY_LAND, // Drone descends immediately to a safe landing due to critical safety issues.
        RETURN_HOME     // Drone returns to its home location, typically after completing a task.
    };

    enum class CurrentMission
    {
        LANDED = 0,
        GOTO,
        PATH,
        HOVER,
        HOME,
        EMERGENCY
    };

    enum class CommandStatus
    {
        IDLE = 0,
        BUSY,
        MISSION_ABORT
    };
} // namespace drone_sdk

#endif // ICD_HPP
