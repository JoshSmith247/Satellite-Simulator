#ifndef SIM_CLOCK_HPP
#define SIM_CLOCK_HPP

#include "DateTime.h"

// Controllable simulation clock. Decouples the simulation from wall-clock "now"
// so the user can pause, speed up / slow down, and jump to an arbitrary instant
// to inspect future (or past) passes. Every time-dependent subsystem — Earth
// rotation, Sun position, SGP4 propagation, ground tracks, pass prediction —
// reads its instant from here instead of calling DateTime::Now() directly.
class SimClock {
public:
    SimClock();

    // Advance simulation time by realDtSeconds of wall time scaled by the current
    // speed multiplier. A no-op while paused. Call once per frame.
    void update(double realDtSeconds);

    void   pause()            { paused_ = true; }
    void   resume()           { paused_ = false; }
    void   togglePause()      { paused_ = !paused_; }
    bool   isPaused() const   { return paused_; }

    // Speed multiplier: 1.0 = real time, 60.0 = a minute per second, etc.
    void   setSpeed(double s) { speed_ = s; }
    double speed() const      { return speed_; }

    // Jump the simulation to the real wall-clock instant ("now") and resume 1x.
    void   resetToNow();
    // Offset the current simulation instant by the given number of seconds.
    void   jumpSeconds(double seconds) { unixSeconds_ += seconds; }
    // Jump the simulation to a specific instant.
    void   setUnixSeconds(double s) { unixSeconds_ = s; }
    void   setDateTime(const libsgp4::DateTime& dt) { unixSeconds_ = toUnixSeconds(dt); }

    // Convert a libsgp4::DateTime to Unix epoch seconds (UTC).
    static double toUnixSeconds(const libsgp4::DateTime& dt);

    // Current simulation instant in various forms.
    double            unixSeconds() const { return unixSeconds_; }
    double            julianDate()  const;
    libsgp4::DateTime dateTime()    const;

private:
    double unixSeconds_;   // current simulation instant, Unix epoch seconds (UTC)
    double speed_  = 1.0;  // time multiplier
    bool   paused_ = false;
};

#endif
