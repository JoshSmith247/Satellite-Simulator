#ifndef SIM_CLOCK_HPP
#define SIM_CLOCK_HPP

#include "DateTime.h"

// Controllable simulation clock: pause, change speed, jump in time. Every
// time-dependent subsystem reads its instant from here, not DateTime::Now().
class SimClock {
public:
    SimClock();

    // Advance by realDtSeconds of wall time scaled by speed; no-op while paused.
    void update(double realDtSeconds);

    void   pause()            { paused_ = true; }
    void   resume()           { paused_ = false; }
    void   togglePause()      { paused_ = !paused_; }
    bool   isPaused() const   { return paused_; }

    // Speed multiplier: 1.0 = real time, 60.0 = a minute per second, etc.
    void   setSpeed(double s) { speed_ = s; }
    double speed() const      { return speed_; }

    // Jump to the real wall-clock instant and resume 1x.
    void   resetToNow();
    void   jumpSeconds(double seconds) { unixSeconds_ += seconds; }
    void   setUnixSeconds(double s) { unixSeconds_ = s; }
    void   setDateTime(const libsgp4::DateTime& dt) { unixSeconds_ = toUnixSeconds(dt); }

    static double toUnixSeconds(const libsgp4::DateTime& dt);

    // Current simulation instant in various forms.
    double            unixSeconds() const { return unixSeconds_; }
    double            julianDate()  const;
    libsgp4::DateTime dateTime()    const;

private:
    double unixSeconds_;   // current simulation instant, Unix epoch seconds (UTC)
    double speed_  = 1.0;
    bool   paused_ = false;
};

#endif
