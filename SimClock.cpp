#include "SimClock.hpp"
#include <chrono>

namespace {
    // libsgp4::DateTime ticks are microseconds since 0001-01-01 (TimeSpan::TicksPerSecond
    // == 1e6). 719162 days elapse from then to the Unix epoch (1970-01-01); this constant,
    // verified against DateTime(1970,1,1).Ticks(), lets us build one from Unix seconds.
    constexpr int64_t UNIX_EPOCH_TICKS = 719162LL * 86400LL * 1000000LL;
    constexpr double  TICKS_PER_SECOND = 1.0e6;

    double wallClockUnixSeconds() {
        auto now = std::chrono::system_clock::now().time_since_epoch();
        return std::chrono::duration_cast<std::chrono::milliseconds>(now).count() / 1000.0;
    }
}

SimClock::SimClock() : unixSeconds_(wallClockUnixSeconds()) {}

void SimClock::update(double realDtSeconds) {
    if (!paused_)
        unixSeconds_ += realDtSeconds * speed_;
}

void SimClock::resetToNow() {
    unixSeconds_ = wallClockUnixSeconds();
    speed_  = 1.0;
    paused_ = false;
}

double SimClock::julianDate() const {
    // 2440587.5 is the Julian Date of the Unix epoch.
    return unixSeconds_ / 86400.0 + 2440587.5;
}

libsgp4::DateTime SimClock::dateTime() const {
    return libsgp4::DateTime(
        static_cast<int64_t>(UNIX_EPOCH_TICKS + unixSeconds_ * TICKS_PER_SECOND));
}

double SimClock::toUnixSeconds(const libsgp4::DateTime& dt) {
    return (double)(dt.Ticks() - UNIX_EPOCH_TICKS) / TICKS_PER_SECOND;
}
