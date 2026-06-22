#ifndef PASS_PREDICT_HPP
#define PASS_PREDICT_HPP

#include <vector>
#include "SGP4.h"
#include "Observer.h"
#include "DateTime.h"
#include "CoordGeodetic.h"

// Observer-relative geometry and pass prediction for a tracked satellite.
// Built on libsgp4's Observer/GetLookAngle, which yields azimuth, elevation,
// slant range and range-rate — everything needed to point an antenna, schedule
// a contact, and compute the Doppler shift on a radio link.
namespace PassPredict {

    // Instantaneous look angle from a ground station to the satellite.
    struct LookAngle {
        double azimuthDeg   = 0.0;  // 0 = North, clockwise
        double elevationDeg = 0.0;  // above the local horizon (negative = below)
        double rangeKm      = 0.0;  // slant range
        double rangeRateKmS = 0.0;  // +ve = receding (red shift), -ve = approaching
        bool   visible      = false;// elevation above the mask
    };

    // A single above-the-horizon pass over the ground station.
    struct Pass {
        libsgp4::DateTime aos;          // acquisition of signal (rise)
        libsgp4::DateTime los;          // loss of signal (set)
        libsgp4::DateTime tca;          // time of closest approach (peak elevation)
        double maxElevationDeg = 0.0;
        double aosAzimuthDeg   = 0.0;
        double losAzimuthDeg   = 0.0;
        double durationSec     = 0.0;
    };

    // Look angle at a single instant.
    LookAngle lookAngle(const libsgp4::SGP4& sgp4,
                        const libsgp4::CoordGeodetic& station,
                        const libsgp4::DateTime& when,
                        double elevationMaskDeg = 0.0);

    // Upcoming passes whose peak elevation clears elevationMaskDeg, scanning
    // [start, start + horizonHours]. AOS/LOS are refined to ~1 second.
    std::vector<Pass> predictPasses(const libsgp4::SGP4& sgp4,
                                    const libsgp4::CoordGeodetic& station,
                                    const libsgp4::DateTime& start,
                                    double horizonHours,
                                    double elevationMaskDeg = 5.0);

    // Doppler-shifted observed frequency (Hz) for an emitted frequency, given the
    // range-rate. f_obs = f_emit * (1 - rangeRate / c).
    double dopplerShiftedHz(double emittedHz, double rangeRateKmS);

} // namespace PassPredict

#endif
