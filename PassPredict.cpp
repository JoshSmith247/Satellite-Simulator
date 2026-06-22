#include "PassPredict.hpp"
#include "CoordTopocentric.h"
#include "Eci.h"
#include <cmath>

namespace PassPredict {

using libsgp4::DateTime;
using libsgp4::Observer;
using libsgp4::CoordTopocentric;

namespace {
    constexpr double RAD2DEG    = 57.29577951308232;
    constexpr double C_KM_S     = 299792.458;  // speed of light, km/s

    // Elevation (radians) of the satellite seen from the station at an instant.
    double elevationAt(const libsgp4::SGP4& sgp4, Observer& obs, const DateTime& t) {
        CoordTopocentric topo = obs.GetLookAngle(sgp4.FindPosition(t));
        return topo.elevation;
    }

    // Bisection for the horizon crossing (elevation == 0) known to lie in (lo, hi),
    // where elevation has opposite signs at the two ends. Refines to ~1 second.
    DateTime refineCrossing(const libsgp4::SGP4& sgp4, Observer& obs,
                            DateTime lo, DateTime hi) {
        for (int i = 0; i < 32; ++i) {
            if ((hi - lo).TotalSeconds() < 1.0) break;
            DateTime mid(lo.Ticks() + (hi - lo).Ticks() / 2);
            double eLo  = elevationAt(sgp4, obs, lo);
            double eMid = elevationAt(sgp4, obs, mid);
            if ((eLo < 0.0) == (eMid < 0.0)) lo = mid;
            else                             hi = mid;
        }
        return DateTime(lo.Ticks() + (hi - lo).Ticks() / 2);
    }
}

LookAngle lookAngle(const libsgp4::SGP4& sgp4,
                    const libsgp4::CoordGeodetic& station,
                    const DateTime& when,
                    double elevationMaskDeg) {
    Observer obs(station);
    CoordTopocentric topo = obs.GetLookAngle(sgp4.FindPosition(when));
    LookAngle la;
    la.azimuthDeg   = topo.azimuth   * RAD2DEG;
    la.elevationDeg = topo.elevation * RAD2DEG;
    la.rangeKm      = topo.range;
    la.rangeRateKmS = topo.range_rate;
    la.visible      = la.elevationDeg >= elevationMaskDeg;
    return la;
}

std::vector<Pass> predictPasses(const libsgp4::SGP4& sgp4,
                                const libsgp4::CoordGeodetic& station,
                                const DateTime& start,
                                double horizonHours,
                                double elevationMaskDeg) {
    std::vector<Pass> passes;
    Observer obs(station);

    const double coarseStepSec = 30.0;           // scan granularity
    const double horizonSec    = horizonHours * 3600.0;
    const DateTime end         = start.AddSeconds(horizonSec);

    DateTime t    = start;
    double   ePrev = elevationAt(sgp4, obs, t);

    while (t < end) {
        DateTime tNext = t.AddSeconds(coarseStepSec);
        if (tNext > end) tNext = end;
        double eNext = elevationAt(sgp4, obs, tNext);

        // Rising edge: satellite crosses above the horizon -> AOS.
        if (ePrev < 0.0 && eNext >= 0.0) {
            DateTime aos = refineCrossing(sgp4, obs, t, tNext);

            // Walk forward to the setting edge, tracking peak elevation (TCA).
            DateTime a     = aos;
            DateTime b     = aos.AddSeconds(coarseStepSec);
            double   eA    = elevationAt(sgp4, obs, a);
            double   maxEl = eA;
            DateTime tca   = a;
            while (b < end) {
                double eB = elevationAt(sgp4, obs, b);
                if (eB > maxEl) { maxEl = eB; tca = b; }
                if (eB < 0.0) break;           // crossed below -> LOS bracket found
                a  = b;
                eA = eB;
                b  = b.AddSeconds(coarseStepSec);
            }
            DateTime los = refineCrossing(sgp4, obs, a, b);

            // Refine TCA around the coarse peak with a few bisection-like probes.
            {
                DateTime lo = tca.AddSeconds(-coarseStepSec);
                DateTime hi = tca.AddSeconds(coarseStepSec);
                for (int i = 0; i < 24; ++i) {
                    if ((hi - lo).TotalSeconds() < 1.0) break;
                    DateTime m1(lo.Ticks() + (hi - lo).Ticks() / 3);
                    DateTime m2(lo.Ticks() + 2 * (hi - lo).Ticks() / 3);
                    if (elevationAt(sgp4, obs, m1) < elevationAt(sgp4, obs, m2)) lo = m1;
                    else                                                         hi = m2;
                }
                tca   = DateTime(lo.Ticks() + (hi - lo).Ticks() / 2);
                maxEl = elevationAt(sgp4, obs, tca);
            }

            if (maxEl * RAD2DEG >= elevationMaskDeg) {
                Pass p;
                p.aos = aos;
                p.los = los;
                p.tca = tca;
                p.maxElevationDeg = maxEl * RAD2DEG;
                p.aosAzimuthDeg   = obs.GetLookAngle(sgp4.FindPosition(aos)).azimuth * RAD2DEG;
                p.losAzimuthDeg   = obs.GetLookAngle(sgp4.FindPosition(los)).azimuth * RAD2DEG;
                p.durationSec     = (los - aos).TotalSeconds();
                passes.push_back(p);
            }

            // Continue scanning just past this pass's LOS.
            t     = los.AddSeconds(coarseStepSec);
            ePrev = elevationAt(sgp4, obs, t);
            continue;
        }

        t     = tNext;
        ePrev = eNext;
    }

    return passes;
}

double dopplerShiftedHz(double emittedHz, double rangeRateKmS) {
    return emittedHz * (1.0 - rangeRateKmS / C_KM_S);
}

} // namespace PassPredict
