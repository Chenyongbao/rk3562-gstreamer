#include "focal_range_normalizer.h"

#include <cmath>
#include <cstdio>

namespace {

constexpr double kNewProjectorShortMin = 6.0;
constexpr double kNewProjectorShortMax = 10.0;
constexpr double kNewProjectorShortFallback = 8.0;
constexpr double kNewProjectorLongMin = 10.0;
constexpr double kNewProjectorLongMax = 14.0;
constexpr double kNewProjectorLongFallback = 12.0;

bool isStrictlyBetween(double value, double lo, double hi) {
    return std::isfinite(value) && value > lo && value < hi;
}

void clampFocusValue(const char* label,
                     double min_value,
                     double max_value,
                     double fallback_value,
                     double& value) {
    if (isStrictlyBetween(value, min_value, max_value)) {
        return;
    }
    std::fprintf(stderr,
                 "[FocalAutoFocus] %s focal %.3f outside (%.3f, %.3f), forcing fallback %.3f\n",
                 label ? label : "unknown",
                 value,
                 min_value,
                 max_value,
                 fallback_value);
    value = fallback_value;
}

} // namespace

void normalizeFocalResultForNewProjectorRanges(FocalAutoFocusResult& result) {
    clampFocusValue("new/short",
                    kNewProjectorShortMin,
                    kNewProjectorShortMax,
                    kNewProjectorShortFallback,
                    result.focal_short);
    clampFocusValue("new/long",
                    kNewProjectorLongMin,
                    kNewProjectorLongMax,
                    kNewProjectorLongFallback,
                    result.focal_long);

    result.short_focus.focal_distance_mm = result.focal_short;
    result.long_focus.focal_distance_mm = result.focal_long;
}
