#include <NanMath.h>
#include <math.h>

float NanMath::nanIfOutOfBounds(float value, float min, float max) {
    if (isnan(value) || value < min || value > max ) {
        return NAN;
    }

    return value;
}

float NanMath::averageIgnoringNan(float value1, float value2) {
  if (isnan(value1) && isnan(value2)) {
    return NAN;
  } else if (isnan(value1)) {
    return value2;
  } else if (isnan(value2)) {
    return value1;
  } else {
    return (value1 + value2) / 2;
  }
}