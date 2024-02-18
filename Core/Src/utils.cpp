#include <utils.hpp>
#include <tim.h>

// ==================================================
// Functions
// ==================================================

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

float remap(float v, float a1, float b1, float a2, float b2, bool clamp = false) {
  float res = a2 + (v - a1) / (b1 - a1) * (b2 - a2);
  if(clamp){
    if(a2 <= b2) {
      return fmin(fmax(res, a2), b2);
    } else {
      return fmin(fmax(res, b2), a2);
    }
  } else {
    return res;
  }
}

float remap(long v,  long a1,  long b1, float a2, float b2, bool clamp = false){
  return remap((float) v, (float) a1, (float) b1, a2, b2, clamp);
}

long remap(float v, float a1, float b1,  long a2,  long b2, bool clamp = false){
  return round(remap(v, a1, b1, (float) a2, (float) b2, clamp));
}
long remap( long v,  long a1,  long b1,  long a2,  long b2, bool clamp = false){
  return round(remap((float) v, (float) a1, (float) b1, (float) a2, (float) b2, clamp));
}

/*uint32_t getCurrentTime() {
    return HAL_GetTick();
}

uint32_t getElapsedTime(uint32_t startTime) {
    uint32_t currentTime = getCurrentTime();
    uint32_t elapsedTime;

    if (currentTime >= startTime) {
        elapsedTime = currentTime - startTime;
    } else {
        // Handle timer overflow
        elapsedTime = currentTime - startTime + UINT32_MAX;
    }

    return elapsedTime;
}*/
