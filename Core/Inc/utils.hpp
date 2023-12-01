#ifndef UTILS_HPP
#define UTILS_HPP

#define PI 3.1415926535897932384626433832795028841971693993751058209

#include <math.h>
#include <cstdint>

template <typename T>
int sgn(T val);

float remap(float v, float a1, float b1, float a2, float b2, bool clamp);
float remap(long v,  long a1,  long b1, float a2, float b2, bool clamp);

double remap(float v, float a1, float b1,  double a2, double b2, bool clamp);
double remap(double v, double a1, double b1, double a2, double b2, bool clamp);

uint32_t getCurrentTime();
uint32_t getElapsedTime(uint32_t startTime);

#endif  // UTILS_HPP
