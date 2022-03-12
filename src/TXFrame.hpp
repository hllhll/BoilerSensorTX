// Test calculations, expirimental
#define MIN_TEMP 10
#define MAX_TEMP 70
#define PRECITION 0.5

// 10-70 @ 0.5  ==> 120, That's ~ 2^7, So lets keep it 1 Byte
#define STEPS ((int)(1/PRECITION)*(MAX_TEMP-MIN_TEMP))

#define MESURE_SIZE 1

#include "Sensor.hpp"

SensorSnapshot