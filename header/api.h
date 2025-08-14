#ifndef __API_H__
#define __API_H__

#include "halGPIO.h"
#include <cstdint>

#define SCAN_STEP       1
#define NUM_OF_SAMPLES  181

void scan_with_motor(uint8_t type);
void scan_for_distance();
void scan_at_given_angle();
void counting();

#endif
