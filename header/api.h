#ifndef __API_H__
#define __API_H__

#include "halGPIO.h"

#define SCAN_STEP       1
#define NUM_OF_SAMPLES  181

void scan_with_motor();
void scan_at_given_angle();
void counting();

#endif
