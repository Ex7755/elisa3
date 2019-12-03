#ifndef EEPROM_IO_H
#define EEPROM_IO_H


#include <avr\eeprom.h>
#include "variables.h"

#ifdef __cplusplus
extern "C" {
#endif

void writeCalibrationToFlash();
void readCalibrationFromFlash();


#ifdef __cplusplus
} // extern "C"
#endif

#endif

