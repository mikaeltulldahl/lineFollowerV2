#ifndef MAIN_H
#define MAIN_H

#include "master_include.h"

volatile uint32_t state;
volatile uint32_t begun_backing_tick;
volatile float acc_filt[3];
volatile float acc_angle;

#endif //MAIN_H
