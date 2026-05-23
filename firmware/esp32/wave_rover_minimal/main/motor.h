#pragma once

#include <stdint.h>

void wr_motor_init(void);
void wr_motor_set_milli(int16_t left_milli, int16_t right_milli);
void wr_motor_stop(void);
