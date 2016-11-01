#ifndef __IMU_MOTION_H
#define __IMU_MOTION_H	

#include "sys.h"
#include "math.h"

#define q30  1073741824.0f

void Imu_motion_read(void);
void Imu_motion_timer(void);

#endif
