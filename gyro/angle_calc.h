
#include <stdint.h>
#define DEGREE_PER_VALUE	((float)(2000/32768.0))		//dps
#define DT_GYRO_ASKING_TIME 0.002

#define SHARE_GYRO  0
#define SHARE_ACC   (1-SHARE_GYRO)


double _angle_x, _angle_y, _angle_z;

void angle_init();

void angle_update(int16_t gyro_x, int16_t gyro_y, int16_t gyro_z, int16_t acc_x, int16_t acc_y, int16_t acc_z);