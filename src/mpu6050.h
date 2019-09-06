
#define MPU_RAW_ADDRESS 0xD0    //AD0->0; IF AD0->1 then TWI_ADDRESS is 0xD2
#define MPU_ADD_READ    0x01
#define MPU_ADD_WRITE   0x00


#define SMPRT_DIV       25
#define CONFIGURATION	26
#define GYRO_CONFIG		27
#define ACCEL_CONFIG	28
#define ACCEL_CONFIG2	29
#define FIFO_EN         35
#define INT_ENABLE      56
#define ACCEL_XOUT_H	59
#define ACCEL_XOUT_L	60
#define ACCEL_YOUT_H	61
#define ACCEL_YOUT_L	62
#define ACCEL_ZOUT_H	63
#define ACCEL_ZOUT_L	64
#define EMP_OUT_H		65
#define TEMP_OUT_H		65
#define TEMP_OUT_L		66
#define GYRO_XOUT_H		67
#define GYRO_XOUT_L		68
#define GYRO_YOUT_H		69
#define GYRO_YOUT_L		70
#define GYRO_ZOUT_H		71
#define GYRO_ZOUT_L		72
#define PWR_MGMT_1		107	
#define USER_CTRL	    106

#define I2C_IF_DIS_PS		4
#define GYRO_FS_SEL_PS	    3
#define ZGYRO_CTEN_PS		5
#define YGYRO_CTEN_PS		6
#define XGYRO_Cten_PS		7
#define ACCEL_FS_SEL_PS	    3
#define AZ_ST_EN_PS			5
#define AY_ST_EN_PS			6
#define AX_ST_EN_PS			7

#define	ACC_SC_2G		0x00
#define ACC_SC_4G		0x01
#define ACC_SC_8G		0x02
#define ACC_SC_16G	    0x03

#define GYRO_SC_250DPS	0x00
#define GYRO_SC_500DPS	0x01
#define GYRO_SC_1000DPS	0x02
#define GYRO_SC_2000DPS	0x03


#define MPU_ADDR_W (MPU_RAW_ADDRESS | MPU_ADD_WRITE)
#define MPU_ADDR_R (MPU_RAW_ADDRESS | MPU_ADD_READ)

#include <stdint.h>

#define mpu_gyro_x ((int16_t)((((uint16_t)mpu_gyro_val.x_h) << 8) | mpu_gyro_val.x_l) - mpu_gyro_corr_x)
#define mpu_gyro_y ((int16_t)((((uint16_t)mpu_gyro_val.y_h) << 8) | mpu_gyro_val.y_l) - mpu_gyro_corr_y)
#define mpu_gyro_z ((int16_t)((((uint16_t)mpu_gyro_val.z_h) << 8) | mpu_gyro_val.z_l) - mpu_gyro_corr_z)

#define mpu_acc_x ((int16_t)((((uint16_t)mpu_acc_val.x_h) << 8) | mpu_acc_val.x_l) - mpu_acc_corr_x)
#define mpu_acc_y ((int16_t)((((uint16_t)mpu_acc_val.y_h) << 8) | mpu_acc_val.y_l) - mpu_acc_corr_y)
#define mpu_acc_z ((int16_t)((((uint16_t)mpu_acc_val.z_h) << 8) | mpu_acc_val.z_l) - mpu_acc_corr_z)

#define mpu_gyro_x_raw ((int16_t)(((uint16_t)mpu_gyro_val.x_h) << 8) | mpu_gyro_val.x_l)
#define mpu_gyro_y_raw ((int16_t)(((uint16_t)mpu_gyro_val.y_h) << 8) | mpu_gyro_val.y_l)
#define mpu_gyro_z_raw ((int16_t)(((uint16_t)mpu_gyro_val.z_h) << 8) | mpu_gyro_val.z_l)

#define mpu_acc_x_raw ((int16_t)(((uint16_t)mpu_acc_val.x_h) << 8) | mpu_acc_val.x_l)
#define mpu_acc_y_raw ((int16_t)(((uint16_t)mpu_acc_val.y_h) << 8) | mpu_acc_val.y_l)
#define mpu_acc_z_raw ((int16_t)(((uint16_t)mpu_acc_val.z_h) << 8) | mpu_acc_val.z_l)

#pragma pack(push,1)
struct {
    uint8_t x_h;
    uint8_t x_l;
    uint8_t y_h;
    uint8_t y_l;
    uint8_t z_h;
    uint8_t z_l;
}mpu_acc_val,mpu_gyro_val;
#pragma pack(pop)

int16_t mpu_acc_corr_x;
int16_t mpu_acc_corr_y;
int16_t mpu_acc_corr_z;

int16_t mpu_gyro_corr_x;
int16_t mpu_gyro_corr_y;
int16_t mpu_gyro_corr_z;

uint8_t mpu_init();

uint8_t mpu_write_addr(uint8_t regaddr, uint8_t regval);

uint8_t mpu_read_addr(uint8_t regaddr, uint8_t *regval);

uint8_t mpu_signal_path_reset(uint8_t sources);

uint8_t mpu_get_gyro_data();

uint8_t mpu_get_acc_data();

uint8_t mpu_burst_read_addr(uint8_t regaddr, uint8_t *regval, uint8_t cnt);

uint8_t mpu_update_all_sensor_data();

uint8_t mpu_scale_gyro(uint8_t scale);

uint8_t mpu_scale_accel(uint8_t scale);

void mpu_calc_initvalues(uint8_t amount);