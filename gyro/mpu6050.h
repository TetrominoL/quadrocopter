
#define MPU_RAW_ADDRESS 0xD0    //AD0->0; IF AD0->1 then TWI_ADDRESS is 0xD2
#define MPU_ADD_READ    0x01
#define MPU_ADD_WRITE   0x00

#define MPU_ADDR_W (MPU_RAW_ADDRESS | MPU_ADD_WRITE)
#define MPU_ADDR_R (MPU_RAW_ADDRESS | MPU_ADD_READ)

#include <stdint.h>

#define mpu_gyro_x ((((uint16_t)mpu_gyro_val.x_h) << 8) | mpu_gyro_val.x_l)
#define mpu_gyro_y ((((uint16_t)mpu_gyro_val.y_h) << 8) | mpu_gyro_val.y_l)
#define mpu_gyro_z ((((uint16_t)mpu_gyro_val.z_h) << 8) | mpu_gyro_val.z_l)

#define mpu_acc_x ((((uint16_t)mpu_acc_val.x_h) << 8) | mpu_acc_val.x_l)
#define mpu_acc_y ((((uint16_t)mpu_acc_val.y_h) << 8) | mpu_acc_val.y_l)
#define mpu_acc_z ((((uint16_t)mpu_acc_val.z_h) << 8) | mpu_acc_val.z_l)

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

uint8_t mpu_init();

uint8_t mpu_write_addr(uint8_t regaddr, uint8_t regval);

uint8_t mpu_read_addr(uint8_t regaddr, uint8_t *regval);

uint8_t mpu_signal_path_reset(uint8_t sources);

uint8_t mpu_get_gyro_data();

uint8_t mpu_get_acc_data();

uint8_t mpu_burst_read_addr(uint8_t regaddr, uint8_t *regval, uint8_t cnt);

uint8_t mpu_update_all_sensor_data();