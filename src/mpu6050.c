
#include "mpu6050.h"
#include <avr/io.h>
#include "twi.h"
#include "usart.h"
#include <util/delay.h>

#define GYRO_SCALE  GYRO_SC_250DPS
#define ACC_SCALE   ACC_SC_2G

#if GYRO_SCALE == GYRO_SC_250DPS
    
#endif

#if ACC_SCALE == ACC_SC_2G
    #define ACC_VALUE_PER_G 16384
#elif ACC_SCALE == ACC_SC_4G
    #define ACC_VALUE_PER_G 8192
#endif

uint8_t mpu_init(){
    twi_init();
    _delay_ms(10);

    mpu_write_addr(PWR_MGMT_1,0x80);	//Reset all registers
	_delay_ms(30);
    mpu_write_addr(PWR_MGMT_1,0x00);	//No sleep
	_delay_ms(10);

    //disable output to FIFO buffer
    mpu_write_addr(0x6A,0x88);
    //Power Management: GyroZ PLL Reference
	mpu_write_addr(PWR_MGMT_1,0x03);	//Set Gyroscop x as clocksource as recommended
	_delay_ms(200);
    //mpu_write_addr(0x6b,0x00);  //On Powerup MPU is in Sleepmode (Bit6=1)
    //mpu_write_addr(0x6a,0x00);  //Disable Fifo and spi and I2CMaster
    mpu_signal_path_reset(0x07);

    mpu_write_addr(SMPRT_DIV, 0x00);    //No Divider for Clkfreq
    //Low-Pass-Filter 1kHz, Bandwith 5Hz
    mpu_write_addr(CONFIGURATION,0x03);	// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    mpu_write_addr(FIFO_EN, 0x00);  //Disalble FIFO
    mpu_write_addr(INT_ENABLE,0x00); //Disable Interrupts
	mpu_scale_gyro(GYRO_SCALE);
	mpu_scale_accel(ACC_SCALE);
    _delay_ms(500); //Lange Wartezeit bis Werte soweit normal, dass sinvoller Durchschnitt gebildet werden kann

    mpu_calc_initvalues(100);
    
    return 0;
}

/**
 * Calculate correction value for all gyro- and acc-sensor raw data
 * @param amount for calculate average 
 */
void mpu_calc_initvalues(uint8_t amount){
    int32_t avg_gyro_x,avg_gyro_y,avg_gyro_z, avg_acc_x,avg_acc_y,avg_acc_z;
    uint8_t i = amount;
    /*
    Max for amount is 65535 because sensorvalues are 2^16 and avg-datatype is 2^32;
    */
    avg_gyro_x = avg_gyro_y = avg_gyro_z = avg_acc_x = avg_acc_y = avg_acc_z = 0;
    
    i=amount;
    while(i){

        mpu_update_all_sensor_data();
        avg_gyro_x += (int16_t)mpu_gyro_x_raw;
        avg_gyro_y += (int16_t)mpu_gyro_y_raw;
        avg_gyro_z += (int16_t)mpu_gyro_z_raw;

        avg_acc_x += (int16_t)mpu_acc_x_raw;
        avg_acc_y += (int16_t)mpu_acc_y_raw;
        avg_acc_z += (int16_t)mpu_acc_z_raw;

        i--;
        _delay_ms(2);
    }
    // mpu_acc_corr_x = avg_acc_x / amount;
    // mpu_acc_corr_y = avg_acc_y / amount;
    // mpu_acc_corr_z = avg_acc_z / amount- ACC_VALUE_PER_G;
    mpu_acc_corr_x = 0;
    mpu_acc_corr_y = 0;
    mpu_acc_corr_z = 0;
    
    mpu_gyro_corr_x = avg_gyro_x / amount;
    mpu_gyro_corr_y = avg_gyro_y / amount;
    mpu_gyro_corr_z = avg_gyro_z / amount;
}

/**
 * Run Signal path reset for given sources
 * @param sources Bit2=gyro, Bit1=accelerator, Bit0=temperatur so 0x07 will reset all
 * @return 0 on success, else !=0
 */
uint8_t mpu_signal_path_reset(uint8_t sources){
    return mpu_write_addr(0x68,sources);
}

uint8_t mpu_get_gyro_data(){
    return mpu_burst_read_addr(0x43, (uint8_t*)(&mpu_gyro_val), 6);
}

uint8_t mpu_get_acc_data(){
    uint8_t ret;
    const float e=0.2;
    int16_t save_acc_x = mpu_acc_x_raw;
    int16_t save_acc_y = mpu_acc_y_raw;
    int16_t save_acc_z = mpu_acc_z_raw;
    ret = mpu_burst_read_addr(0x3b, (uint8_t*)(&mpu_acc_val), 6);
    save_acc_x = (int16_t)((1-e)*(float)mpu_acc_x_raw + e*(float)save_acc_x + 0.5);
    save_acc_y = (int16_t)((1-e)*(float)mpu_acc_y_raw + e*(float)save_acc_y + 0.5);
    save_acc_z = (int16_t)((1-e)*(float)mpu_acc_z_raw + e*(float)save_acc_z + 0.5);
    mpu_acc_val.x_h = (uint8_t)(save_acc_x>>8);
    mpu_acc_val.x_l = (uint8_t)(save_acc_x>>0);
    mpu_acc_val.y_h = (uint8_t)(save_acc_y>>8);
    mpu_acc_val.y_l = (uint8_t)(save_acc_y>>0);
    mpu_acc_val.z_h = (uint8_t)(save_acc_z>>8);
    mpu_acc_val.z_l = (uint8_t)(save_acc_z>>0);
}

uint8_t mpu_update_all_sensor_data(){
    uint8_t ret = 0;
    if(mpu_get_gyro_data()){
        ret = 1;
    }
    if(mpu_get_acc_data()){
        ret = 1;
    }
    return ret;
}

uint8_t mpu_write_addr(uint8_t regaddr, uint8_t regval){
    uint8_t ret;

    twi_start();
    if(!twi_write_byte(MPU_ADDR_W)){
        if(!twi_write_byte(regaddr)){
            if(!twi_write_byte(regval)){
                twi_stop();
                return 0;
            }
        }
    }
    twi_stop(); 
    return 1;
}

uint8_t mpu_read_addr(uint8_t regaddr, uint8_t *regval){
    twi_start();
    
    if(!twi_write_byte(MPU_ADDR_W)){
        if(!twi_write_byte(regaddr)){
            twi_start();
            if(!twi_write_byte(MPU_ADDR_R)){
                if(!twi_read_byte(regval,0)){
                    twi_stop();
                    return 0;
                }else{
                    debug("mpu_read 4 %x\n",MPU_ADDR_R);    
                }
            }else{
                debug("mpu_read 3\n");
            }
        }else{
           debug("mpu_read 2\n");
        }
    }else{
        debug("mpu_read 1\n");
    }
    twi_stop();
    return 1;
}

uint8_t mpu_burst_read_addr(uint8_t regaddr, uint8_t *regval, uint8_t cnt){
    uint8_t i;
    
    if(cnt == 0)
        return 1;

    twi_start();
    
    if(!twi_write_byte(MPU_ADDR_W)){
        if(!twi_write_byte(regaddr)){
            twi_start();
            if(!twi_write_byte(MPU_ADDR_R)){
                for(i=0;i < cnt-1;i++){
                    if(twi_read_byte(regval++,1)){
                        break;
                        debug("mpu_read 4 %x\n",MPU_ADDR_R);    
                    }
                }
                if(i>= cnt-1){
                    if(!twi_read_byte(regval,0)){
                        twi_stop();
                        return 0;
                    }
                }
            }else{
                debug("mpu_read 3\n");
            }
        }else{
           debug("mpu_read 2\n");
        }
    }else{
        debug("mpu_read 1\n");
    }
    twi_stop();
    return 1;
}

uint8_t mpu_scale_accel(uint8_t scale){
	uint8_t reg_val;
	uint8_t ret;
	
	ret = mpu_read_addr(ACCEL_CONFIG, &reg_val);
	if(ret)
		return ret;
	reg_val = (reg_val & ~(3<<ACCEL_FS_SEL_PS)) | (scale<<ACCEL_FS_SEL_PS);
	
	return mpu_write_addr(ACCEL_CONFIG, reg_val);
}

uint8_t mpu_scale_gyro(uint8_t scale){
	uint8_t reg_val;
	uint8_t ret;
	
	ret = mpu_read_addr(GYRO_CONFIG, &reg_val);
	if(ret)	return ret;
	
	reg_val = (reg_val & ~(3<<GYRO_FS_SEL_PS)) | (scale<<GYRO_FS_SEL_PS);
	
	return mpu_write_addr(GYRO_CONFIG, reg_val);
}