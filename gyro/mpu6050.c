
#include "mpu6050.h"
#include <avr/io.h>
#include "twi.h"
#include "usart.h"

uint8_t mpu_init(){
    twi_init();
    mpu_write_addr(0x6b,0x00);  //On Powerup MPU is in Sleepmode (Bit6=1)
    mpu_write_addr(0x6a,0x00);  //Disable Fifo and spi and I2CMaster
    mpu_signal_path_reset(0x07);
    
    return 0;
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
    return mpu_burst_read_addr(0x3b, (uint8_t*)(&mpu_acc_val), 6);
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