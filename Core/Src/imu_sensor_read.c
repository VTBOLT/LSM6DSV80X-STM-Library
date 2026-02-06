# if defined STM32G474xx
#include "stm32g4xx_hal.h"
# else
# if defined STM32H523xx
#include "stm32h523xx"
# endif
# endif
#include "imu_sensor_read.h"
#include <stdbool.h>
#include "string.h"

void imuSetConfigRegs(I2C_HandleTypeDef *hi2c1) {
    //// reset device registries. Conf-reset should not be disbaled
    // uint8_t global_reset = GLOBAL_RESET;
    // HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, GLOBAL_REG, 1, &global_reset, 1, 200);
    uint8_t writePointer = CONF_RESET;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, CTRL_CONF_REG, 1, &writePointer, 1, 200);

    // enable gyroscope, accelerometer, and temperature sensors
    writePointer = GRYO_240HZ;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, GYRO_CTRL_REG, 1, &writePointer, 1, 200);
    writePointer = GYRO_250DPS;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, GYRO_DPS_REG, 1, &writePointer, 1, 200);
    writePointer = GYRO_ENABLE_FILTER;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, GYRO_FILTER_REG, 1, &writePointer, 1, 200);
    writePointer = ACCEL_240HZ;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, ACCEL_CTRL_REG, 1, &writePointer, 1, 200);
    writePointer = ACCEL_16_FS;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, ACCEL_FS_REG, 1, &writePointer, 1, 200);
    writePointer = HG_ACCEL_OFFSET;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, HG_ACCEL_OFFSET_REG, 1, &writePointer, 1, 200);
    //writePointer = ACCEL_ENABLE_FILTER;
    //HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, ACCEL_FILTER_REG, 1, &writePointer, 1, 200);
    writePointer = HG_ACCEL_19kHZ_32;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, HG_ACCEL_CTRL_REG, 1, &writePointer, 1, 200);
    writePointer = I2C_BDU_INC;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, CTRL_CONF_REG, 1, &writePointer, 1, 200);

    // enable global interrupts and set allow each to access gryo and accel data. Also enables hg shock 
    writePointer = INT_ENABLE;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, FUNCTIONS_ENABLE, 1, &writePointer, 1, 200);
    writePointer = INT1_GYROACCEL; 
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, INT1_CTRL_REG, 1, &writePointer, 1, 200);  
    writePointer = INT2_GYROACCEL; 
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, INT2_CTRL_REG, 1, &writePointer, 1, 200);  
    writePointer = HG_INT_ENABLE; 
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, HG_FUNCTIONS_ENABLE, 1, &writePointer, 1, 200);  

    // enable freefall detection and send to int1
    writePointer = WAKEUP_DUR;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, WAKEUP_DUR_REG, 1, &writePointer, 1, 200);
    writePointer = FREEFALL_DUR;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, FF_REG, 1, &writePointer, 1, 200);
    writePointer = ROUTE_INT1;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, MD1_CFG_REG, 1, &writePointer, 1, 200);
   
   
    // enable orientation change detection and send data to int2
    writePointer = ORIENT_CHECK;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, ORIENTATION_REG, 1, &writePointer, 1, 200);
    writePointer = ROUTE_INT2;
    HAL_I2C_Mem_Write(hi2c1, ADDR_WRITE, MD2_CFG_REG, 1, &writePointer, 1, 200);


}

float readGyroX(I2C_HandleTypeDef *hi2c1) {
    uint8_t regGyroX1, regGyroX2;

    // Read gyroscope X values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GYRO_X1, 1, &regGyroX1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GYRO_X2, 1, &regGyroX2, 1, 50); 
    int16_t gyroXraw = (int16_t)(regGyroX2 << 8) | regGyroX1;

    // convert to dps
    float gyroX=gyroXraw*GYRO_DPS;
    // convert to rad/s
    gyroX=gyroX*GYRO_RADS;

    return gyroX;
}

float readGyroY(I2C_HandleTypeDef *hi2c1) {
    uint8_t regGyroY1, regGyroY2;

    // Read gyroscope Y values then combine
	HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GYRO_Y1, 1, &regGyroY1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GYRO_Y2, 1, &regGyroY2, 1, 50);
    int16_t gyroYraw = (int16_t)(regGyroY2 << 8) | regGyroY1;

    // convert to dps
    float gyroY=gyroYraw*GYRO_DPS;
    // convert to rad/s
    gyroY=gyroY*GYRO_RADS;

    return gyroY;
}

float readGyroZ(I2C_HandleTypeDef *hi2c1) {
    uint8_t regGyroZ1, regGyroZ2;

    // Read gyroscope Z values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GYRO_Z1, 1, &regGyroZ1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_GYRO_Z2, 1, &regGyroZ2, 1, 50);
    int16_t gyroZraw = (int16_t)(regGyroZ2 << 8) | regGyroZ1;

    // convert to dps 
    float gyroZ=gyroZraw*GYRO_DPS;
    // convert to rad/s
    gyroZ=gyroZ*GYRO_RADS;

    return gyroZ;
}

float readAccelX(I2C_HandleTypeDef *hi2c1) {
    uint8_t regAccelX1, regAccelX2;

    // Read accelerometers X values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_X1, 1, &regAccelX1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_X2, 1, &regAccelX2, 1, 50); 
    int16_t accelXraw = (int16_t)(regAccelX2 << 8) | regAccelX1;

    // convert to mgs
    float accelX=accelXraw*ACCEL_G;
    // convert to m/s^2
    accelX=accelX*ACCEL_MS2;
    

    return accelX;
}

float readAccelY(I2C_HandleTypeDef *hi2c1) {
    uint8_t regAccelY1, regAccelY2;

    // Read accelerometers Y values then combine
	HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_Y1, 1, &regAccelY1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_Y2, 1, &regAccelY2, 1, 50);
    int16_t accelYraw = (int16_t)(regAccelY2 << 8) | regAccelY1;

    // convert to mgs
    float accelY=accelYraw*ACCEL_G;
    // convert to m/s^2
    accelY=accelY*ACCEL_MS2;

    return accelY;
}

float readAccelZ(I2C_HandleTypeDef *hi2c1) {
    uint8_t regAccelZ1, regAccelZ2;

    // Read accelerometers Y values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_Z1, 1, &regAccelZ1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_ACCEL_Z2, 1, &regAccelZ2, 1, 50);
    int16_t accelZraw = (int16_t)(regAccelZ2 << 8) | regAccelZ1;

    // convert to mgs
    float accelZ=accelZraw*ACCEL_G;
    // convert to m/s^2
    accelZ=accelZ*ACCEL_MS2;

    return accelZ;
}

float readHGAccelX(I2C_HandleTypeDef *hi2c1) {
    uint8_t regHGAccelX1, regHGAccelX2;

    // Read accelerometers X values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_HG_ACCEL_X1, 1, &regHGAccelX1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_HG_ACCEL_X2, 1, &regHGAccelX2, 1, 50); 
    int16_t accelHGXraw = (int16_t)(regHGAccelX2 << 8) | regHGAccelX1;

    // convert to mgs
    float accelHGX=accelHGXraw*HG_ACCEL_G;

    return accelHGX;
}

float readHGAccelY(I2C_HandleTypeDef *hi2c1) {
    uint8_t regHGAccelY1, regHGAccelY2;

    // Read accelerometers X values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_HG_ACCEL_Y1, 1, &regHGAccelY1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_HG_ACCEL_Y2, 1, &regHGAccelY2, 1, 50); 
    int16_t accelHGYraw = (int16_t)(regHGAccelY2 << 8) | regHGAccelY1;

    // convert to mgs
    float accelHGY=accelHGYraw*HG_ACCEL_G;

    return accelHGY;
}

float readHGAccelZ(I2C_HandleTypeDef *hi2c1) {
    uint8_t regHGAccelZ1, regHGAccelZ2;

    // Read accelerometers X values then combine
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_HG_ACCEL_Z1, 1, &regHGAccelZ1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_HG_ACCEL_Z2, 1, &regHGAccelZ2, 1, 50); 
    int16_t accelHGZraw = (int16_t)(regHGAccelZ2 << 8) | regHGAccelZ1;

    // convert to mgs
    float accelHGZ=accelHGZraw*HG_ACCEL_G;

    return accelHGZ;
}

float readTempIMU(I2C_HandleTypeDef *hi2c1) {
    uint8_t regTemp1, regTemp2;

    // Read temperature
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_TEMP1, 1, &regTemp1, 1, 50);
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ADDR_TEMP2, 1, &regTemp2, 1, 50);
    int16_t tempRaw = (int16_t)(regTemp2 << 8) | regTemp1;

    // convert to degrees C
    float temp=tempRaw*TEMP_C+25; // this is the coversion factor dont ask

    return temp;
}

bool hgShockDetect(I2C_HandleTypeDef *hi2c1) {
    uint8_t regInt;
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ALL_INT_SRC, 1, &regInt, 1, 50);
    return(regInt & 0b0001000) != 0;
}

bool freefallDetection(I2C_HandleTypeDef *hi2c1){
    uint8_t regFF;
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, ALL_INT_SRC, 1, &regFF, 1, 50);
    return (regFF & 0b00000001) != 0;
}

bool sixDChangeDetection(I2C_HandleTypeDef *hi2c1){
    uint8_t reg6D;
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, D6D_SRC, 1, &reg6D, 1, 50);
    return (reg6D & 0b01000000) != 0;
}
    
int checkOrintation(I2C_HandleTypeDef *hi2c1){
    uint8_t orient;
    HAL_I2C_Mem_Read(hi2c1, ADDR_READ, D6D_SRC, 1, &orient, 1, 50);
    if (orient & 0b00100000) {
        return 6;
    }
    if (orient & 0b00010000) {
        return 5;
    }
    if (orient & 0b00001000) {
        return 4;
    }
    if (orient & 0b00000100) {
        return 3;
    }
    if (orient & 0b00000010) {
        return 2;
    }
    if (orient & 0b00000001) {
        return 1;
    }
    return 0;
}