#ifndef IMU_SENSOR_READ
#define IMU_SENSOR_READ



#include "main.h"
#include <stdbool.h>

//// Sensor configuration
// conversion constants 
#define TEMP_C 0.00390625
#define ACCEL_G 0.488
#define ACCEL_MS2 9.80665
#define GYRO_DPS 0.035
#define GYRO_RADS 0.0174533
#define HG_ACCEL_G 0.976

// device registries
#define ADDR_READ 0b11010101
#define ADDR_WRITE 0b11010100

// registries for enable writing
#define GLOBAL_REG 0x01
#define CTRL_CONF_REG 0x12
#define GYRO_CTRL_REG 0x11
#define GYRO_DPS_REG 0x15
#define GYRO_FILTER_REG 0x16
#define ACCEL_CTRL_REG 0x10
#define ACCEL_FS_REG 0x17
#define HG_ACCEL_OFFSET_REG 0x18
//#define ACCEL_FILTER_REG 0x16
#define HG_ACCEL_CTRL_REG 0x4E

// values for enable writing
#define GLOBAL_RESET 0b00000100
#define CONF_RESET 0b00000001
#define GRYO_240HZ 0b00000111
#define GYRO_250DPS 0b00111011
#define GYRO_ENABLE_FILTER 0b00000001
#define ACCEL_240HZ 0b00000111
#define ACCEL_16_FS 0b00000011
#define HG_ACCEL_OFFSET 0b00000011
//#define ACCEL_ENABLE_FILTER 0b00000000
#define HG_ACCEL_19kHZ_32 0b11011000
#define I2C_BDU_INC 0b01000100

// addresses for gryo values
#define ADDR_GYRO_X1 0x22
#define ADDR_GYRO_X2 0x23
#define ADDR_GYRO_Y1 0x24
#define ADDR_GYRO_Y2 0x25
#define ADDR_GYRO_Z1 0x26
#define ADDR_GYRO_Z2 0x27

// addresses for gryo values
#define ADDR_ACCEL_X1 0x28
#define ADDR_ACCEL_X2 0x29
#define ADDR_ACCEL_Y1 0x2A
#define ADDR_ACCEL_Y2 0x2B
#define ADDR_ACCEL_Z1 0x2C
#define ADDR_ACCEL_Z2 0x2D
#define ADDR_HG_ACCEL_X1 0x34
#define ADDR_HG_ACCEL_X2 0x35
#define ADDR_HG_ACCEL_Y1 0x36
#define ADDR_HG_ACCEL_Y2 0x37
#define ADDR_HG_ACCEL_Z1 0x38
#define ADDR_HG_ACCEL_Z2 0x39

// addresses for temperature sensor values
#define ADDR_TEMP1 0x20
#define ADDR_TEMP2 0x21

//// Interrupt configuration
// registries to enable interrupts
#define FUNCTIONS_ENABLE 0x50
#define INT2_CTRL_REG 0x0E
#define INT1_CTRL_REG 0x0D
#define HG_FUNCTIONS_ENABLE 0x52

// values to enable interrupts
#define INT_ENABLE 0b10000000
#define INT1_GYROACCEL 0b00000011
#define INT2_GYROACCEL 0b00000000
#define HG_INT_ENABLE 0b10000000

// registries to enable free-fall detection on int1 
#define WAKEUP_DUR_REG 0x5C
#define FF_REG 0x5D
#define MD1_CFG_REG 0x5E

// values to enable free-fall detection on int1 
#define WAKEUP_DUR 0b10000000
#define FREEFALL_DUR 0b00001000
#define ROUTE_INT1 0b00010000

// registries to enable orientation detection on int2 
#define ORIENTATION_REG 0x47
#define MD2_CFG_REG 0x5F

// values to enable orientation detection on int2
#define ORIENT_CHECK 0b01111111
#define ROUTE_INT2 0b00000100

#define ALL_INT_SRC 0x1D
#define D6D_SRC 0x47

// 

//// IMU sensor headers

// sets the configuration registries for the IMU. do not alter
void imuSetConfigRegs(I2C_HandleTypeDef *hi2c1);

// functions for each of the gyroscope axes
float readGyroX(I2C_HandleTypeDef *hi2c1);
float readGyroY(I2C_HandleTypeDef *hi2c1);
float readGyroZ(I2C_HandleTypeDef *hi2c1);

// functions for each of the accelerometer axes
float readAccelX(I2C_HandleTypeDef *hi2c1);
float readAccelY(I2C_HandleTypeDef *hi2c1);
float readAccelZ(I2C_HandleTypeDef *hi2c1);

// functions for each of the accelerometer axes
float readHGAccelX(I2C_HandleTypeDef *hi2c1);
float readHGAccelY(I2C_HandleTypeDef *hi2c1);
float readHGAccelZ(I2C_HandleTypeDef *hi2c1);

// function to read the imu temp sensor
float readTempIMU(I2C_HandleTypeDef *hi2c1);

bool hgShockDetect(I2C_HandleTypeDef *hi2c1);

//function that tells you whether you are in freefall
bool freefallDetection(I2C_HandleTypeDef *hi2c1);

//function that tells you whether its changing orientation
bool sixDChangeDetection(I2C_HandleTypeDef *hi2c1);

//functions that read high or low for each axis (x,y,z) depending on orientation
//returns H for high, L for low
int checkOrintation(I2C_HandleTypeDef *hi2c1);

#endif