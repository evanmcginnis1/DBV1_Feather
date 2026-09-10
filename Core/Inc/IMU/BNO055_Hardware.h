/*
 * IMU_Hardware.h
 *
 *  Created on: May 13, 2026
 *      Author: Evan McGinnis
 *      Low-level code to directly modify registers on BNO055 IMU. Doesn't use static functions to enable easier testing
 */

#ifndef IMU_HARDWARE_H_
#define IMU_HARDWARE_H_

#include <main.h>
#include <stdio.h>
#include <stdbool.h>


//constants
#define SINGLE_WRITE_DATA_SIZE    1
#define IMU_REGISTER_ADDR_SIZE    1
#define IMU_I2C_TIMEOUT           1000

//IMU config registers
#define IMU_REG_CHIP_ID           0x00

//addr is 0x28 if COM3 pulled low
// if COM3 pulled high, addr is 0x29
#define IMU_I2C_ADDR          0x28
//why shift?
#define IMU_I2C_ADDR_SHIFTED      ((IMU_I2C_ADDR) << 1)

/*************************************************
 *           IMU register page select            *
**************************************************/
#define IMU_REG_PAGE_ID           0x07
typedef enum {
    IMU_PAGE_0 = 0x00,
    IMU_PAGE_1 = 0x01,
} IMU_Page_Sel_t;
/*************************************************
 *                Operation Mode                 *
**************************************************/
#define IMU_REG_OPR_MODE          0x3D
#define IMU_OPR_MODE_MASK         0x0F

/*************************************************
 *                Unit Selection                 *
**************************************************/
#define IMU_REG_UNIT_SEL          0x3B
#define IMU_UNIT_SEL_MASK         0x17

// Bit flags for units
// accel units
#define IMU_UNIT_SEL_ACCEL_MS2    0
#define IMU_UNIT_SEL_ACCEL_MG     1
// angular rate units
#define IMU_UNIT_SEL_ANGULAR_RATE_DPS 0
#define IMU_UNIT_SEL_ANGULAR_RATE_RPS (1 << 1)
// Euler angle units
#define IMU_UNIT_SEL_EULER_ANGLE_DEG  0
#define IMU_UNIT_SEL_EULER_ANGLE_RAD  (1 << 2)
// Temperature units
#define IMU_UNIT_SEL_TEMP_C           0
#define IMU_UNIT_SEL_TEMP_F           (1 << 4)

/*************************************************
*       Individual Sensor Config Registers       *
**************************************************/
//accelerometer range config
//all bits written to configure accel
#define IMU_REG_ACC_CONFIG      0x08

#define IMU_REG_MAG_CONFIG      0x09
// preserve highest bit
#define IMU_MAG_CONFIG_MASK     0x7F

#define IMU_REG_GYR_CONFIG_0    0x0A 
//preserve upper two bits
#define IMU_GYR_CONFIG_0_MASK   0x3F

#define IMU_REG_GYR_CONFIG_1    0x0B
#define IMU_GYR_CONFIG_1_MASK   0x07

#define IMU_REG_CALIB_STAT      0x35

#define IMU_REG_AXIS_MAP_CONFIG 0x41
#define IMU_REG_AXIS_MAP_SIGN   0x42

//page 0
#define IMU_REG_SYS_TRIGGER 0x3F
#define IMU_CLK_SEL_EN 0x80

//page 0
#define IMU_REG_AXIS_MAP_CONFIG 0x41
#define IMU_REG_AXIS_MAP_SIGN 0x42

#define IMU_REMAP_X_SHIFT 0
#define IMU_REMAP_Y_SHIFT 2
#define IMU_REMAP_Z_SHIFT 4

#define NEW_X_AXIS_MASK (0x03 << IMU_REMAP_X_SHIFT)
#define NEW_Y_AXIS_MASK (0x03 << IMU_REMAP_Y_SHIFT)
#define NEW_Z_AXIS_MASK (0x03 << IMU_REMAP_Z_SHIFT)

#define IMU_AXIS_X_SIGN_SHIFT 2
#define IMU_AXIS_Y_SIGN_SHIFT 1
#define IMU_AXIS_Z_SIGN_SHIFT 0

#define IMU_AXIS_X_SIGN_MASK (0x01 << IMU_AXIS_X_SIGN_SHIFT)
#define IMU_AXIS_Y_SIGN_MASK (0x01 << IMU_AXIS_Y_SIGN_SHIFT)
#define IMU_AXIS_Z_SIGN_MASK (0x01 << IMU_AXIS_Z_SIGN_SHIFT)



/*************************************************
*             IMU Data Registers              *
**************************************************/

//raw gyro data register
#define IMU_REG_GYR_DATA_X_LSB 0x14
#define IMU_REG_GYR_DATA_X_MSB 0x15

#define IMU_REG_GYR_DATA_Y_LSB 0x16
#define IMY_REG_GYR_DATA_Y_MSB 0x17

#define IMU_REG_GYR_DATA_Z_LSB 0x18
#define IMU_REG_GYR_DATA_Z_MSB 0x19

//FUSED
// Fused Euler orientation data registers
#define IMU_REG_EUL_DATA_X_LSB 0x1A
#define IMU_REG_EUL_DATA_X_MSB 0x1B

#define IMU_REG_EUL_DATA_Y_LSB 0x1C
#define IMU_REG_EUL_DATA_Y_MSB 0x1D

#define IMU_REG_EUL_DATA_Z_LSB 0x1E
#define IMU_REG_EUL_DATA_Z_MSB 0x1F

//Fused Quaternion orientation registers
#define IMU_REG_QUA_DATA_W_LSB 0x20
#define IMU_REG_QUA_DATA_W_MSB 0x21

#define IMU_REG_QUA_DATA_X_LSB 0x22
#define IMU_REG_QUA_DATA_X_MSB 0x23

#define IMU_REG_QUA_DATA_Y_LSB 0x24
#define IMU_REG_QUA_DATA_Y_MSB 0x25

#define IMU_REG_QUA_DATA_Z_LSB 0x26
#define IMU_REG_QUA_DATA_Z_MSB 0x27



//Linear acceleration data registers
#define IMU_REG_LIA_DATA_X_LSB 0x28
#define IMU_REG_LIA_DATA_X_MSB 0x29

#define IMU_REG_LIA_DATA_Y_LSB 0x2A
#define IMU_REG_LIA_DATA_Y_MSB 0x2B

#define IMU_REG_LIA_DATA_Z_LSB 0x2C
#define IMU_REG_LIA_DATA_Z_MSB 0x2D



//gravity vector data
#define IMU_REG_GRV_DATA_X_LSB 0x2E
#define IMU_REG_GRV_DATA_X_MSB 0x2F

#define IMU_REG_GRV_DATA_Y_LSB 0x30
#define IMU_REG_GRV_DATA_Y_MSB 0x31

#define IMU_REG_GRV_DATA_Z_LSB 0x32
#define IMU_REG_GRV_DATA_Z_MSB 0x33

//UN_FUSED
#define IMU_ACC_DATA_X_LSB     0X08
#define IMU_ACC_DATA_X_MSB     0X09

#define IMU_ACC_DATA_Y_LSB     0X0A
#define IMU_ACC_DATA_Y_MSB     0X0B

#define IMU_ACC_DATA_Z_LSB     0X0C
#define IMU_ACC_DATA_Z_MSB     0X0D

/*************************************************
*             Sensor Configuration Options       *
**************************************************/

typedef enum {
    IMU_AXIS_X = 0b00, 
    IMU_AXIS_Y = 0b01,
    IMU_AXIS_Z = 0b10,
} IMU_Axis_t;

typedef enum {
    IMU_AXIS_SIGN_POSITIVE = 0,
    IMU_AXIS_SIGN_NEGATIVE = 1,
} IMU_Axis_Sign_t;
/*
    IMU_OPR_MODE_CONFIGMODE = 0x00,
    IMU_OPR_MODE_ACCONLY = 0x01,
    IMU_OPR_MODE_MAGONLY = 0x02,
    IMU_OPR_MODE_GYROONLY = 0x03,
    IMU_OPR_MODE_ACCMAG = 0x04,
    IMU_OPR_MODE_ACCGYRO = 0x05,
    IMU_OPR_MODE_MAGGYRO = 0x06,
    IMU_OPR_MODE_AMG = 0x07,
    IMU_OPR_MODE_IMU = 0x08,
    IMU_OPR_MODE_COMPASS = 0x09,
    IMU_OPR_MODE_M4G = 0x0A,
    IMU_OPR_MODE_NDOF_FMC_OFF = 0x0B,
    IMU_OPR_MODE_NDOF = 0x0C
*/
typedef enum {
    IMU_OPR_MODE_CONFIGMODE = 0x00,
//non-fusion modes
    IMU_OPR_MODE_ACCONLY = 0x01,
    IMU_OPR_MODE_MAGONLY = 0x02,
    IMU_OPR_MODE_GYROONLY = 0x03,
    IMU_OPR_MODE_ACCMAG = 0x04,
    IMU_OPR_MODE_ACCGYRO = 0x05,
    IMU_OPR_MODE_MAGGYRO = 0x06,
    IMU_OPR_MODE_AMG = 0x07,
//fusion modes
    IMU_OPR_MODE_IMU = 0x08,
    IMU_OPR_MODE_COMPASS = 0x09,
    IMU_OPR_MODE_M4G = 0x0A,
    IMU_OPR_MODE_NDOF_FMC_OFF = 0x0B,
    IMU_OPR_MODE_NDOF = 0x0C
} IMU_OprMode_t;

/*
    ACCEL_RANGE_2G = 0x00u,
    ACCEL_RANGE_4G = 0x01u,
    ACCEL_RANGE_8G = 0x02u,
    ACCEL_RANGE_16G = 0x03u
*/
typedef enum {
    IMU_ACCEL_RANGE_2G = 0x00u,
    IMU_ACCEL_RANGE_4G = 0x01u,
    IMU_ACCEL_RANGE_8G = 0x02u,
    IMU_ACCEL_RANGE_16G = 0x03u
} IMU_AccelRange_t;

/*
    IMU_ACCEL_BANDWIDTH_8HZ =    0x00u,
    IMU_ACCEL_BANDWIDTH_16HZ =   0x01u,
    IMU_ACCEL_BANDWIDTH_31HZ =   0x02u,
    IMU_ACCEL_BANDWIDTH_63HZ =   0x03u,
    IMU_ACCEL_BANDWIDTH_125HZ =  0x04u,
    IMU_ACCEL_BANDWIDTH_250HZ =  0x05u,
    IMU_ACCEL_BANDWIDTH_500HZ =  0x06u,
    IMU_ACCEL_BANDWIDTH_1000HZ = 0x07u
*/
typedef enum {
    //Actual: 7.81 Hz
    IMU_ACCEL_BANDWIDTH_8HZ =    0x00u,
    //Actual: 15.63 Hz
    IMU_ACCEL_BANDWIDTH_16HZ =   0x01u,
    //Actual: 31.25 Hz
    IMU_ACCEL_BANDWIDTH_31HZ =   0x02u,
    //Actual: 62.5 Hz
    IMU_ACCEL_BANDWIDTH_63HZ =   0x03u,
    IMU_ACCEL_BANDWIDTH_125HZ =  0x04u,
    IMU_ACCEL_BANDWIDTH_250HZ =  0x05u,
    IMU_ACCEL_BANDWIDTH_500HZ =  0x06u,
    IMU_ACCEL_BANDWIDTH_1000HZ = 0x07u
} IMU_AccelBandwidth_t;
/*
    IMU_ACCEL_OPR_MODE_NORMAL =       0x00u,
    IMU_ACCEL_OPR_MODE_SUSPEND =      0x01u,
    IMU_ACCEL_OPR_MODE_LOW_1 =        0x02u,
    IMU_ACCEL_OPR_MODE_STANDBY =      0x03u,
    IMU_ACCEL_OPR_MODE_LOW_2 =        0x04u,
    IMU_ACCEL_OPR_MODE_DEEP_SUSPEND = 0x05u
*/
typedef enum {
    IMU_ACCEL_OPR_MODE_NORMAL =       0x00u,
    IMU_ACCEL_OPR_MODE_SUSPEND =      0x01u,
    IMU_ACCEL_OPR_MODE_LOW_1 =        0x02u,
    IMU_ACCEL_OPR_MODE_STANDBY =      0x03u,
    IMU_ACCEL_OPR_MODE_LOW_2 =        0x04u,
    IMU_ACCEL_OPR_MODE_DEEP_SUSPEND = 0x05u
} IMU_AccelOprMode_t;

/*
    IMU_GYRO_RANGE_2000DPS = 0x00u,
    IMU_GYRO_RANGE_1000DPS = 0x01u,
    IMU_GYRO_RANGE_500DPS =  0x02u,
    IMU_GYRO_RANGE_250DPS =  0x03u,
    IMU_GYRO_RANGE_125DPS =  0x04u,
*/
typedef enum {
    IMU_GYRO_RANGE_2000DPS = 0x00u,
    IMU_GYRO_RANGE_1000DPS = 0x01u,
    IMU_GYRO_RANGE_500DPS =  0x02u,
    IMU_GYRO_RANGE_250DPS =  0x03u,
    IMU_GYRO_RANGE_125DPS =  0x04u,
} IMU_GyroRange_t;

/*
    IMU_GYRO_BANDWIDTH_523HZ = 0x00u,
    IMU_GYRO_BANDWIDTH_230HZ = 0x01u,
    IMU_GYRO_BANDWIDTH_116HZ = 0x02u,
    IMU_GYRO_BANDWIDTH_47HZ =  0x03u,
    IMU_GYRO_BANDWIDTH_23HZ =  0x04u,
    IMU_GYRO_BANDWIDTH_12HZ =  0x05u,
    IMU_GYRO_BANDWIDTH_64HZ =  0x06u,
    IMU_GYRO_BANDWIDTH_32HZ =  0x07u
*/
typedef enum {
    IMU_GYRO_BANDWIDTH_523HZ = 0x00u,
    IMU_GYRO_BANDWIDTH_230HZ = 0x01u,
    IMU_GYRO_BANDWIDTH_116HZ = 0x02u,
    IMU_GYRO_BANDWIDTH_47HZ =  0x03u,
    IMU_GYRO_BANDWIDTH_23HZ =  0x04u,
    IMU_GYRO_BANDWIDTH_12HZ =  0x05u,
    IMU_GYRO_BANDWIDTH_64HZ =  0x06u,
    IMU_GYRO_BANDWIDTH_32HZ =  0x07u
} IMU_GyroBandwidth_t;

/*
    IMU_GYRO_OPR_MODE_NORMAL =       0x00u,
    IMU_GYRO_OPR_MODE_FAST_PWR_UP =  0x01u,
    IMU_GYRO_OPR_MODE_DEEP_SUSPEND = 0x02u,
    IMU_GYRO_OPR_MODE_SUSPEND =      0x03u,
    IMU_GYRO_OPR_MODE_ADV_POW_SAVE = 0x04u,
*/
typedef enum {
    IMU_GYRO_OPR_MODE_NORMAL =       0x00u,
    IMU_GYRO_OPR_MODE_FAST_PWR_UP =  0x01u,
    IMU_GYRO_OPR_MODE_DEEP_SUSPEND = 0x02u,
    IMU_GYRO_OPR_MODE_SUSPEND =      0x03u,
    IMU_GYRO_OPR_MODE_ADV_POW_SAVE = 0x04u,
} IMU_GyroOprMode_t;

/*
    IMU_MAG_RATE_2HZ =  0x00u,
    IMU_MAG_RATE_6HZ =  0x01u,
    IMU_MAG_RATE_8HZ =  0x02u,
    IMU_MAG_RATE_10HZ = 0x03u,
    IMU_MAG_RATE_15HZ = 0x04u,
    IMU_MAG_RATE_20HZ = 0x05u,
    IMU_MAG_RATE_25HZ = 0x06u,
    IMU_MAG_RATE_30HZ = 0x07u
*/
typedef enum {
    IMU_MAG_RATE_2HZ =  0x00u,
    IMU_MAG_RATE_6HZ =  0x01u,
    IMU_MAG_RATE_8HZ =  0x02u,
    IMU_MAG_RATE_10HZ = 0x03u,
    IMU_MAG_RATE_15HZ = 0x04u,
    IMU_MAG_RATE_20HZ = 0x05u,
    IMU_MAG_RATE_25HZ = 0x06u,
    IMU_MAG_RATE_30HZ = 0x07u
} IMU_MagDataRate_t;

/* 
    IMU_MAG_OPR_MODE_LOW_PWR =          0x00u,
    IMU_MAG_OPR_MODE_REGULAR =          0x01u,
    IMU_MAG_OPR_MODE_ENHANCED_REGULAR = 0x02u,
    IMU_MAG_OPR_MODE_HIGH_ACCURACY =    0x03u  
*/
typedef enum {
    IMU_MAG_OPR_MODE_LOW_PWR =          0x00u,
    IMU_MAG_OPR_MODE_REGULAR =          0x01u,
    IMU_MAG_OPR_MODE_ENHANCED_REGULAR = 0x02u,
    IMU_MAG_OPR_MODE_HIGH_ACCURACY =    0x03u
} IMU_MagOprMode_t;

/* 
* IMU_MAG_PWR_MODE_NORMAL,
* IMU_MAG_PWR_MODE_SLEEP,
* IMU_MAG_PWR_MODE_SUSPEND,
* IMU_MAG_PWR_MODE_FORCE_MODE,
*/
typedef enum {
    IMU_MAG_PWR_MODE_NORMAL =     0x00u,
    IMU_MAG_PWR_MODE_SLEEP =      0x01u,
    IMU_MAG_PWR_MODE_SUSPEND =    0x02u,
    IMU_MAG_PWR_MODE_FORCE_MODE = 0x03u
} IMU_MagPwrMode_t;

typedef struct {
    IMU_AccelRange_t range;
    IMU_AccelBandwidth_t bandwidth;
    IMU_AccelOprMode_t opr_mode;
} IMU_AccelConfig_t;

typedef struct {
    IMU_GyroRange_t range;
    IMU_GyroBandwidth_t bandwidth;
    IMU_GyroOprMode_t opr_mode;
} IMU_GyroConfig_t;

typedef struct {
    IMU_MagDataRate_t data_rate;
    IMU_MagOprMode_t opr_mode;
    IMU_MagPwrMode_t pwr_mode;
} IMU_MagConfig_t;

//for non-fusion modes -- fusion mode sets these automatically
typedef struct {
    IMU_OprMode_t imu_opr_mode;
    IMU_AccelConfig_t accel;
    IMU_GyroConfig_t gyro;
    IMU_MagConfig_t mag;
} IMU_Config_t;

typedef struct{
    bool sys_good;
    bool accel_good;
    bool gyro_good;
    bool mag_good;
} IMU_Calibration_status_t;

//wrapper around sub-structs 
//define once per imu, in main.c
typedef struct {
    I2C_HandleTypeDef* hi2c;
    IMU_Config_t config;
    IMU_Calibration_status_t cal_status;
} IMU_t;

/*************************************************
*           IMU I2C Interface                    *
**************************************************/

/*
* Requires: hi2c object is I2C_HandleTypeDef object
* Modifies: imu object hi2c member
* Effects: Returns nothing. 
*/
void IMU_configure_i2c(I2C_HandleTypeDef* hi2c);

/*
 * Requires: register_addr is the address of the sensor register to be read
 * 			 rx_buffer is the buffer to store the data recieved from the sensor
 * 			 num_bytes_to_read is an unsigned integer greater than zero
 * Modifies: rx_buffer
 * Effects: returns status of transaction
 */
HAL_StatusTypeDef IMU_read_register(uint8_t register_addr, uint8_t* rx_buffer, 
                                    uint8_t num_bytes_to_read);

/* Requires: register_addr is a valid register address on IMU, tx_buffer is 1 byte of data to send
 * Modifies: One IMU register byte
 * Effects: returns i2c transaction status. 
 */
HAL_StatusTypeDef IMU_write_register(uint8_t register_addr, uint8_t* tx_buffer);

/*
* Requires: page_num is either 1 or 0
* Modifies: Page_ID register on IMU
* Effects: sets page ID register on IMU
*/ 
HAL_StatusTypeDef IMU_set_page(IMU_Page_Sel_t page_num);

/*************************************************
*           IMU Overall Configuration            *
**************************************************/

/*
* Requires: nothing
* Modifies: 
* Effects: updates config registers on sensor for accelerometer, magnetometer, gyroscope.
*          
*/
HAL_StatusTypeDef IMU_default_config(void);

/*
* Requires: 
* Modifies: IMU PAGE_ID, 
* Effects: If a fusion mode is selected, sets IMU OPR_MODE register to that mode, and skips individual sensor config.
           Otherwise, writes to all config registers of IMU (besides IMU PWR_MODE)
*/
HAL_StatusTypeDef IMU_send_config_to_sensor(void);

//function to set units - units are chosen by #define in IMU_conductor.h
/*
* Requires: Requires: IMU registers pre-set to page 0. Units are selected before compile in IMU_config.h
* Modifies: IMU UNIT_SEL register
* Effects:  Sets units for acceleration, rotation, rotation rate, and magnetic field strength
*/
HAL_StatusTypeDef IMU_set_units(void);

//functions for overall sensor config
/*
* Requires: IMU registers pre-set to page 0. operation_mode is a valid mode
* Modifies: IMU OPR_MODE register
* Effects:  Sets operation mode on IMU
*/
HAL_StatusTypeDef IMU_set_operation_mode(IMU_OprMode_t operation_mode);

/*************************************************
*           Individual Sensor Config             *
**************************************************/

/*
* Requires: IMU registers pre-set to page 1
* Modifies:
* Effects:
*/
HAL_StatusTypeDef IMU_set_accel_config(IMU_AccelConfig_t* accel_config);

/*
* Requires: IMU registers pre-set to page 1
* Modifies:
* Effects:
*/
HAL_StatusTypeDef IMU_set_gyro_config(IMU_GyroConfig_t* gyro_config);

/*
* Requires: IMU registers pre-set to page 1
* Modifies: 
* Effects:
*/
HAL_StatusTypeDef IMU_set_mag_config(IMU_MagConfig_t* mag_config);

HAL_StatusTypeDef IMU_remap_axes(IMU_Axis_t remap_x_value, IMU_Axis_t remap_y_value, IMU_Axis_t remap_z_value);
HAL_StatusTypeDef IMU_change_axis_signs(IMU_Axis_Sign_t x_sign, IMU_Axis_Sign_t y_sign, IMU_Axis_Sign_t z_sign);
/*************************************************
*              Sensor Calibration                *
**************************************************/
HAL_StatusTypeDef IMU_calibrate_sensors(void);
HAL_StatusTypeDef IMU_check_calibration_status(void);
HAL_StatusTypeDef IMU_set_calibration_profile(void);

/*************************************************
*           Sensor Data Functions                *
**************************************************/

/*
* Requires: QUAT_w, QUAT_x, QUAT_y, QUAT_z are pointers to floats. 
* Modifies: QUAT_w, QUAT_x, QUAT_y, QUAT_z
* Effects: Reads IMU fused quaterion data
*/
HAL_StatusTypeDef IMU_get_quat_data(float* QUAT_w, float* QUAT_x, float* QUAT_y, float* QUAT_z);

/*
* Requires: 
* Modifies:
* Effects:
*/
HAL_StatusTypeDef IMU_get_euler_data(float* EUL_heading, float* EUL_roll, float* EUL_pitch);

/*
* Requires: 
* Modifies:
* Effects:
*/
HAL_StatusTypeDef IMU_get_accel_rawdata(float* accel_x, float* accel_y, float* accel_z);

/*
* Requires: 
* Modifies:
* Effects:
*/
HAL_StatusTypeDef IMU_get_grav_data(float* grav_x, float* grav_y, float* grav_z);

/*
* Requires: 
* Modifies:
* Effects:
*/
HAL_StatusTypeDef IMU_get_lin_accel_data(float* linaccel_x, float* linaccel_y, float* linaccel_z);

HAL_StatusTypeDef IMU_get_gyro_rawdata(float* pitch, float* roll, float* yaw);
/*
* Requires: 
* Modifies:
* Effects:
*/
HAL_StatusTypeDef IMU_get_chipID(uint8_t* chipID);

/*
 * Requires: External oscillator is connected to IMU. Ensure IMU is already in configuration mode
 * Modifies: IMU_SYS_TRIGGER register
 * Effects: Sets IMU to use an external oscillator for its clock
 */
HAL_StatusTypeDef IMU_enable_external_oscillator(void);

#endif /* SRC_IMU_HARDWARE_H_ */
