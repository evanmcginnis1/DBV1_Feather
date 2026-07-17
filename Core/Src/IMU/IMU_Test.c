/*
 * IMU_hardware_Test.c
 *
 *  Created on: May 13, 2026
 *  Author: Evan McGinnis
 */

#include <IMU_Hardware.h>
#include <main.h>
#include <string.h>

// function declarations
void UART_print(const char* msg);
void print_HAL_status(HAL_StatusTypeDef* status);
void print_line_break(void);
void run_IMU_tests(void);
void test_read_IMU_register(void); // done
void test_set_IMU_opr_mode(void);  // redundant - included in set_IMU_config
void test_update_IMU_config(void);   
void test_set_IMU_page(void);      
void test_get_IMU_euler_data(void); 
void test_get_IMU_quat_data(void);
void test_get_IMU_accel_rawdata(void);  


// function definitions
//UART print needs to be setup for STM32F405. 
void UART_print(const char* msg) {
    //HAL_UART_Transmit(&hcom_uart[COM1], (uint8_t*)msg, strlen(msg), 100);
    ;
}
void run_IMU_tests(void) {
    test_read_IMU_register();
    print_line_break();
    test_set_IMU_page();
    print_line_break();
    test_set_IMU_opr_mode();
    print_line_break();
    test_update_IMU_config();
    print_line_break();
    //test_get_IMU_euler_data();
    //print_line_break();
    HAL_Delay(2000);
    //while(1);
}

void print_HAL_status(HAL_StatusTypeDef* status) {
    if (*status == HAL_OK) {
        UART_print("HAL_OK\n");
    } else if (*status == HAL_ERROR) {
        UART_print("HAL_ERROR\n");
    } else if (*status == HAL_BUSY) {
        UART_print("HAL_BUSY\n");
    } else if (*status == HAL_TIMEOUT) {
        UART_print("HAL_TIMEOUT\n");
    } else {
        UART_print("Unknown error\n");
    }
}

void print_line_break(void) {
    UART_print("\n----------------------------------\n\n");
}

// try reading chip ID register; check for read success & verify id is correct
void test_read_IMU_register(void) {
    HAL_StatusTypeDef status;
    uint8_t id;

    status = read_IMU_register(IMU_REG_CHIP_ID, &id, 1);
    UART_print("IMU register read chip id I2C status: ");
    print_HAL_status(&status);
    if (status != HAL_OK) {
        return;
    }

    if (id == 0xA0) {
        UART_print("Successfully read & verified chip ID\n\n");
    } else {
        UART_print("Failed reading chip ID register: incorrect ID\n\n");
    }
}

void test_set_IMU_page(void) {
    uint8_t current_page;
    UART_print("set_IMU_page function: \n");
    set_IMU_page(IMU_PAGE_0);

    read_IMU_register(IMU_REG_PAGE_ID, &current_page, 1);
    if (current_page != IMU_PAGE_0) {
        UART_print("Error: page not set to page 0 \n");
    } else {
        UART_print("Successfully set page 0\n");
    }

    //test setting page 1
    set_IMU_page(IMU_PAGE_1); 
    read_IMU_register(IMU_REG_PAGE_ID, &current_page, 1);
    if (current_page != IMU_PAGE_1) {
        UART_print("Error: failed to set page 1 \n\n");
    } else {
        UART_print("Successfully set page 1\n\n");
    }   
}


void test_set_IMU_opr_mode(void) {
    uint8_t reg_val;
    HAL_StatusTypeDef status;
    status = set_IMU_operation_mode(IMU_OPR_MODE_IMU);
    if (status != HAL_OK) {
        UART_print("i2c set operation mode failed \n");
        while(1);
    }

    status = read_IMU_register(IMU_REG_OPR_MODE, &reg_val, 1);
    if (status != HAL_OK) {
        UART_print("Failed to read IMU_opr_mode register \n");
        while(1);
    }
    //extract lowest 4 bits
    reg_val &= 0x0F;
    if (reg_val == 0x08) {
        UART_print("set_IMU_opr_mode(IMU) success!");
    } else {
        UART_print("set_IMU_opr_mode(IMU) fail!");
    }
}

void test_update_IMU_config() {
    IMU_Config_t test_config = {
        .imu_opr_mode = IMU_OPR_MODE_AMG,
        .accel = {
            .range = IMU_ACCEL_RANGE_8G,
            .bandwidth = IMU_ACCEL_BANDWIDTH_16HZ,
            .opr_mode = IMU_ACCEL_OPR_MODE_SUSPEND
        },
        .gyro = {
            .range = IMU_GYRO_RANGE_1000DPS,
            .bandwidth = IMU_GYRO_BANDWIDTH_47HZ,
            .opr_mode = IMU_GYRO_OPR_MODE_SUSPEND
        },
        .mag = {
            .data_rate = IMU_MAG_RATE_6HZ,
            .opr_mode =  IMU_MAG_OPR_MODE_ENHANCED_REGULAR,
            .pwr_mode =  IMU_MAG_PWR_MODE_SUSPEND
        }
    };
    UART_print("Testing update_IMU_config() function:\n");
    // updates sensor registers
    update_IMU_config(&test_config);
    uint8_t imu_config_reg;
    //imu_config_regs[0] = IMU_OPR_MODE
    //imu_config_regs[1] = IMU_PWR_MODE
    read_IMU_register(IMU_REG_OPR_MODE, &imu_config_reg, 1);
    if ((imu_config_reg & 0x0F) == 0x07) {
        UART_print("Update IMU opr_mode successful\n");
    } else {
        UART_print("Failed to update IMU operation mode\n");
    }
    //switch to page 1 to read individual sensor configurations
    set_IMU_page(IMU_PAGE_1);
    uint8_t sensor_config_regs[4];
    //sensor_config_regs[0] = ACC_CONFIG
    //sensor_config_regs[1] = MAG_CONFIG
    //sensor_config_regs[2] = GYR_CONFIG_0
    //sensor_config_regs[3] = GYR_CONFIG_1
    read_IMU_register(IMU_REG_ACC_CONFIG, sensor_config_regs, 4);
    // check if accel reg configured properly
    if (sensor_config_regs[0] == 0b00100110) {
        UART_print("Update accel config success\n");
    } else {
        UART_print("update accel config fail\n");
    }

    // MSB is not written, so need to mask it out
    if ((sensor_config_regs[1] & IMU_MAG_CONFIG_MASK) == 0b01010001) {
        UART_print("Update mag config success\n");
    } else {
        UART_print("update mag config fail\n");
    }
    // highest two bits of gyr_config reg 0 ignored, highest 5 bits of gyro_config reg 1 ignored
    if (((sensor_config_regs[2] & IMU_GYR_CONFIG_0_MASK) == 0b00011001) && 
        ((sensor_config_regs[3] & IMU_GYR_CONFIG_1_MASK) == 0b00000011)) {
        UART_print("Update gyro config success\n");
    } else {
        UART_print("update gyro config fail\n");
    }
    set_IMU_page(IMU_PAGE_0);

}
// prints 5 datapoints of euler angle data
void test_get_IMU_euler_data(void) {
    HAL_StatusTypeDef transaction_status;
    float EUL_heading, EUL_roll, EUL_pitch;
    char data[55];
    UART_print("Testing get_IMU_euler_data():\n");
    for (int i = 0; i < 10; i++) {
        transaction_status = get_IMU_euler_data(&EUL_heading, &EUL_roll, &EUL_pitch);
        if (transaction_status != HAL_OK) {
            print_HAL_status(&transaction_status);
            return;
        }
        snprintf(data, sizeof(data), "Heading: %f, Roll: %f, Pitch: %f\n", EUL_heading, EUL_pitch, EUL_roll);
    }
}

void test_get_IMU_quat_data() {
    HAL_StatusTypeDef transaction_status;
    float quat_w, quat_x, quat_y, quat_z;
    char msg[80];
    UART_print("Testing get_IMU_quat_data():\n");
    for(int i = 0; i < 10; i++) {
        transaction_status = get_IMU_quat_data(&quat_w, &quat_x, &quat_y, &quat_z);
        if ((quat_w > 1 || quat_w < -1) || 
            (quat_x > 1 || quat_x < -1) ||
            (quat_y > 1 || quat_y < -1) ||
            (quat_z > 1 || quat_z < -1)) 
        {
            snprintf(msg,sizeof(msg),"Invalid quaternion data recieved on attempt %i", i);
            UART_print(msg);
        }
        if (transaction_status != HAL_OK) {
            print_HAL_status(&transaction_status);
            return;
        }
        snprintf(msg, sizeof(msg), "W: %f X: %f, Y: %f, Z: %f", quat_w, quat_x, quat_y, quat_z);
    }
}