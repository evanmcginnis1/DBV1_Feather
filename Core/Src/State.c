/*
 * State.c
 *
 * Created on: August 19, 2026
 * Author: Evan McGinnis
 * 
 */

#include "State.h"
#include <stdint.h>
#include <stdbool.h>
void update_state(const uint16_t* ibus_data, const IMU_Model_t* imu_data, bool* disarm_locked, Quadcopter_State_t* current_state) {
    switch (*current_state) {
        case ARMED:
            if (ibus_data[4] == 1000) {
                *current_state = ARMED;
                break;
            } else {
                // if transmitter loses power, reciever failsafe state is to set its fifth channel to 2000
                *current_state = SOFT_DISARM;
                break;
            }
        case HARD_DISARM: 
            // arm switch must be in DISARMED position (=2000) in order to exit HARD_DISARM mode
            if (*disarm_locked || ibus_data[4] != 2000) {
                break;
            } else {
                *current_state = SOFT_DISARM;
                break;
            }
        case SOFT_DISARM:
            if (ibus_data[5] == 1000) {
                *current_state = HARD_DISARM;
                // main loop responsible for unlocking from hard_disarm
                *disarm_locked = true;
                break;
            } else if (ibus_data[4] == 1000) {
                *current_state = ARMED;
                break;
            } else {
                // stay in soft disarm mode
                break;
            }
        default: 
            *current_state = SOFT_DISARM;
    }
}