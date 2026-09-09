/*
 * State.h
 *
 * Created on: August 14, 2026
 * Author: Evan McGinnis
 * 
 * Contains state enum for quadcopter
 */
#ifndef STATE_H
#define STATE_H

#include "IMU_Model.h"
#include <stdbool.h>
#include <stdint.h>


typedef enum {
    ARMED,
    SOFT_DISARM,
    HARD_DISARM,
} Quadcopter_State_t;


/*
 * Requires: All parameters have been initialized, ibus_data, imu_data have been updated to the current loop iteration
 * Modifies: current_state variable
 * Effects: 
*/
void update_state(const uint16_t* ibus_data, const IMU_Model_t* imu_data, bool* disarm_locked, Quadcopter_State_t* current_state);

#endif