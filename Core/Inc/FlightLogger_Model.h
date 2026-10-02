/*
 * FlightLogger.h
 *
 * Created on: August 20, 2026
 * Author: Evan McGinnis
 * 
 */

 #include "USB_Handler.h"
 #include "State.h"

 #ifndef FLIGHTLOGGER_H
 #define FLIGHTLOGGER_H

 #define PACKET_VERSION 1
 
 //MUST be of size 64
 typedef struct {
   //settings information
   Quadcopter_State_t current_state;
   //convert to timestamp
   uint16_t entry_counter;
   //time between entries - placeholder for planned variable pid dt based on IMU ready interrupt
   uint16_t loop_dt_us;

   uint16_t batt_voltage_mV;

   //data that pid loop is based on
   float pitch_angle;
   float roll_angle;

   //raw gyro data
   float pitch_rate;
   float roll_rate;
   float yaw_rate;

 //pilot command input after being converted into real units
   float pilot_pitch_command_angle;
   float pilot_roll_command_angle;
   float pilot_yaw_command_rate;
   float pilot_throttle_command;
//
   int16_t pid_pitch_out_pct;
   int16_t pid_roll_out_pct;
   int16_t pid_yaw_out_pct;

   //int16_t m4_output_raw;

   uint16_t m1_output_synthesized;
   uint16_t m2_output_synthesized;
   uint16_t m3_output_synthesized;
   uint16_t m4_output_synthesized;


   uint16_t crc;
 } FlightLog_Packet_t;

 typedef struct {
   //ticks up forever, allow automatic overflow to wraparound to zero
   uint32_t log_counter;
   uint32_t packet_version;

    float pitch_proportional_gain;
    float pitch_integrator_gain;
    float pitch_derivative_gain;

    float roll_proportional_gain;
    float roll_integrator_gain;
    float roll_derivative_gain;

    float yaw_proportional_gain;
    float yaw_integrator_gain;
    float yaw_derivative_gain;
    uint16_t crc;

 } FlightLogger_Metadata_t;

#endif