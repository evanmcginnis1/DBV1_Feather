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
 
 typedef struct {
   //settings information
   Quadcopter_State_t current_state;
   //convert to timestamp
   uint16_t data_counter;

   //data that pid loop is based on
   float pitch_angle;
   float roll_angle;


   //raw gyro data
   float pitch_rate;
   float roll_rate;
   float yaw_rate;

   //missing roll rate 

   float pitch_command_angle;
   float roll_command_angle;

   float yaw_command_rate;
   float throttle_command;

   uint16_t m1_output_raw;
   uint16_t m1_output_normalized;
   uint16_t m2_output_raw;
   uint16_t m2_output_normalized;
   uint16_t m3_output_raw;
   uint16_t m3_output_normalized;
   uint16_t m4_output_raw;
   uint16_t m4_output_normalized;


   uint16_t crc;
 } FlightLog_Packet_t;

 typedef struct {
   //ticks up forever, allow automatic overflow to wraparound to zero
   uint32_t chunk_counter;
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