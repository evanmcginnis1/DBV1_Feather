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

 #define PACKET_VERSION 4
 
 //MUST be of size 64
 typedef struct {
   //settings information
   Quadcopter_State_t current_state;
   //convert to timestamp
   uint16_t entry_counter;
   //time between entries - placeholder for planned variable pid dt based on IMU ready interrupt
   uint16_t loop_dt_us;

   uint16_t batt_voltage_mV;

   //unused for rate mode
   float pitch_angle;
   float roll_angle;

   //raw gyro data that pid loop is based on
   float pitch_rate;
   float roll_rate;
   float yaw_rate;

 //pilot command input after being converted into real units
   float pilot_pitch_command_rate;
   float pilot_roll_command_rate;
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

   uint16_t reserved[2];
   uint16_t crc;

 } FlightLog_Packet_t;

 typedef struct {
   //ticks up forever, allow automatic overflow to wraparound to zero
   uint32_t log_counter;
   uint32_t packet_version;

   uint16_t motor_output_idle;
   uint16_t motor_output_max;

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

 _Static_assert(sizeof(FlightLog_Packet_t) == 64, "FlightLog_Packet_t must be 64 bytes");
 _Static_assert(sizeof(FlightLogger_Metadata_t) <= 256, "Metadata must be less than 256 bytes");
#endif