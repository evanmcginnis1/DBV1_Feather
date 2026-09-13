# Overview
Dshot is a protocol for communication between a flight controller and ESC. It features error-checking, high resolution, and very high data rates. It can run at DShot150, DShot300, DShot600,

![[DShot Timing Info.png]]^[https://betaflight.com/docs/development/API/Dshot]

# Frame Structure
* 2 bytes total
* First 11 bits represent throttle command (represents throttle values from 48-2048)
	* Bit 0 reserved for disarmed
	* Bits 1-47 reserved for special commands
* 1 bit telemetry request
* 4 bit CRC
* 2 µs break between frames to indicate frame reset BUT can be much longer
	* make break between frames longer to sync with PID loop

# CRC Calculation
CRC = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F
* uses [[Glossary#XOR| XOR operator]] and three copies of value to ensure that CRC most likely changes if any bit changes
* Value = First 12 bits of frame (throttle + telemetry request bit)

# Arming Sequence
Can differ by implementation, but most of the time, a 0 is expected for some period of time

# DShot 600
* 600 kbit/s baud rate
* 12MHz timer rate
	* So that DShot300 will be 6MHz, DShot150 will be 3MHz
* T1H is 75% of bit length
* T0H is 37.5% of bit length
# Code Implementation
need a  DMA_buffer for each motor (uint32_t). 16 bits to hold + 2 bits of dead time between bytes
## 1. Determine Tick Frequency
1. Goal: Want the tick period to add up cleanly into the T1H time and T0H time
2. Tick frequency must be a factor of 84MHz because timer clock prescaler must be an integer
3. Tick frequency (ticks/s) = bit rate (bits/s) * bit length (ticks/bit)
	1. First, try 12MHz, following stm_hal_dshot example code. Tick length = 20. Result: T1H = 15 ticks (1.25µs), T0H = 7.5 ticks --> 8 ticks (0.6666666µs)
	2. Try running at 42MHz. Tick length = 70. Result: T1H = 52.5 ticks --> 53 ticks (1.262µs), T0H = 26.25 ticks --> 26 ticks (0.619µs)
	3. Try 84MHz (full speed). Tick length = 140. T1H = 105 ticks (1.25µs), T0H = 52.5 ticks --> 53 ticks (0.631µs).
4. Result: 84MHz is closest to the DShot spec (least % error), so run at that speed. 
Final numbers:
Tick length = 140 ticks/bit
Tick Frequency = 84MHz
T1H = 105 ticks
T0H: 53 ticks
### DShot600 Timings for Various Tick Frequencies

| Tick Frequency (MHz) | PSC | Tick Length (bits/s) | T1H (µs) | T1H error (%) | T0H (µs) | T0H error (%) |
| -------------------- | --- | -------------------- | -------- | ------------- | -------- | ------------- |
| 84                   | 0   | 140                  | 1.25     | 0%            | 0.631    | 0.96%         |
| 42                   | 1   | 70                   | 1.262    | 0.96%         | 0.619    | 0.96%         |
| 12                   | 6   | 20                   | 1.25     | 0%            | 0.666    | 6.56%         |

* Tick length = Tick Rate / baud rate (600Kbit/s)
* Actual prescaler value is PSC + 1
## 2. Determine Prescaler Values
1. We know that: tick_freq = TIMxCLK / (PSC + 1)
2. To find TIM1 PSC, rearrange to get: PSC = (TIMxCLK / tick_freq) - 1 
**Because TIM8 is on the faster APB2, its PSC must be twice as much as the TIM3 PSC**
* Adjusting for different baud rates
	* Prescaler is based on tick frequency so that all of the rest of the math can be the same; a high signal and a low one are still the same number of ticks, but because the ticks are further apart, the timing scales appropriately for the different baud rates. 
	* DShot300: PSC = 2
	* DShot150: PSC= 4
## 3. Start PWM for each channel
## 4. Calculate CRC
See [[#CRC Calculation]]
## 4. Put data in DMA Buffer
1. Read through each bit in the throttle buffer
	1. If a bit is a 1, write 105 into dmabuf
	2. if bit is a 0, write 53 into dmabuf
## 5. Start DMA
Tell DMA controller to send data 

# Actual DShot Timings (84MHz TIM_CLK)

| DShot | Bitrate   | Tick Frequency | Tick Period | T1H (µs) | T0H (µs) | Bit (µs) | Frame (µs) |
| ----- | --------- | -------------- | ----------- | -------- | -------- | -------- | ---------- |
| 150   | 150kbit/s | 21 MHz         | 0.048µs     | 5        | 2.52     | 6.67     | 106.66     |
| 300   | 300kbit/s | 42 MHz         | 0.024µs     | 2.50     | 1.262    | 3.33     | 53.33      |
| 600   | 600kbit/s | 84 MHz         | 0.012µs     | 1.25     | 0.63     | 1.67     | 26.66      |




# References
https://blck.mn/2016/11/dshot-the-new-kid-on-the-block/ 
https://github.com/mokhwasomssi/stm32_hal_dshot/tree/main (MIT License)
https://betaflight.com/docs/development/API/Dshot
https://www.swallenhardware.io/battlebots/2019/4/20/a-developers-guide-to-dshot-escs
https://brushlesswhoop.com/dshot-and-bidirectional-dshot/#frame-structure
https://docs.px4.io/main/en/peripherals/dshot

[[DShot Debugging]]

# Tags
#DBV1/esc 