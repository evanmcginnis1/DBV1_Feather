# Overview
iBus is a protocol for communication between a wireless reciever and flight controller. 
* New value can be read every 7 ms
* Supports up to 14 channels
* Each frame is 32 bytes
* Values received on each channel are between 1000 and 2000
* 115200 serial baud rate
* Requires UART RX pin
* Little Endian
# Packet Structure
* 2 header bytes
* 14 channels (2 bytes each)
* 2 byte checksum
# Checksum Calculation
	checksum = 0xFFFF - (sum of previous 30 bytes)

# Decoding the Packet
* The first byte should always be 0x20, which specifies that the packet will be 32 bytes long
* The next byte will always be 0x40, which is the command code for throttling a motor
* Then, the next 28 bytes will be the channel data
	* Data is sent in little endian, which means that the least significant byte comes first. To combine the two data bytes per channel, you need to bit shift the second byte to the left by 8 bits, and then OR it with the first byte. 

```
##channel 1 data
buf[4] << 8 | buf[3];
```

# Code Implementation
## High Level
Read 32 byte UART packets using DMA, then decode them and send commands to FC
## Low Level
* Need software failsafe in order to avoid sending stale data to flight controller
	* Every time DMA sends a packet of data, reset stale_flag & stale_counter to zero
	* use HAL_UART_RxCpltCallback function. It is called every time the UART completes (reaches a stop bit)
* Normal operation
	1. UART buffer constantly updated by DMA
		1. every time UART reads new data, it resets stale_flag
	2. Read first two bytes of buffer, verify they are correct 
	3. Update channel data
	4. Call failsafe function
		1. increments stale_flag by 1 every time it is called
	5. 

## FS i6 Channel Mapping (AETR)

| Channel Number |  Quad Function |
| -------------- |  ------------- |
| 1              | Roll           |
| 2              | Pitch          |
| 3              | Throttle       |
| 4              | Yaw            |
| 5              | Soft-disarm    |
| 6              | hard-disarm    |

# References
https://github.com/mokhwasomssi/stm32_hal_ibus
https://thenerdyengineer.com/ibus-and-arduino/#iBus_protocol_Description
