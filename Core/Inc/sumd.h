/*
 * sumd.h
 *
 *  Created on: Dec 5, 2022
 *      Author: doyounyu
 */

#ifndef INC_SUMD_H_
#define INC_SUMD_H_

#include "main.h"




typedef struct _SUMD_channel
{
	uint16_t ch1;  // Throttle
	uint16_t ch2;  // Roll
	uint16_t ch3;  // Pitch
	uint16_t ch4;  // Yaw
	uint16_t ch5;  // Arming
	uint16_t ch6;
	uint16_t ch7;
	uint16_t ch8;
	uint16_t ch9;
	uint16_t ch10;
	uint16_t ch11;
	uint16_t ch12;
	uint16_t ch13;
	uint16_t ch14;
	uint16_t ch15;
	uint16_t ch16;

}SUMD_channel;


extern SUMD_channel sumd_ch;


void SUMD_parsing(uint8_t sumd_rx_buf[], SUMD_channel sumd_ch[]);

void ch_data_to_oneShot125(double oneShot125[], SUMD_channel sumd_ch[]);


uint8_t CRC16_check(uint8_t sumd_rx_buf[], uint8_t len);

uint16_t CRC16(uint16_t crc, uint8_t value);

#endif /* INC_SUMD_H_ */
