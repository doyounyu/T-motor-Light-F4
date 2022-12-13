/*
 * sumd.c
 *
 *  Created on: Dec 5, 2022
 *      Author: doyounyu
 */

#include "sumd.h"
#define CRC_POLYNOME 0x1021

SUMD_channel sumd_ch;

/**
  * @brief  Parse the received sumd data to channel output.
  * @param  sumd received data buffer array, output sumd channel structure
  * @retval none
  */


void SUMD_parsing(uint8_t sumd_rx_buf[], SUMD_channel sumd_ch[])
{
	sumd_ch ->  ch1 =  sumd_rx_buf[3]  << 8  |  sumd_rx_buf[4]  >> 3;
	sumd_ch ->  ch2 =  sumd_rx_buf[5]  << 8  |  sumd_rx_buf[6]  >> 3;
	sumd_ch ->  ch3 =  sumd_rx_buf[7]  << 8  |  sumd_rx_buf[8]  >> 3;
	sumd_ch ->  ch4 =  sumd_rx_buf[9]  << 8  |  sumd_rx_buf[10] >> 3;
	sumd_ch ->  ch5 =  sumd_rx_buf[11] << 8  |  sumd_rx_buf[12] >> 3;

	sumd_ch ->  ch6 =  sumd_rx_buf[13] << 8  |  sumd_rx_buf[14] >> 3;
	sumd_ch ->  ch7 =  sumd_rx_buf[15] << 8  |  sumd_rx_buf[16] >> 3;
	sumd_ch ->  ch8 =  sumd_rx_buf[17] << 8  |  sumd_rx_buf[18] >> 3;
	sumd_ch ->  ch9 =  sumd_rx_buf[19] << 8  |  sumd_rx_buf[20] >> 3;
	sumd_ch ->  ch10 = sumd_rx_buf[21] << 8  |  sumd_rx_buf[22] >> 3;

	sumd_ch ->  ch11 = sumd_rx_buf[23] << 8  |  sumd_rx_buf[24] >> 3;
	sumd_ch ->  ch12 = sumd_rx_buf[25] << 8  |  sumd_rx_buf[26] >> 3;
	sumd_ch ->  ch13 = sumd_rx_buf[27] << 8  |  sumd_rx_buf[28] >> 3;
	sumd_ch ->  ch14 = sumd_rx_buf[29] << 8  |  sumd_rx_buf[30] >> 3;
	sumd_ch ->  ch15 = sumd_rx_buf[31] << 8  |  sumd_rx_buf[32] >> 3;

	sumd_ch ->  ch16 = sumd_rx_buf[33] << 8  |  sumd_rx_buf[34] >> 3;

}


/**
  * @brief  converts SUMD channel value to OneShot 125 pulsewidth and parse the data to oneshot125 array
  * @param  oneShot125[]: converted 125-250 us values are saved here
  * 		sumd_ch[]: function gets SUMD channel data from here
  * @retval none
  */
void ch_data_to_oneShot125(double oneShot125[], SUMD_channel sumd_ch[])
{

		oneShot125[0] = (sumd_ch -> ch1)/51.2 - 45.234375;
		oneShot125[1] = (sumd_ch -> ch2)/51.2 - 45.234375;
		oneShot125[2] = (sumd_ch -> ch3)/51.2 - 45.234375;
		oneShot125[3] = (sumd_ch -> ch4)/51.2 - 45.234375;
		oneShot125[4] = (sumd_ch -> ch5)/51.2 - 45.234375;

		oneShot125[5] = (sumd_ch -> ch6)/51.2 - 45.234375;
		oneShot125[6] = (sumd_ch -> ch7)/51.2 - 45.234375;
		oneShot125[7] = (sumd_ch -> ch8)/51.2 - 45.234375;
		oneShot125[8] = (sumd_ch -> ch9)/51.2 - 45.234375;
		oneShot125[9] = (sumd_ch -> ch10)/51.2 - 45.234375;

		oneShot125[10] = (sumd_ch -> ch11)/51.2 - 45.234375;
		oneShot125[11] = (sumd_ch -> ch12)/51.2 - 45.234375;
		oneShot125[12] = (sumd_ch -> ch13)/51.2 - 45.234375;
		oneShot125[13] = (sumd_ch -> ch14)/51.2 - 45.234375;
		oneShot125[15] = (sumd_ch -> ch15)/51.2 - 45.234375;

		oneShot125[15] = (sumd_ch -> ch16)/51.2 - 45.234375;


}

/**
  * @brief  Check whether CRC16-Sick output is valid
  * @param  sumd received data buffer array, length of the buffer.
  * @retval calculated crc validity. 0 when valid.
  */
uint8_t CRC16_check(uint8_t sumd_rx_buf[], uint8_t len)
{
	uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++)
    {
        crc = CRC16(crc,sumd_rx_buf[i]);
    }

    return crc;
}

/**
  * @brief  calculate CRC16-Sick
  * @param
  * @retval calculated crc.
  */
uint16_t CRC16(uint16_t crc, uint8_t value)
{
	uint8_t i;
	crc = crc ^ (int16_t)value<<8;
	for(i=0; i<8; i++)
		{
		 if (crc & 0x8000)
		 crc = (crc << 1) ^ CRC_POLYNOME;

		 else
		 crc = (crc << 1);
		}
	return crc;
}
