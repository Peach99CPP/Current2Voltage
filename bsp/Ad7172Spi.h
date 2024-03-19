/***********************************************************************
* @file Ad7172Spi
* Description:
*
**********************************************************************
* File name:      Ad7172Spi.h
* Date:           2023-07-19
* Version:        V1.0
* Author          KongYao
* @history:
* V1.0 创建文件
***********************************************************************/
#ifndef __Ad7172Spi_H__
#define __Ad7172Spi_H__
#include "stdint.h"
#include "stdbool.h"


#define AD7172SPI_TIMEOUT_MS    200



/**
 * @struct no_os_spi_msg_list
 * @brief List item describing a SPI transfer
 */
struct no_os_spi_msg {
	/** Buffer with data to send. If NULL, 0x00 will be sent */
	uint8_t			*tx_buff;
	/** Buffer where to store data. If NULL, incoming data won't be saved */
	uint8_t			*rx_buff;
	/** Length of buffers. Must have equal size. */
	uint32_t		bytes_number;
	/** If set, CS will be deasserted after the transfer */
	uint8_t			cs_change;
	/**
	 * Minimum delay (in us) between the CS de-assert event of the current message
	 * and the assert of the next one.
	 */
	uint32_t		cs_change_delay;
	/** Delay (in us) between the CS assert and the first SCLK edge. */
	uint32_t		cs_delay_first;
	/** Delay (in us) between the last SCLK edge and the CS deassert */
	uint32_t		cs_delay_last;
};






int SpiWriteAndRead(uint8_t *sendData, uint8_t *reData, uint16_t len);
int SpiTransfer(uint8_t *pData, uint16_t len);
int SpiRecevice(uint8_t *pData, uint16_t len);

int32_t stm32_spi_write_and_read(uint8_t *data, uint16_t bytes_number);
void SetAD7172CsPin(uint8_t state);
int32_t stm32_spi_receive(uint8_t *data, uint16_t bytes_number);

uint8_t spi_miso_input(void);

#endif //__Ad7172Spi_H__




