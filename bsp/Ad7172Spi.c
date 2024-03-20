/***********************************************************************
* @file Ad7172Spi
* Description:
*
**********************************************************************
* File name:      Ad7172Spi.c
* Date:           2023-07-19
* Version:        V1.0
* Author          KongYao
* @history:
* V1.0 创建文件
***********************************************************************/
#include "Ad7172Spi.h"
#include "main.h"
#include "spi.h"
#include "stdint.h"

extern SPI_HandleTypeDef hspi1;
#define USED_SPI  hspi1  //hsp1 
//CS 
#define USED_SPI_CS_PORT AD_CS_GPIO_Port //CS 
#define USED_SPI_CS_PIN AD_CS_Pin
//PA6 MISO
#define USED_SPI_MISO_PORT GPIOA
#define USED_SPI_MISO_PIN  GPIO_PIN_6
//PA7 MOSI
#define USED_SPI_MOSI_PORT GPIOA
#define USED_SPI_MOSI_PIN  GPIO_PIN_7
//PA5 SCK
#define USED_SPI_SCK_PORT GPIOA
#define USED_SPI_SCK_PIN  GPIO_PIN_5

/*************************************************************
** Function name:       SpiWriteAndRead
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
int SpiWriteAndRead(uint8_t* sendData, uint8_t* reData, uint16_t len) {
	HAL_SPI_TransmitReceive(&USED_SPI, sendData, reData, len, AD7172SPI_TIMEOUT_MS);
	return 1;
}

/*************************************************************
** Function name:       SpiTransfer
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
int SpiTransfer(uint8_t* pData, uint16_t len) {
	SetAD7172CsPin(0);
	HAL_SPI_Transmit(&USED_SPI, pData, len, AD7172SPI_TIMEOUT_MS);
	SetAD7172CsPin(1);
	return 1;
}

/*************************************************************
** Function name:       SpiRecevice
** Descriptions:
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
int SpiRecevice(uint8_t* pData, uint16_t len) {
	SetAD7172CsPin(0);
	HAL_SPI_Receive(&USED_SPI, pData, len, AD7172SPI_TIMEOUT_MS);
	SetAD7172CsPin(1);
	return 1;
}

/*************************************************************
** Function name:       SetAD7172CsPin
** Descriptions:        操作AD7172的片选引脚
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void SetAD7172CsPin(uint8_t state) {
	HAL_GPIO_WritePin(USED_SPI_CS_PORT, USED_SPI_CS_PIN, (GPIO_PinState)state);
}

/**
 * @brief Write/read multiple messages to/from SPI.
 * @param desc - The SPI descriptor.
 * @param msgs - The messages array.
 * @param len - Number of messages.
 * @return 0 in case of success, errno codes otherwise.
 */
int32_t stm32_spi_transfer(struct no_os_spi_msg* msgs, uint32_t len)
{
	for (uint32_t i = 0; i < len; i++) {

		SetAD7172CsPin(1);//TODO 在DOUT_RESET开启状态下 在读操作之前将CS拉高 使得输出为RDY信号
		// CS 拉低
			// gdesc->port->BSRR = NO_OS_BIT(sdesc->chip_select->number) << 16;
		SetAD7172CsPin(0);

		// 如果CS拉低之后需要延迟 则delay
		if (msgs[i].cs_delay_first) {
			// no_os_udelay(msgs[i].cs_delay_first);
		}

		HAL_SPI_TransmitReceive(&USED_SPI, msgs[i].tx_buff, msgs[i].rx_buff, msgs[i].bytes_number, AD7172SPI_TIMEOUT_MS);

		if (msgs[i].cs_delay_last) {
			// no_os_udelay(msgs[i].cs_delay_last);
		}

		if (msgs[i].cs_change) {
			/* De-assert CS */
			// gdesc->port->BSRR = NO_OS_BIT(sdesc->chip_select->number);
			SetAD7172CsPin(1);
		}


		if (msgs[i].cs_change_delay) {
			// no_os_udelay(msgs[i].cs_change_delay);
		}
	}

	return 0;
}


int32_t stm32_spi_write_and_read(uint8_t* data, uint16_t bytes_number)
{
	struct no_os_spi_msg msg = {
		.bytes_number = bytes_number,
		.cs_change = true,
		.rx_buff = data,
		.tx_buff = data,
	};

	if (!bytes_number)
		return 0;

	return stm32_spi_transfer(&msg, 1);
}



int32_t stm32_spi_receive(uint8_t* data, uint16_t bytes_number) {

	struct no_os_spi_msg msg = {
		.bytes_number = bytes_number,
		.cs_change = true,
		.rx_buff = data,
		.tx_buff = data,
	};

	if (!bytes_number)
		return 0;
	SetAD7172CsPin(0);//TODO 拉低CS 芯片响应
	HAL_SPI_Receive(&USED_SPI, msg.rx_buff, msg.bytes_number, AD7172SPI_TIMEOUT_MS);
	SetAD7172CsPin(1);//TODO 拉高CS 开启DOUT RESET
	return 0;
}


// SPI 的 MISO信号引脚 读取功能
uint8_t spi_miso_input(void) {
	return (uint8_t)HAL_GPIO_ReadPin(USED_SPI_MISO_PORT, USED_SPI_MISO_PIN);//通过宏定义进行切换
}



