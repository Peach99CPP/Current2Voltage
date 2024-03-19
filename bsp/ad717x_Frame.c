/***************************************************************************/ /**
																			   *   @file    AD717X.c
																			   *   @brief   AD717X implementation file.
																			   *   	     Devices: AD7172-2, AD7172-4, AD7173-8, AD7175-2, AD7175-8, AD7176-2
																			   *            AD7177-2, AD4111, AD4112, AD4114, AD4115, AD4116
																			   *   @author  acozma (andrei.cozma@analog.com)
																			   *            dnechita (dan.nechita@analog.com)
																			   *
																			   ********************************************************************************
																			   * Copyright 2015(c) Analog Devices, Inc.
																			   *
																			   * All rights reserved.
																			   *
																			   * Redistribution and use in source and binary forms, with or without modification,
																			   * are permitted provided that the following conditions are met:
																			   *  - Redistributions of source code must retain the above copyright
																			   *    notice, this list of conditions and the following disclaimer.
																			   *  - Redistributions in binary form must reproduce the above copyright
																			   *    notice, this list of conditions and the following disclaimer in
																			   *    the documentation and/or other materials provided with the
																			   *    distribution.
																			   *  - Neither the name of Analog Devices, Inc. nor the names of its
																			   *    contributors may be used to endorse or promote products derived
																			   *    from this software without specific prior written permission.
																			   *  - The use of this software may or may not infringe the patent rights
																			   *    of one or more patent holders.  This license does not release you
																			   *    from the requirement that you obtain separate licenses from these
																			   *    patent holders to use this software.
																			   *  - Use of the software either in source or binary form, must be run
																			   *    on or directly connected to an Analog Devices Inc. component.
																			   *
																			   * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
																			   * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
																			   * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
																			   * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
																			   * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
																			   * INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
																			   * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
																			   * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
																			   * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
																			   * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
																			   *******************************************************************************/

																			   /******************************************************************************/
																			   /***************************** Include Files **********************************/
																			   /******************************************************************************/
#include <stdlib.h>
#include "ad717x_Frame.h"
#include "string.h"
#include "main.h"
#include "gpio.h"
#include "usart.h"
#include "stdio.h"
#include "My_AD7172.h"
#define REF_POS_Voltage 5.00
#define REF_NEG_Voltage 0.00
double convertedVal[4] = { 0, 0, 0, 0 };
ad717x_st_reg* ID_preg = 0;
ad717x_st_reg* CH_map0_preg = 0;
ad717x_st_reg* CH_map1_preg = 0;
ad717x_st_reg* CH_map2_preg = 0;
ad717x_st_reg* CH_map3_Preg = 0;
ad717x_st_reg* ADCMODE_preg = 0;
ad717x_st_reg* IFMODE_preg = 0;
ad717x_st_reg* SETUPCON0_preg = 0;
ad717x_st_reg* FILTCON0_preg = 0;
ad717x_st_reg* OFFSET0_preg = 0;
ad717x_st_reg* GAIN0_preg = 0;

/* Error codes */
#define INVALID_VAL -1 /* Invalid argument */
#define COMM_ERR -2	   /* Communication error on receive */
#define TIMEOUT -3	   /* A timeout has occured */
#define AD7172ERR 99   /* A timeout has occured */

/***************************************************************************
																			   * @brief Set channel status - Enable/Disable
																			   * @param device - AD717x Device descriptor.
																			   * @param channel_id - Channel ID (number) of the channel whose status is to be set.
																			   * @param channel_status - Required status of the channel-True in case of Enable
																			   *			    	and False in case of Disable
																			   * @return Returns 0 for success or negative error code in case of failure.
																			   *******************************************************************************/
int ad717x_set_channel_status(PAD7172_Struct dev, uint8_t channel_id,
	bool channel_status)
{
	ad717x_st_reg* chn_register;
	int ret;

	/* Point to the Channel register */
	chn_register = AD717X_GetReg(dev, AD717X_CHMAP0_REG + channel_id);
	if (!chn_register)
		return -AD7172ERR;

	if (channel_status)
		/* Assign the Channel enable bit and write to channel register */
		chn_register->value |= AD717X_CHMAP_REG_CH_EN;
	else
		chn_register->value &= ~(AD717X_CHMAP_REG_CH_EN);

	ret = AD717X_WriteRegister(dev, AD717X_CHMAP0_REG + channel_id);
	if (ret < 0)
		return ret;
	dev->chan_map[channel_id].channel_enable = channel_status;

	return 0;
}

uint32_t AD7172_GENMASK(int h, int l)
{
	uint32_t t = (uint32_t)(~0UL);
	t = t << (NO_OS_BITS_PER_LONG - (h - l + 1));
	t = t >> (NO_OS_BITS_PER_LONG - (h + 1));
	return t;
}

/***************************************************************************/ /**
																			   * @brief Set ADC Mode
																			   * @param device - AD717x Device Descriptor
																			   * @param adc_mode - ADC Mode to be configured
																			   * @return Returns 0 for success or negative error code in case of failure.
																			   ******************************************************************************/
int ad717x_set_adc_mode(PAD7172_Struct dev, enum ad717x_mode adc_mode)
{
	ad717x_st_reg* adc_mode_reg;

	if (!dev)
		return -AD7172ERR;

	/* Retrieve the ADC Mode reigster */
	adc_mode_reg = AD717X_GetReg(dev, AD717X_ADCMODE_REG);
	if (!adc_mode_reg)
	{
		return -AD7172ERR;
	}

	/* Clear the Mode[6:4] bits in the ADC Mode Register */
	adc_mode_reg->value &= ~(AD717X_ADCMODE_REG_MODE_MSK);

	/* Set the required conversion mode, write to register */
	adc_mode_reg->value |= AD717X_ADCMODE_REG_MODE(adc_mode);
	if (AD717X_WriteRegister(dev, AD717X_ADCMODE_REG) < 0)
		return -AD7172ERR;
	dev->mode = adc_mode;

	return 0;
}

/***************************************************************************/ /**
																			   * @brief Set Analog Inputs to channel
																			   * @param device - AD717x Device Descriptor
																			   * @param channel_id - Channel whose Analog input is to be configured
																			   * @param analog_input - Analog Inputs to the Channel
																			   * @return Returns 0 for success or negative error code in case of failure.
																			   *****************************************************************************/
int ad717x_connect_analog_input(PAD7172_Struct dev, uint8_t channel_id,
	union ad717x_analog_inputs analog_input)
{
	ad717x_st_reg* channel_reg;

	if (!dev)
		return -AD7172ERR;

	/* Retrieve the channel register */
	channel_reg = AD717X_GetReg(dev, AD717X_CHMAP0_REG + channel_id);
	if (!channel_reg)
		return -AD7172ERR;

	switch ((uint8_t)dev->active_device)
	{
	case ID_AD4111:
	case ID_AD4112:
	case ID_AD4114:
	case ID_AD4115:
	case ID_AD4116:
		/* Clear and Set the required analog input pair to channel */
		channel_reg->value &= ~AD717x_CHANNEL_INPUT_MASK;
		channel_reg->value |= AD4111_CHMAP_REG_INPUT(analog_input.analog_input_pairs);
		if (AD717X_WriteRegister(dev, AD717X_CHMAP0_REG + channel_id) < 0)
			return -AD7172ERR;

		dev->chan_map[channel_id].analog_inputs.analog_input_pairs =
			analog_input.analog_input_pairs;
		break;

	case ID_AD7172_4:
	case ID_AD7173_8:
	case ID_AD7175_2:
	case ID_AD7175_8:
	case ID_AD7176_2:
	case ID_AD7177_2:
	case ID_AD7172_2:
		/* Select the Positive Analog Input */
		channel_reg->value &= ~AD717X_CHMAP_REG_AINPOS_MSK;
		channel_reg->value |= AD717X_CHMAP_REG_AINPOS(
			analog_input.ainp.pos_analog_input);

		/* Select the Negative Analog Input */
		channel_reg->value &= ~AD717X_CHMAP_REG_AINNEG_MSK;
		channel_reg->value |= AD717X_CHMAP_REG_AINNEG(
			analog_input.ainp.neg_analog_input);
		if (AD717X_WriteRegister(dev, AD717X_CHMAP0_REG + channel_id) < 0)
			return -AD7172ERR;

		dev->chan_map[channel_id].analog_inputs.ainp.pos_analog_input =
			analog_input.ainp.pos_analog_input;
		dev->chan_map[channel_id].analog_inputs.ainp.neg_analog_input =
			analog_input.ainp.neg_analog_input;
		break;

	default:
		return -AD7172ERR;
	}

	return 0;
}

/***************************************************************************/ /**
																			   * @brief Assign Setup to Channel
																			   * @param device - AD717x Device Descriptor
																			   * @param channel_id - Channel ID (number)
																			   * @param setup - Setup ID (number)
																			   * @return Returns 0 for success or negative error code in case of failure.
																			   ******************************************************************************/
int ad717x_assign_setup(PAD7172_Struct dev, uint8_t channel_id, uint8_t setup)
{
	ad717x_st_reg* p_register;

	if (!dev)
		return -AD7172ERR;

	/* Retrieve the Channel Register */
	p_register = AD717X_GetReg(dev, AD717X_CHMAP0_REG + channel_id);
	if (!p_register)
		return -AD7172ERR;

	/* Assign set up to the chosen channel */
	p_register->value &= ~AD717X_CHMAP_REG_SETUP_SEL_MSK;
	p_register->value |= AD717X_CHMAP_REG_SETUP_SEL(setup);

	if (AD717X_WriteRegister(dev, AD717X_CHMAP0_REG + channel_id) < 0)
		return -AD7172ERR;
	dev->chan_map[channel_id].setup_sel = setup;

	return 0;
}

/***************************************************************************/ /**
																			   * @brief Set Polarity
																			   * @param device - AD717x Device Descriptor
																			   * @param bipolar - Polarity Select:True in case of Bipolar, False in case of Unipolar
																			   * @param setup_id - Setup ID (number)
																			   * @return Returns 0 for success or negative error code in case of failure.
																			   *****************************************************************************/
int ad717x_set_polarity(PAD7172_Struct dev, bool bipolar, uint8_t setup_id)
{
	ad717x_st_reg* setup_reg;

	/* Retrieve the SETUPCON Register */
	setup_reg = AD717X_GetReg(dev, AD717X_SETUPCON0_REG + setup_id);
	if (!setup_reg)
		return -AD7172ERR;

	/* Set the BI_UNIPOLAR bit in case of BIPOLAR operation */
	if (bipolar)
		setup_reg->value |= AD717X_SETUP_CONF_REG_BI_UNIPOLAR;
	else
		setup_reg->value &= ~(AD717X_SETUP_CONF_REG_BI_UNIPOLAR);

	if (AD717X_WriteRegister(dev,
		AD717X_SETUPCON0_REG + setup_id) < 0)
		return -AD7172ERR;
	dev->setups[setup_id].bi_unipolar = bipolar;

	return 0;
}

/***************************************************************************/ /**
																			   * @brief Select the reference source
																			   * @param device - AD717x Device Descriptor
																			   * @param ref_source - Reference source
																			   * @param setup_id - Setup ID (Number)
																			   * @return Returns 0 for success or negative error code in case of failure.
																			   ******************************************************************************/
int ad717x_set_reference_source(PAD7172_Struct dev, enum ad717x_reference_source ref_source, uint8_t setup_id)
{
	ad717x_st_reg* setup_reg;
	ad717x_st_reg* adc_mode_reg;

	/* Retrieve the SETUPCON Register */
	setup_reg = AD717X_GetReg(dev, AD717X_SETUPCON0_REG + setup_id);
	if (!setup_reg)
		return -AD7172ERR;

	/* Choose the reference source for the selected setup */
	setup_reg->value &= ~AD717X_SETUP_CONF_REG_REF_SEL_MSK;
	setup_reg->value |= (AD717X_SETUP_CONF_REG_REF_SEL(ref_source));

	if (AD717X_WriteRegister(dev,
		AD717X_SETUPCON0_REG + setup_id) < 0)
		return -AD7172ERR;
	dev->setups[setup_id].ref_source = ref_source;

	/* Enable the REF_EN Bit in case of Internal reference */
	if (ref_source == INTERNAL_REF)
	{
		/* Retrieve the ADC Mode reigster */
		adc_mode_reg = AD717X_GetReg(dev, AD717X_ADCMODE_REG);
		if (!adc_mode_reg)
			return -AD7172ERR;

		/* Set the REF_EN Bit */
		adc_mode_reg->value |= AD717X_ADCMODE_REG_REF_EN;
		if (AD717X_WriteRegister(dev, AD717X_ADCMODE_REG) < 0)
			return -AD7172ERR;
		dev->ref_en = true;
	}

	return 0;
}

/***************************************************************************/ /**
																			   * @brief Enable Input Buffer
																			   * @param device - AD717x Device Descriptor
																			   * @param inbuf_en - Enable Inpur Buffer
																			   * @param refbuf_en - Enable referece Buffer
																			   * @param setup_id - Setup ID (Number)
																			   * @return Returns 0 for success or negative error code in case of failure.
																			   ******************************************************************************/
int ad717x_enable_input_buffer(PAD7172_Struct dev, bool inbuf_en, bool refbuf_en, uint8_t setup_id)
{
	ad717x_st_reg* setup_reg;

	/* Retrieve the SETUPCON Register */
	setup_reg = AD717X_GetReg(dev, AD717X_SETUPCON0_REG + setup_id);
	if (!setup_reg)
		return -AD7172ERR;

	if (inbuf_en)
		/* Enable input buffer for the chosen set up */
		setup_reg->value |= (AD717X_SETUP_CONF_REG_AINBUF_P |
			AD717X_SETUP_CONF_REG_AINBUF_N);
	else
		setup_reg->value &= (~(AD717X_SETUP_CONF_REG_AINBUF_P |
			AD717X_SETUP_CONF_REG_AINBUF_N));
	if (refbuf_en)
		/* Enable reference buffer for the chosen set up */
		setup_reg->value |= (AD717X_SETUP_CONF_REG_REFBUF_P |
			AD717X_SETUP_CONF_REG_REFBUF_N);
	else
		setup_reg->value &= (~(AD717X_SETUP_CONF_REG_REFBUF_P |
			AD717X_SETUP_CONF_REG_REFBUF_N));

	if (AD717X_WriteRegister(dev,
		AD717X_SETUPCON0_REG + setup_id) < 0)
		return -AD7172ERR;
	dev->setups[setup_id].input_buff = inbuf_en;
	dev->setups[setup_id].ref_buff = refbuf_en;
	return 0;
}

/***************************************************************************/ /**
																			   * @brief Perform Single Conversion
																			   * @param device - AD717x Device Descriptor
																			   * @param id - Channel ID (number) requested
																			   * @param adc_raw_data ADC Raw Value
																			   * @return Returns 0 for success or negative error code in case of failure.
																			   ******************************************************************************/
int ad717x_single_read(PAD7172_Struct dev, uint8_t id, int32_t* adc_raw_data)
{
	int ret;

	/* Enable the requested channel */
	ret = ad717x_set_channel_status(dev, id, true);
	if (ret < 0)
		return ret;

	/* Set Mode to Single Conversion */
	ret = ad717x_set_adc_mode(dev, SINGLE);
	if (ret < 0)
		return ret;

	/* Wait for Conversion completion */
	ret = AD717X_WaitForReady(dev, AD717X_CONV_TIMEOUT);
	if (ret < 0)
		return ret;

	/* Read the data register */
	ret = AD717X_ReadData(dev, adc_raw_data);
	if (ret < 0)
		return ret;

	/* Disable the current channel */
	return ad717x_set_channel_status(dev, id, false);
}

/***************************************************************************/ /**
																			   * @brief  Searches through the list of registers of the driver instance and
																			   *         retrieves a pointer to the register that matches the given address.
																			   *
																			   * @param device - The handler of the instance of the driver.
																			   * @param reg_address - The address to be used to find the register.
																			   *
																			   * @return A pointer to the register if found or 0.
																			   *******************************************************************************/
ad717x_st_reg* AD717X_GetReg(PAD7172_Struct dev,
	uint8_t reg_address)
{
	uint8_t i;
	ad717x_st_reg* reg = 0;

	for (i = 0; i < dev->num_regs; i++)
	{
		if (dev->regs[i].addr == reg_address)
		{
			reg = &dev->regs[i];
			break;
		}
	}

	return reg;
}

/***************************************************************************/ /**
																			   * @brief Reads the value of the specified register.
																			   *
																			   * @param device - The handler of the instance of the driver.
																			   * @param addr - The address of the register to be read. The value will be stored
																			   *         inside the register structure that holds info about this register.
																			   *
																			   * @return Returns 0 for success or negative error code.
																			   *******************************************************************************/
int32_t AD717X_ReadRegister(PAD7172_Struct dev,
	uint8_t addr)
{
	int32_t ret = 0;
	uint8_t buffer[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t i = 0;
	uint8_t check8 = 0;
	uint8_t msgBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	ad717x_st_reg* pReg;

	pReg = AD717X_GetReg(dev, addr);
	if (!pReg)
		return INVALID_VAL;

	/* Build the Command word */
	buffer[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD |
		AD717X_COMM_REG_RA(pReg->addr);

	/* Read data from the device */
	ret = dev->spi_write_and_read(buffer,
		((dev->useCRC != AD717X_DISABLE) ? pReg->size + 1
			: pReg->size) +
		1);
	if (ret < 0)
		return ret;

	/* Check the CRC */
	if (dev->useCRC == AD717X_USE_CRC)
	{
		msgBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD |
			AD717X_COMM_REG_RA(pReg->addr);
		for (i = 1; i < pReg->size + 2; ++i)
		{
			msgBuf[i] = buffer[i];
		}
		check8 = AD717X_ComputeCRC8(msgBuf, pReg->size + 2);
	}
	if (dev->useCRC == AD717X_USE_XOR)
	{
		msgBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_RD |
			AD717X_COMM_REG_RA(pReg->addr);
		for (i = 1; i < pReg->size + 2; ++i)
		{
			msgBuf[i] = buffer[i];
		}
		check8 = AD717X_ComputeXOR8(msgBuf, pReg->size + 2);
	}

	if (check8 != 0)
	{
		/* ReadRegister checksum failed. */
		return COMM_ERR;
	}

	/* Build the result */
	pReg->value = 0;
	for (i = 1; i < pReg->size + 1; i++)
	{
		pReg->value <<= 8;
		pReg->value += buffer[i];
	}

	return ret;
}

/***************************************************************************/ /**
																			   * @brief Writes the value of the specified register.
																			   *
																			   * @param device - The handler of the instance of the driver.
																			   * @param addr   - The address of the register to be written with the value stored
																			   *               inside the register structure that holds info about this
																			   *               register.
																			   *
																			   * @return Returns 0 for success or negative error code.
																			   *******************************************************************************/
																			   // wrBuf 组成：
																			   // 		COMMS寄存器 + Data(length看要操作的Reg的长度决定) + CRC

int32_t AD717X_WriteRegister(PAD7172_Struct dev, uint8_t addr)
{
	int32_t ret = 0;
	int32_t regValue = 0;
	uint8_t wrBuf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t i = 0;
	uint8_t crc8 = 0;
	ad717x_st_reg* preg;

	preg = AD717X_GetReg(dev, addr);
	if (!preg)
		return INVALID_VAL;

	/* Build the Command word */
	wrBuf[0] = AD717X_COMM_REG_WEN | AD717X_COMM_REG_WR |
		AD717X_COMM_REG_RA(preg->addr);

	/* Fill the write buffer */
	regValue = preg->value;
	for (i = 0; i < preg->size; i++)
	{
		wrBuf[preg->size - i] = regValue & 0xFF;
		regValue >>= 8;
	}

	/* Compute the CRC */
	if (dev->useCRC != AD717X_DISABLE)
	{
		crc8 = AD717X_ComputeCRC8(wrBuf, preg->size + 1);
		wrBuf[preg->size + 1] = crc8;
	}

	/* Write data to the device */
	ret = dev->spi_write_and_read(wrBuf,
		(dev->useCRC != AD717X_DISABLE) ? preg->size + 2 : preg->size + 1);
	return ret;
}

/***************************************************************************/ /**
																			   * @brief Resets the device.
																			   *
																			   * @param device - The handler of the instance of the driver.
																			   *
																			   * @return Returns 0 for success or negative error code.
																			   *******************************************************************************/
int32_t AD717X_Reset(PAD7172_Struct dev)
{
	int32_t ret = 0;
	uint8_t wrBuf[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

	ret = dev->spi_write_and_read(wrBuf,
		8);

	return ret;
}

/***************************************************************************/ /**
																			   * @brief Waits until a new conversion result is available.
																			   *
																			   * @param device  - The handler of the instance of the driver.
																			   * @param timeout - Count representing the number of polls to be done until the
																			   *                  function returns if no new data is available.
																			   *
																			   * @return Returns 0 for success or negative error code.
																			   *******************************************************************************/
int32_t AD717X_WaitForReady(PAD7172_Struct dev,
	uint32_t timeout)
{
	ad717x_st_reg* statusReg;
	int32_t ret;
	int8_t ready = 0;

	statusReg = AD717X_GetReg(dev, AD717X_STATUS_REG);
	if (!statusReg)
		return INVALID_VAL;

	while (!ready && --timeout)
	{
		/* Read the value of the Status Register */
		ret = AD717X_ReadRegister(dev, AD717X_STATUS_REG);
		if (ret < 0)
			return ret;

		/* Check the RDY bit in the Status Register */
		ready = (statusReg->value & AD717X_STATUS_REG_RDY) == 0;
	}
	return timeout ? 0 : TIMEOUT;
}

/***************************************************************************/ /**
																			   * @brief Reads the conversion result from the device.
																			   *
																			   * @param device - The handler of the instance of the driver.
																			   * @param pData  - Pointer to store the read data.
																			   *
																			   * @return Returns 0 for success or negative error code.
																			   *******************************************************************************/
int32_t AD717X_ReadData(PAD7172_Struct dev,
	int32_t* pData)
{
	ad717x_st_reg* dataReg;
	int32_t ret;

	if (!dev || !dev->regs)
		return INVALID_VAL;

	dataReg = AD717X_GetReg(dev, AD717X_DATA_REG);
	if (!dataReg)
		return INVALID_VAL;

	/* Update the data register length with respect to device and options */
	ret = AD717X_ComputeDataregSize(dev);

	/* Read the value of the Status Register */
	ret |= AD717X_ReadRegister(dev, AD717X_DATA_REG);

	/* Get the read result */
	*pData = dataReg->value;

	return ret;
}

/***************************************************************************/ /**
																			   * @brief Computes data register read size to account for bit number and status
																			   * 		 read.
																			   *
																			   * @param device - The handler of the instance of the driver.
																			   *
																			   * @return 0in case of success or negative code in case of failure.
																			   *******************************************************************************/
int32_t AD717X_ComputeDataregSize(PAD7172_Struct dev)
{
	ad717x_st_reg* reg_ptr;
	ad717x_st_reg* datareg_ptr;
	uint16_t case_var;

	/* Get interface mode register pointer */
	reg_ptr = AD717X_GetReg(dev, AD717X_IFMODE_REG);
	/* Get data register pointer */
	datareg_ptr = AD717X_GetReg(dev, AD717X_DATA_REG);
	case_var = reg_ptr->value & (AD717X_IFMODE_REG_DATA_STAT |
		AD717X_IFMODE_REG_DATA_WL16);

	/* Compute data register size */
	datareg_ptr->size = 3;
	if ((case_var & AD717X_IFMODE_REG_DATA_WL16) == AD717X_IFMODE_REG_DATA_WL16)
		datareg_ptr->size--;
	if ((case_var & AD717X_IFMODE_REG_DATA_STAT) == AD717X_IFMODE_REG_DATA_STAT)
		datareg_ptr->size++;

	/* Get ID register pointer */
	reg_ptr = AD717X_GetReg(dev, AD717X_ID_REG);

	/* If the part is 32/24 bit wide add a byte to the read */
	if ((reg_ptr->value & AD717X_ID_REG_MASK) == AD7177_2_ID_REG_VALUE)
		datareg_ptr->size++;

	return 0;
}

/***************************************************************************/ /**
																			   * @brief Computes the CRC checksum for a data buffer.
																			   *
																			   * @param pBuf    - Data buffer
																			   * @param bufSize - Data buffer size in bytes
																			   *
																			   * @return Returns the computed CRC checksum.
																			   *******************************************************************************/
uint8_t AD717X_ComputeCRC8(uint8_t* pBuf,
	uint8_t bufSize)
{
	uint8_t i = 0;
	uint8_t crc = 0;

	while (bufSize)
	{
		for (i = 0x80; i != 0; i >>= 1)
		{
			if (((crc & 0x80) != 0) != ((*pBuf & i) !=
				0))
			{ /* MSB of CRC register XOR input Bit from Data */
				crc <<= 1;
				crc ^= AD717X_CRC8_POLYNOMIAL_REPRESENTATION;
			}
			else
			{
				crc <<= 1;
			}
		}
		pBuf++;
		bufSize--;
	}
	return crc;
}

/***************************************************************************/ /**
																			   * @brief Computes the XOR checksum for a data buffer.
																			   *
																			   * @param pBuf    - Data buffer
																			   * @param bufSize - Data buffer size in bytes
																			   *
																			   * @return Returns the computed XOR checksum.
																			   *******************************************************************************/
uint8_t AD717X_ComputeXOR8(uint8_t* pBuf,
	uint8_t bufSize)
{
	uint8_t xor = 0;

	while (bufSize)
	{
		xor ^= *pBuf;
		pBuf++;
		bufSize--;
	}
	return xor;
}

/***************************************************************************/ /**
																			   * @brief Updates the CRC settings.
																			   *
																			   * @param device - The handler of the instance of the driver.
																			   *
																			   * @return Returns 0 for success or negative error code.
																			   *******************************************************************************/
int32_t AD717X_UpdateCRCSetting(PAD7172_Struct dev)
{
	ad717x_st_reg* interfaceReg;

	interfaceReg = AD717X_GetReg(dev, AD717X_IFMODE_REG);
	if (!interfaceReg)
		return INVALID_VAL;

	/* Get CRC State. */
	if (AD717X_IFMODE_REG_CRC_STAT(interfaceReg->value))
	{
		dev->useCRC = AD717X_USE_CRC;
	}
	else if (AD717X_IFMODE_REG_XOR_STAT(interfaceReg->value))
	{
		dev->useCRC = AD717X_USE_XOR;
	}
	else
	{
		dev->useCRC = AD717X_DISABLE;
	}

	return 0;
}

/**
 * @brief Configure ODR for the device
 * @param dev - The AD717x Device descriptor
 * @param filtcon_id - Filter Configuration Register ID (Number)
 * @param odr_sel - ODR[4:0] bitfield value as a decimal
 * @return 0 in case of success, negative error code otherwise
 */
int32_t ad717x_configure_device_odr(PAD7172_Struct dev,
	uint8_t filtcon_id,
	uint8_t odr_sel)
{
	ad717x_st_reg* filtcon_reg;
	int32_t ret;

	/* Retrieve the FILTCON register */
	filtcon_reg = AD717X_GetReg(dev,
		AD717X_FILTCON0_REG + filtcon_id);
	if (!filtcon_reg)
	{
		return -AD7172ERR;
	}

	/* Clear the ODR bits, configure the requested ODR */
	filtcon_reg->value &= ~(AD717x_ODR_MSK);
	filtcon_reg->value |= AD717X_FILT_CONF_REG_ODR(odr_sel);

	ret = AD717X_WriteRegister(dev, AD717X_FILTCON0_REG + filtcon_id);
	if (ret)
	{
		return ret;
	}

	return 0;
}

/*************************************************************
** Function name:       ad717x_set_data_state
** Descriptions:        设置芯片是否打开DATA_STATE位
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
int32_t ad717x_set_data_state(PAD7172_Struct dev,
	uint8_t dataState)
{

	ad717x_st_reg* ifmode_reg;

	/* Retrieve the SETUPCON Register */
	ifmode_reg = AD717X_GetReg(dev, AD717X_IFMODE_REG);
	if (!ifmode_reg)
		return -AD7172ERR;

	if (dataState)
	{
		// Enable input buffer for the chosen set up
		ifmode_reg->value |= AD717X_IFMODE_REG_DATA_STAT;
		ifmode_reg->value |= AD717X_IFMODE_REG_CONT_READ;
	}
	else
	{
		ifmode_reg->value &= ~(AD717X_IFMODE_REG_DATA_STAT);
		ifmode_reg->value &= ~(AD717X_IFMODE_REG_CONT_READ);
	}
	if (AD717X_WriteRegister(dev, AD717X_IFMODE_REG) < 0)
		return -AD7172ERR;

	return 0;
}

/***************************************************************************/ /**
																			   * @brief Initializes the AD717X.
																			   *
																			   * @param device     - The device structure.
																			   * @param init_param - The structure that contains the device initial
																			   * 		       parameters.
																			   *
																			   * @return Returns 0 for success or negative error code.
																			   *******************************************************************************/
int32_t AD717X_Init(PAD7172_Struct dev_7172)
{
	int32_t ret;
	ad717x_st_reg* preg;
	uint8_t setup_index;
	uint8_t ch_index;

	//赋值preg
	AD717X_LoadRegsPtr(dev_7172);
	//开始初始化
	dev_7172->SetCsPin(0);//开启操作 拉低信号线
	/*  Reset the device interface.*/
	ret = AD717X_Reset(dev_7172);
	HAL_Delay(1);//延时一会
	if (ret < 0)
		return ret;
	/* Initialize ADC mode register. */
	ret = AD717X_WriteRegister(dev_7172, AD717X_ADCMODE_REG);
	if (ret < 0)
		return ret;

	/* Initialize Interface mode register. */
	ret = AD717X_WriteRegister(dev_7172, AD717X_IFMODE_REG);
	if (ret < 0)
		return ret;

	/* Get CRC State */
	ret = AD717X_UpdateCRCSetting(dev_7172);
	if (ret < 0)
		return ret;

	/* Initialize registers AD717X_GPIOCON_REG through AD717X_OFFSET0_REG */
	preg = AD717X_GetReg(dev_7172, AD717X_GPIOCON_REG);
	if (!preg)
		return INVALID_VAL;

	while (preg && preg->addr != AD717X_OFFSET0_REG)
	{
		if (preg->addr == AD717X_ID_REG)//ID寄存器是一个只读寄存器
		{
			preg++;
			continue;
		}

		ret = AD717X_WriteRegister(dev_7172, preg->addr);//将对应的值写入寄存器
		if (ret < 0)
			break;
		preg++;
	}
	/* Read ID register to identify the part */
	ret = AD717X_ReadRegister(dev_7172, AD717X_ID_REG);
	// 可以判断是否通讯成功 通过读取id寄存器
	preg = AD717X_GetReg(dev_7172, AD717X_ID_REG);
	if ((preg->value & 0xF0) == 0XD0)
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, LED_ON);
	}

	for (setup_index = 0; setup_index < dev_7172->num_channels; setup_index++)
	{
		/* Set Polarity */
		ret = ad717x_set_polarity(dev_7172, dev_7172->setups[setup_index].bi_unipolar,
			setup_index);
		if (ret < 0)
			return ret;

		/* Select the reference source */
		ret = ad717x_set_reference_source(dev_7172,
			dev_7172->setups[setup_index].ref_source, setup_index);
		if (ret < 0)
			return ret;

		/* Enable reference and input buffers */
		/* AD7172 应该用不到 */
		ret = ad717x_enable_input_buffer(dev_7172,
			dev_7172->setups[setup_index].input_buff,
			dev_7172->setups[setup_index].ref_buff,
			setup_index);
		if (ret < 0)
			return ret;
		/* 输出速率 */
		ret = ad717x_configure_device_odr(dev_7172, setup_index,
			dev_7172->filter_configuration[setup_index].odr);
		if (ret < 0)
			return ret;
		ret = AD717X_Configure_FilterOrder(dev_7172, setup_index,
			dev_7172->filter_configuration[setup_index].oder);
	}
	/* Set Conversion Mode */
	ret = ad717x_set_adc_mode(dev_7172, dev_7172->mode);
	if (ret < 0)
		return ret;

	/*  Connect Analog Inputs, Assign Setup, Disable all channels */
	for (ch_index = 0; ch_index < dev_7172->num_channels; ch_index++)
	{
		ret = ad717x_connect_analog_input(dev_7172, ch_index,
			dev_7172->chan_map[ch_index].analog_inputs);
		if (ret < 0)
			return ret;

		ret = ad717x_assign_setup(dev_7172, ch_index,
			dev_7172->chan_map[ch_index].setup_sel);
		if (ret < 0)
			return ret;

		ret = ad717x_set_channel_status(dev_7172, ch_index,
			dev_7172->chan_map[ch_index].channel_enable);
		if (ret < 0)
			return ret;
	}
	//读取多个寄存器的值
	AD717X_ReadRegs(dev_7172);

	// 设置ifmode寄存器 连续读取+状态数据叠加
	// ad717x_set_data_state(dev_7172, true);TODO  为了能够进入设置配置模式 暂时关闭连续读取 此时对其他指令有响应

	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////



	//为了使DOUT引脚被复用为RDY，手动将片选拉高
	dev_7172->SetCsPin(1);

	return ret;
}

/***************************************************************************/ /**
																			   * @brief Free the resources allocated by AD717X_Init().
																			   * @param dev - The device structure.
																			   * @return 0 in case of success, negative error code otherwise.
																			   *******************************************************************************/
int32_t AD717X_remove(PAD7172_Struct dev)
{
	int32_t ret;

	// ret = no_os_spi_remove(dev->spi_desc);

	// no_os_free(dev);

	return ret;
}

/*************************************************************
** Function name:       AD717X_OnlyRead32
** Descriptions:        连续转换模式下只读数据 32位(当状态寄存器叠加的时候位32位，否则24位)
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
int32_t AD717X_OnlyRead32(PAD7172_Struct dev)
{
	int32_t value = 0;
	uint8_t reBuff[4] = { 0 };
	uint8_t tempBuff[4] = { 0 };
	// 使得DOUT输出为RDY数据 此时若为低电平 则说明转换完成
	dev->SetCsPin(1);//根据DOUT_RESET位的定义 需要拉高CS使得此时DOUT输出为RDY信息
	HAL_Delay(1);
	dev->SetCsPin(0);

	// 判断DOUT  MISO引脚电平 如果为低电平则读取信号 高电平则跳过
	// 或者使用中断下降沿读取信号
	if (dev->GetDinPin() == 0)
	{
		dev->spi_receive(reBuff, 4);
		tempBuff[0] = reBuff[2];
		tempBuff[1] = reBuff[1];
		tempBuff[2] = reBuff[0];
		memcpy(&value, tempBuff, 3);
		uint8_t recID = reBuff[3] & 0x03; // Mask with 0x03 to get bits 0 and 1
		convertedVal[recID] = value / ((1 << 24) - 1) * REF_POS_Voltage; //直接存储对应数值
		if (recID == 0x00)
		{
			dev->adcValue[0] = value;
		}
		if (recID == 0x01)
		{
			dev->adcValue[1] = value;
		}
		if (recID == 0x02)
		{
			dev->adcValue[2] = value;
		}
		if (recID == 0x03)
		{
			dev->adcValue[3] = value;
		}
	}
	else
	{
		dev->SetCsPin(1);//拉高CS使得此时DOUT输出为RDY信息
		return -1;
	}
	dev->SetCsPin(1);//拉高CS使得此时DOUT输出为RDY信息
	return 0;
}

/*************************************************************
** Function name:       AD717X_GetChannelValue
** Descriptions:        获取AD读取的数据
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
int32_t AD717X_GetChannelValue(PAD7172_Struct dev, uint8_t channel)
{
	if (channel >= 4)
	{
		return -1;
	}
	return dev->adcValue[channel];
}


/// @brief 配置滤波器寄存器的滤波器选择
/// @param dev  句柄
/// @param filter_orderSel  选择哪个滤波器 0代表sinc5+sinc1(默认)  3代表sinc3
/// @param filcon_id       滤波器寄存器的ID 0-3
/// @return 
int32_t AD717X_Configure_FilterOrder(PAD7172_Struct dev,
	uint8_t filcon_id,
	uint8_t filter_orderSel)
{
	ad717x_st_reg* filtcon_reg;
	int32_t ret;

	/* Retrieve the FILTCON register */
	filtcon_reg = AD717X_GetReg(dev, AD717X_FILTCON0_REG + filcon_id);
	if (!filtcon_reg)
	{
		return -AD7172ERR;
	}

	/* Clear the ODRER bits, configure the requested ODRER */
	filtcon_reg->value &= ~(AD717X_FLT_ORDER_MSK);
	filtcon_reg->value |= AD717X_FILT_CONF_REG_ORDER(filter_orderSel);
	ret = AD717X_WriteRegister(dev, AD717X_FILTCON0_REG);
	if (ret < 0)
	{
		return ret;
	}
	return 0;
}






/**
 * @description: 为寄存器指针赋值，可以加入监视器
 * @param {PAD7172_Struct} dev 句柄
 * @return {*}
 */
void AD717X_LoadRegsPtr(PAD7172_Struct dev)
{
	ID_preg = AD717X_GetReg(dev, AD717X_ID_REG);
	CH_map0_preg = AD717X_GetReg(dev, AD717X_CHMAP0_REG);
	CH_map1_preg = AD717X_GetReg(dev, AD717X_CHMAP1_REG);
	CH_map2_preg = AD717X_GetReg(dev, AD717X_CHMAP2_REG);
	CH_map3_Preg = AD717X_GetReg(dev, AD717X_CHMAP3_REG);
	ADCMODE_preg = AD717X_GetReg(dev, AD717X_ADCMODE_REG);
	IFMODE_preg = AD717X_GetReg(dev, AD717X_IFMODE_REG);
	SETUPCON0_preg = AD717X_GetReg(dev, AD717X_SETUPCON0_REG);
	FILTCON0_preg = AD717X_GetReg(dev, AD717X_FILTCON0_REG);
	OFFSET0_preg = AD717X_GetReg(dev, AD717X_OFFSET0_REG);
	GAIN0_preg = AD717X_GetReg(dev, AD717X_GAIN0_REG);
}
/**
 * @description: 读取所有需要的寄存器
 * @param {PAD7172_Struct} dev
 * @return {*}
 */
void AD717X_ReadRegs(PAD7172_Struct dev)
{
	AD717X_ReadRegister(dev, AD717X_ID_REG);
	AD717X_ReadRegister(dev, AD717X_CHMAP0_REG);
	AD717X_ReadRegister(dev, AD717X_CHMAP1_REG);
	AD717X_ReadRegister(dev, AD717X_CHMAP2_REG);
	AD717X_ReadRegister(dev, AD717X_CHMAP3_REG);
	AD717X_ReadRegister(dev, AD717X_ADCMODE_REG);
	AD717X_ReadRegister(dev, AD717X_IFMODE_REG);
	AD717X_ReadRegister(dev, AD717X_SETUPCON0_REG);
	AD717X_ReadRegister(dev, AD717X_FILTCON0_REG);
	AD717X_ReadRegister(dev, AD717X_OFFSET0_REG);
	AD717X_ReadRegister(dev, AD717X_GAIN0_REG);
}

