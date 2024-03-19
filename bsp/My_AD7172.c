#include "My_AD7172.h"


#include "Ad7172Spi.h"
#include "ad7172_2_regs.h"
#include "ad717x_Frame.h"
#include "gpio.h"
#include "usart.h"
#include "math.h"
#include "stdio.h"
#include "main.h"


#define REF_Voltage 4.98 ; // 参考电压
#define Negetive_REF_Voltage -5.0 // 参考电压


AD7172_Struct gAd7172; // AD7172 structure句柄

void AD7172ParmInit(void) {
    gAd7172.regs = ad7172_2_regs;// 定义有哪些寄存器及其初始值
    gAd7172.num_regs = sizeof(ad7176_2_regs) / sizeof(ad7176_2_regs[0]);
    gAd7172.active_device = ID_AD7172_2;// 设备ID
    gAd7172.num_channels = 1;// TODO 在这里设置通道数为1
    // 转化模式设置
    gAd7172.mode = CONTINUOUS;

    // 设置极性 false 为单极性 Ture 为双极性
    gAd7172.setups[0].bi_unipolar = 0; //我只需要一个通道
    gAd7172.setups[1].bi_unipolar = 0;
    gAd7172.setups[2].bi_unipolar = 0;
    gAd7172.setups[3].bi_unipolar = 0;
    // 设置参考源
    gAd7172.setups[0].ref_source = EXTERNAL_REF;//外部参考源
    gAd7172.setups[1].ref_source = EXTERNAL_REF;
    gAd7172.setups[2].ref_source = EXTERNAL_REF;
    gAd7172.setups[3].ref_source = EXTERNAL_REF;
    // 设置buff 7176不需要
    // gAd7172.setups[0].ref_buff = 0;
    // gAd7172.setups[0].input_buff = 0;

    // 设置 滤波和速率 不使用增强滤波器的话 无需使用
    // gAd7172.filter_configuration[0].enhfilt =
    // 输出速率
    gAd7172.filter_configuration[0].odr = sps_5;
    gAd7172.filter_configuration[1].odr = sps_5;
    gAd7172.filter_configuration[2].odr = sps_5;
    gAd7172.filter_configuration[3].odr = sps_5;
    // 滤波器类型
    gAd7172.filter_configuration[0].oder = sinc5_sinc1;
    gAd7172.filter_configuration[1].oder = sinc5_sinc1;
    gAd7172.filter_configuration[2].oder = sinc5_sinc1;
    gAd7172.filter_configuration[3].oder = sinc5_sinc1;
    //TODO  根据表
    // 设置通道映射寄存器 选择当前有效的通道、各通道使用哪些输入以及该通道使用何种设置来配置ADC。
    gAd7172.chan_map[0].analog_inputs.ainp.pos_analog_input = AIN0;
    gAd7172.chan_map[0].analog_inputs.ainp.neg_analog_input = REF_M;

    gAd7172.chan_map[1].analog_inputs.ainp.pos_analog_input = AIN1;
    gAd7172.chan_map[1].analog_inputs.ainp.neg_analog_input = REF_M;

    gAd7172.chan_map[2].analog_inputs.ainp.pos_analog_input = AIN2;
    gAd7172.chan_map[2].analog_inputs.ainp.neg_analog_input = REF_M;

    gAd7172.chan_map[3].analog_inputs.ainp.pos_analog_input = AIN3;
    gAd7172.chan_map[3].analog_inputs.ainp.neg_analog_input = REF_M;

    // 使用四种设置中的哪一种来配置ADC。设置由四个寄存器组成：设置配置寄存器、滤波器配置寄存器、失调寄存器和增益寄存器。
    gAd7172.chan_map[0].setup_sel = 0;
    gAd7172.chan_map[1].setup_sel = 0;
    gAd7172.chan_map[2].setup_sel = 0;
    gAd7172.chan_map[3].setup_sel = 0;
    //  1为开 0为关闭
    gAd7172.chan_map[0].channel_enable = 1;
    gAd7172.chan_map[1].channel_enable = 0;
    gAd7172.chan_map[2].channel_enable = 0;
    gAd7172.chan_map[3].channel_enable = 0;


    gAd7172.spi_write_and_read = stm32_spi_write_and_read;
    gAd7172.SetCsPin = SetAD7172CsPin;
    gAd7172.spi_receive = stm32_spi_receive;
    gAd7172.GetDinPin = spi_miso_input;

    gAd7172.SetCsPin(1);
    AD717X_Init(&gAd7172);

}

/*************************************************************
** Function name:       AD7172Loop
** Descriptions:        读取ADC数据的主循环函数 需要根据ADC的配置来 循环调用
** Input parameters:    None
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
void AD7172Loop(void) {
    if (AD717X_OnlyRead32(&gAd7172) == 0)
    {
        //数据存放在gAd7172.的adcValue数组中
        uint32_t adcValue = gAd7172.adcValue[0];
        // Convert the 24-bit ADC value to a voltage reading
        double voltage = ((double)adcValue / (1 << 24)) * REF_Voltage;
        //        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
                // Print the voltage reading with maximum precision
        printf("/*%.15f,%d*/\n", voltage, 0);
        //        HAL_UART_Transmit(&huart1, (uint8_t*)"test\n", 5, 0XFF);
    }
    else
        printf("Error in reading\n");
    //没进入上述循环说明数据没有准备好
}


/*************************************************************
** Function name:       GetAD7172ADCChannel
** Descriptions:        提供ADC数据的接口
** Input parameters:    channel: 要获取的通道 0:ADC0 1:ADC1 2:ADC2 3:ADC
** Output parameters:   None
** Returned value:      None
** Remarks:             None
*************************************************************/
int32_t GetAD7172ADCChannel(uint8_t channel) {
    return AD717X_GetChannelValue(&gAd7172, channel);
}


/**
 * @brief 获取AD7172的ID 用于判断此时是否连接了AD7172 验证通讯
 *
 * @return uint8_t
 */
uint8_t ReadAD7172_ID(void)
{
    AD717X_ReadRegister(&gAd7172, AD717X_ID_REG);//更新ID 寄存器的值
    return 0;
}
/**
 * @description: 进行ADC芯片的校准
 * @param {enum ad717x_mode } calib_mode 参考枚举的定义
 * @return {*} 0 代表正常
 */
uint8_t AD7172_Calib(enum ad717x_mode  calib_mode)
{
#define CHANEL_NUM 1 //只对一个通道进行
    gAd7172.SetCsPin(0);//拉低片选
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, LED_OFF);//提示正在校准
    enum ad717x_mode oldMode = gAd7172.mode;//保存旧模式
    //关闭所有通道
    for (uint8_t i = 0; i < CHANEL_NUM; i++)
    {
        ad717x_set_channel_status(&gAd7172, i, 0);
        //读取该寄存器的值
        AD717X_ReadRegister(&gAd7172, AD717X_CHMAP0_REG + i);
    }
    //单独设置开启某个通道 //分别对每个通道进行校准
    for (uint8_t i = 0;i < CHANEL_NUM;i++)
    {
        printf("perfromance calib to channel %d\n", i);
        ad717x_set_channel_status(&gAd7172, i, 1);//开启通道
        HAL_Delay(1);//确保通道已经开启
        AD717X_ReadRegister(&gAd7172, AD717X_CHMAP0_REG + i);//读取通道映射寄存器
        gAd7172.mode = calib_mode;//选择校准模式
        ad717x_set_adc_mode(&gAd7172, gAd7172.mode);//设置为校准模式
        gAd7172.SetCsPin(0);//拉低片选使得转换后RDY会输出低电平
        //监视引脚信号
        while (gAd7172.GetDinPin() == 1);//高电平说明在校准，低电平表示校准结束
        AD717X_ReadRegister(&gAd7172, AD717X_ADCMODE_REG);
        uint8_t bits_4_to_6 = ((AD717X_GetReg(&gAd7172, AD717X_ADCMODE_REG)->value) >> 4) & 0x07;
        printf("ADC Mode Register Val: 0x%x\n", bits_4_to_6);//查看数据手册的对照表 
        ad717x_set_channel_status(&gAd7172, i, 0);//校准完毕关闭通道
        AD717X_ReadRegister(&gAd7172, AD717X_CHMAP0_REG + i);//读取通道映射寄存器
        printf("channel %d calib done \n", i);
    }
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, LED_ON);
    //复原原先的模式
    gAd7172.mode = oldMode;
    ad717x_set_adc_mode(&gAd7172, gAd7172.mode);
    AD717X_ReadRegister(&gAd7172, AD717X_ADCMODE_REG);
    printf("current ADC mode = %02x \n", ((AD717X_GetReg(&gAd7172, AD717X_ADCMODE_REG)->value) >> 4) & 0x07);

    //只开启通道零
    AD717X_GetReg(&gAd7172, AD717X_CHMAP0_REG + 0)->value = 0;
    AD717X_GetReg(&gAd7172, AD717X_OFFSET0_REG + 0)->value = 0;
    AD717X_GetReg(&gAd7172, AD717X_GAIN0_REG + 0)->value = 0;
    AD717X_ReadRegister(&gAd7172, AD717X_OFFSET0_REG);
    AD717X_ReadRegister(&gAd7172, AD717X_GAIN0_REG);
    ad717x_set_channel_status(&gAd7172, 0, 1);
    AD717X_ReadRegister(&gAd7172, AD717X_CHMAP0_REG + 0);
    gAd7172.SetCsPin(1);//拉高片选
    printf("校准结束\n");
    return 0;
}

/**
 * @description: 用于调试的函数
 * @param {*}  None
 * @return {*} None
 */
void AD7172_DebugFunction(void)
{
    AD717X_ReadRegister(&gAd7172, AD717X_ADCMODE_REG);
    int32_t adcmode_val = AD717X_GetReg(&gAd7172, AD717X_ADCMODE_REG)->value;
    printf("ADCMODE Register Value: 0X%04x\n", adcmode_val);
    AD717X_ReadRegister(&gAd7172, AD717X_IFMODE_REG);
    printf("IFMODE Register Value: 0X%04x\n", AD717X_GetReg(&gAd7172, AD717X_IFMODE_REG)->value);
}

