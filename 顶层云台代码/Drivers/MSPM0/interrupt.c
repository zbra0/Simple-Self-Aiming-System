#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "mpu6050.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "main.h"
extern uint8_t Encoder_L_x,Encoder_L_y,Encoder_L_z;
extern int L_Distance,R_Distance;
int PA,PB;
extern uint8_t SP_buffer[32];
void SysTick_Handler(void)
{
    tick_ms++;
    // if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT, GPIO_Encoder_Left_PIN_Aphase_PIN))
    //     PA = 1;
    // else
    //     PA = 0;
    // if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT, GPIO_Encoder_Left_PIN_Bphase_PIN))
    //     PB = 1+2;
    // else
    //     PB = 0+2;
    // sprintf((char *)SP_buffer, "%d", PA);
    // printf("%s,",SP_buffer);
    // sprintf((char *)SP_buffer, "%d", PB);
    // printf("%s\r\n",SP_buffer);
}

#if defined UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;
    extern uint8_t bno08x_dmaBuffer[19];

    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false)
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);

    for(int i=2; i<=14; i++)
        checkSum += bno08x_dmaBuffer[i];

    if((rxSize == 19) && (bno08x_dmaBuffer[0] == 0xAA) && (bno08x_dmaBuffer[1] == 0xAA) && (checkSum == bno08x_dmaBuffer[18]))
    {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (int16_t)((bno08x_dmaBuffer[4]<<8)|bno08x_dmaBuffer[3]) / 100.0;
        bno08x_data.pitch = (int16_t)((bno08x_dmaBuffer[6]<<8)|bno08x_dmaBuffer[5]) / 100.0;
        bno08x_data.roll = (int16_t)((bno08x_dmaBuffer[8]<<8)|bno08x_dmaBuffer[7]) / 100.0;
        bno08x_data.ax = (bno08x_dmaBuffer[10]<<8)|bno08x_dmaBuffer[9];
        bno08x_data.ay = (bno08x_dmaBuffer[12]<<8)|bno08x_dmaBuffer[11];
        bno08x_data.az = (bno08x_dmaBuffer[14]<<8)|bno08x_dmaBuffer[13];
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID, (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}
#endif

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while(rxSize >= 11)
    {
        checkSum=0;
        for(int i=packCnt*11; i<(packCnt+1)*11-1; i++)
            checkSum += wit_dmaBuffer[i];

        if((wit_dmaBuffer[packCnt*11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt*11+10]))
        {
            if(wit_dmaBuffer[packCnt*11+1] == 0x51)
            {
                wit_data.ax = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 2.048; //mg
                wit_data.ay = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 2.048; //mg
                wit_data.az = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 2.048; //mg
                wit_data.temperature =  (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]) / 100.0; //°C
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x52)
            {
                wit_data.gx = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 16.384; //°/S
                wit_data.gy = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 16.384; //°/S
                wit_data.gz = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 16.384; //°/S
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x53)
            {
                wit_data.roll  = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 32768.0 * 180.0; //°
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 32768.0 * 180.0; //°
                wit_data.yaw   = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 32768.0 * 180.0; //°
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif

void GROUP1_IRQHandler(void)
{
    uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Aphase_PIN | GPIO_Encoder_Left_PIN_Bphase_PIN);
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Aphase_r_PIN | GPIO_Encoder_Right_PIN_Bphase_r_PIN);
    if((gpioB & GPIO_Encoder_Left_PIN_Aphase_PIN) == GPIO_Encoder_Left_PIN_Aphase_PIN )
    {
        if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Bphase_PIN) == 1)
            L_Distance ++;
        else if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Bphase_PIN) == 0)
            L_Distance--;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_Left_PORT, GPIO_Encoder_Left_PIN_Aphase_PIN);

    }
    else if((gpioB & GPIO_Encoder_Left_PIN_Bphase_PIN) == GPIO_Encoder_Left_PIN_Bphase_PIN )
    {
        if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Aphase_PIN) == 0)
            L_Distance ++;
        else if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Aphase_PIN) == 1)
             L_Distance --;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_Left_PORT, GPIO_Encoder_Left_PIN_Bphase_PIN);

    }
    else if((gpioA & GPIO_Encoder_Right_PIN_Aphase_r_PIN) == GPIO_Encoder_Right_PIN_Aphase_r_PIN )
    {
        if(DL_GPIO_readPins(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Bphase_r_PIN) == 1)
            R_Distance ++;
        else if(DL_GPIO_readPins(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Bphase_r_PIN) == 0)
            R_Distance--;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_Right_PORT, GPIO_Encoder_Right_PIN_Aphase_r_PIN);
    }
    else if((gpioA & GPIO_Encoder_Right_PIN_Bphase_r_PIN) == GPIO_Encoder_Right_PIN_Bphase_r_PIN )
    {
        if(DL_GPIO_readPins(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Aphase_r_PIN) == 0)
            R_Distance ++;
        else if(DL_GPIO_readPins(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Aphase_r_PIN) == 1)
            R_Distance --;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_Right_PORT, GPIO_Encoder_Right_PIN_Bphase_r_PIN);
    }
    else
    {
        switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        /* MPU6050 INT */
        #if defined GPIO_MPU6050_PORT
            #if defined GPIO_MPU6050_INT_IIDX
            case GPIO_MPU6050_INT_IIDX:
            #elif (GPIO_MPU6050_PORT == GPIOA) && (defined GPIO_MULTIPLE_GPIOA_INT_IIDX)
            case GPIO_MULTIPLE_GPIOA_INT_IIDX:
            #elif (GPIO_MPU6050_PORT == GPIOB) && (defined GPIO_MULTIPLE_GPIOB_INT_IIDX)
            case GPIO_MULTIPLE_GPIOB_INT_IIDX:
            #endif
                Read_Quad();
                break;
        #endif
        }
    }
    

}
