#include "Blue_teeth.h"
#include "ti_msp_dl_config.h"
#include "oled_software_i2c.h"
#include "stdio.h"
#include "string.h"
#include "pid.h"
#include "k210_use.h"
extern volatile uint8_t gEchoData;
volatile unsigned char uart_data = 0;
char re_str[50];//用来存一组数据
char* stt = re_str;//作为str的指针
uint16_t RX_NUM = 0;//得到蓝牙传的值
extern float Blue_teeth_Kp;
void UART_0_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        // case DL_UART_MAIN_IIDX_RX:
        //     gEchoData = DL_UART_Main_receiveData(UART_0_INST);
        //     *(stt++) = (char)(gEchoData);
        //     if(gEchoData == 10)//扫到回车，把re_str的数据转换为RX_NUM
        //     {
        //         for(int i = 0;i<stt-re_str-1;i++)
        //         {
        //            RX_NUM = RX_NUM*10+(re_str[i]-48);//‘1’的ask是49
        //         }
        //         //OLED_Clear();//清屏
        //         //OLED_ShowNum(40, 8, RX_NUM,stt- re_str, 16);//显示RX_NUM
        //         memset(re_str,0,sizeof(re_str));//清空re_str字符数组
        //         stt = re_str;//对齐指针
        //         Blue_teeth_Kp =RX_NUM;
        //         RX_NUM = 0;//清零
        //    }
            //DL_UART_Main_transmitData(UART_0_INST, gEchoData);
        uart_data = DL_UART_Main_receiveData(UART_0_INST);
 		recv_k210msg(uart_data); 

            break;
        default:
            break;
    }
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
}