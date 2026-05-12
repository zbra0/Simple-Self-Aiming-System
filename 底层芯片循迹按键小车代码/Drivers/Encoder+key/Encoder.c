#include "Encoder.h"
#include "ti_msp_dl_config.h"
#include "oled_software_i2c.h"
#include "stdio.h"
#include "mpu6050.h"
#include "string.h"
/*编码器*/
extern int L_Distance,R_Distance,L_Encoder_speed,R_Encoder_speed,L_Distance_last,R_Distance_last;
extern int expect_circle,start_flag;
void GROUP1_IRQHandler(void)
{
    #if defined GPIO_Encoder_Left_PORT
        #if defined GPIO_Encoder_Right_PORT
    uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Aphase_PIN | GPIO_Encoder_Left_PIN_Bphase_PIN);
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Aphase_r_PIN | GPIO_Encoder_Right_PIN_Bphase_r_PIN|GPIO_Button_circle_PIN|GPIO_Button_start_PIN);
    if((gpioB & GPIO_Encoder_Left_PIN_Aphase_PIN) == GPIO_Encoder_Left_PIN_Aphase_PIN )
    {
        if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Bphase_PIN) == 1)
            L_Distance --;
        else if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Bphase_PIN) == 0)
            L_Distance++;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_Left_PORT, GPIO_Encoder_Left_PIN_Aphase_PIN);

    }
    else if((gpioB & GPIO_Encoder_Left_PIN_Bphase_PIN) == GPIO_Encoder_Left_PIN_Bphase_PIN )
    {
        if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Aphase_PIN) == 0)
            L_Distance --;
        else if(DL_GPIO_readPins(GPIO_Encoder_Left_PORT,GPIO_Encoder_Left_PIN_Aphase_PIN) == 1)
             L_Distance ++;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_Left_PORT, GPIO_Encoder_Left_PIN_Bphase_PIN);

    }
    else if((gpioA & GPIO_Encoder_Right_PIN_Aphase_r_PIN) == GPIO_Encoder_Right_PIN_Aphase_r_PIN )
    {
        if(DL_GPIO_readPins(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Bphase_r_PIN) == 1)
            R_Distance --;
        else if(DL_GPIO_readPins(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Bphase_r_PIN) == 0)
            R_Distance++;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_Right_PORT, GPIO_Encoder_Right_PIN_Aphase_r_PIN);
    }
    else if((gpioA & GPIO_Encoder_Right_PIN_Bphase_r_PIN) == GPIO_Encoder_Right_PIN_Bphase_r_PIN )
    {
        if(DL_GPIO_readPins(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Aphase_r_PIN) == 0)
            R_Distance --;
        else if(DL_GPIO_readPins(GPIO_Encoder_Right_PORT,GPIO_Encoder_Right_PIN_Aphase_r_PIN) == 1)
            R_Distance ++;
        DL_GPIO_clearInterruptStatus(GPIO_Encoder_Right_PORT, GPIO_Encoder_Right_PIN_Bphase_r_PIN);
    }
    #endif
        #endif
    else if((gpioA & GPIO_Button_circle_PIN) == GPIO_Button_circle_PIN )
    {

        if(DL_GPIO_readPins(GPIO_Button_PORT,GPIO_Button_circle_PIN))
        {

            delay_cycles(8000000*5);
            if(DL_GPIO_readPins(GPIO_Button_PORT,GPIO_Button_circle_PIN))
            {
                while(DL_GPIO_readPins(GPIO_Button_PORT,GPIO_Button_circle_PIN));
                expect_circle=(expect_circle+4);
                expect_circle=expect_circle>21?expect_circle-20:expect_circle;
            }
        }
        DL_GPIO_clearInterruptStatus(GPIO_Button_PORT, GPIO_Button_circle_PIN);
    }
    else if((gpioA & GPIO_Button_start_PIN) == GPIO_Button_start_PIN )
    {
        if(DL_GPIO_readPins(GPIO_Button_PORT,GPIO_Button_start_PIN))
        {
            delay_cycles(8000000*5);
            if(DL_GPIO_readPins(GPIO_Button_PORT,GPIO_Button_start_PIN))
            {
                while(DL_GPIO_readPins(GPIO_Button_PORT,GPIO_Button_start_PIN));
                start_flag = 1;
                DL_GPIO_disableInterrupt(GPIO_Button_PORT, GPIO_Button_start_PIN);
                DL_GPIO_disableInterrupt(GPIO_Button_PORT, GPIO_Button_circle_PIN);
            }
        }
        DL_GPIO_clearInterruptStatus(GPIO_Button_PORT, GPIO_Button_start_PIN);
    }

   if(DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)== GPIO_MULTIPLE_GPIOB_INT_IIDX)

   {
        Read_Quad();
   }
}
