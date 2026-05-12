
#include <string.h>
#include "ti_msp_dl_config.h"
#include "main.h"
#include "stdio.h"
#include "car.h"
#include "pid.h"
#include "Blue_teeth.h"
#include "Grayscale_Sensor.h"
#include "bsp_usart.h"
#include "servo.h"
#include "kubian_ditect.h"
#include "k210_use.h"
 /*pid滤波*/
#define factorp 0.9
#define factori 0.0
#define factord 0.002
#define filter_factor 0.7

/*小车速度*/
#define SPEED_BASE 0
#define forward 0
#define backward 1
/*灰度*/
#define BLACK 500
#define WHITE 3200

uint8_t oled_buffer[32],SP_buffer[32];//oled
extern uint16_t RX_NUM;//蓝牙传来的值

volatile uint8_t gEchoData = 0;//串口收
extern char re_str[100];
extern char* stt;
extern float yawdilta_cal_sv,yaw_cal,yaw_base_cal,yawdilta_cal;
extern int angle_turn;//蓝牙传出的值 偏向角PIDmethod
extern int Blue_teeth_Kp;//蓝牙接收的值
extern int left_speed,right_speed;//装载电机左右轮的速度

float  yaw_base = 0.0,yaw_exp = 0.0,yaw_bias = 31.0;//角度 speed_dita
int ditect=1;
//角度pid,要开启MPU6050
    pid_t angle_pid_instance;
    pid_t *angle_pid = &angle_pid_instance;


float speed_exp = 0, speed_dita= 0;//速度pid
int pwm_left,pwm_right;//速度pid输出给电机
float expect_flag = 0,yaw_last = 100.0;//确定基准角
int routflag = 0;//FLAG
extern int flag_circle,flag_stop,flag_judge,special_case_flag;//FLAG
int L_Distance,R_Distance,L_Encoder_speed,R_Encoder_speed,L_Distance_last,R_Distance_last,encoder_dis;//编码器速度和距离
uint8_t ch;//串口重定向
uint16_t disVal;//超声波距离distance value
msg_k210 k210_msg;//k210数字识别的结构体返回值
int L_x,L_y,C_x,C_y,pid_up_angle,pid_down_angle,pid_down_angle_stop;
long long int circles;
/*八路灰度*/
unsigned short Anolog[8]={0};
// unsigned short black[8]={600,600,500,500,600,600,500,500};
unsigned short white[8]={WHITE,WHITE,WHITE,WHITE,WHITE,WHITE,WHITE,WHITE};
unsigned short black[8]={BLACK,BLACK,BLACK,BLACK,BLACK,BLACK,BLACK,BLACK};
unsigned short Normal[8];
int main(void)

{
    SYSCFG_DL_init();
    SysTick_Init();//滴答时钟
    NVIC_EnableIRQ(SysTick_IRQn);
    DL_SYSTICK_enableInterrupt(); 
    //DL_TimerG_startCounter(PWM_0_INST);//电机速度PWM模块
    DL_TimerG_startCounter(servo_INST);//伺服电机模块
    MPU6050_Init();
    OLED_Init();//OLED屏幕
    // Ultrasonic_Init();//超声波模块
    //pid_init( angle_pid, DELTA_PID, factorp, factori, factord);//DELTA_PID、POSITION_PID可修改,pid模块初始化

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);//UART2串口
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    // NVIC_ClearPendingIRQ(MYUART_INST_INT_IRQN);//uart1串口k210模块
	// NVIC_EnableIRQ(MYUART_INST_INT_IRQN);

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);//MPU6050INTB1口触发中断和编码电机中断PortB
    NVIC_EnableIRQ(TIMER_Encoder_speed_INST_INT_IRQN);//定时器100ms触发Zero_event
    /*八路灰度初始化*/
    // No_MCU_Sensor sensor;
	// unsigned char Digtal;
    // No_MCU_Ganv_Sensor_Init(&sensor,white,black);

    // OLED_ShowString(0,0,(uint8_t *)"L_x",8);
    // OLED_ShowString(0,2,(uint8_t *)"L_y",8);
    // OLED_ShowString(0,4,(uint8_t *)"C_x",8);
    // OLED_ShowString(0,6,(uint8_t *)"C_y",8);

//设定初始位置
    DL_GPIO_setPins(lazer_PORT, lazer_PIN_PIN);
    Servo_SetDownAngle(0);
    while (1) 
    {
          /*等待解决MPU6050零漂问题*/
            while(expect_flag<=4)
        {
            sprintf((char *)oled_buffer, "%6.2f    ", yaw);
            OLED_ShowString(15*6,0,oled_buffer,8);
            sprintf((char *)oled_buffer, "%6.2f    ", yaw_base);
            OLED_ShowString(15*6,1,oled_buffer,8);
                    
            if((yaw-yaw_last)<0.001 && (yaw-yaw_last)>-0.001)
            {
                yaw_base = yaw;
                expect_flag ++;
                yaw_last = yaw;
            }
            else
            {
                yaw_last = yaw;       
            }
            delay_cycles(8000000);
        }

        /*打印Normal的值*/

         sprintf((char *)oled_buffer, "%6.2f    ", yaw);
         OLED_ShowString(15*6,0,oled_buffer,8);
         sprintf((char *)oled_buffer, "%6.2f    ", yaw_base);
         OLED_ShowString(15*6,1,oled_buffer,8);
         sprintf((char *)oled_buffer, "%d    ", k210_msg.w);
         OLED_ShowString(15*6,5,oled_buffer,8);
         sprintf((char *)oled_buffer, "%d    ", k210_msg.h);
         OLED_ShowString(15*6,6,oled_buffer,8); 
         sprintf((char *)oled_buffer, "%d    ", pid_down_angle);
         OLED_ShowString(15*6,7,oled_buffer,8);
         sprintf((char *)oled_buffer, "%6.2f  ", yawdilta_cal_sv);
         OLED_ShowString(3*6,0,oled_buffer,8);
         sprintf((char *)oled_buffer, "%6.2f  ", yawdilta_cal_sv+pid_down_angle);
         OLED_ShowString(3*6,1,oled_buffer,8);
         sprintf((char *)oled_buffer, "%d  ", ditect);
         OLED_ShowString(3*6,2,oled_buffer,8);
        sprintf((char *)oled_buffer, "%d  ", !!DL_GPIO_readPins(GPIO_ditect_PORT,GPIO_ditect_PIN_ditect_PIN));
        OLED_ShowString(3*6,3,oled_buffer,8);

    } 
}
void TIMER_Encoder_speed_INST_IRQHandler(void)// 100ms定时器，发送蓝牙
{
    switch(DL_TimerG_getPendingInterrupt(TIMER_Encoder_speed_INST))
    {
        case DL_TIMER_IIDX_ZERO:

        if(k210_msg.w !=0 ||k210_msg.h !=0)//复位完毕
        {
            L_x = 98;
            L_y = 60;
            C_x = k210_msg.w;
            C_y = k210_msg.h;
            pid_down_angle += Position_PID (L_x,C_x);
            if(pid_down_angle>200)
                pid_down_angle = 200;
            else if (pid_down_angle<-200)
                pid_down_angle = -200;
            if(DL_GPIO_readPins(GPIO_ditect_PORT,GPIO_ditect_PIN_ditect_PIN)&&ditect)
            {
                pid_down_angle = 0;
            }
            Servo_SetDownAngle(pid_down_angle+0);//Servo_turn_where_target_is()
            if(k210_msg.w-98>=-4 && k210_msg.w-98<=4 && k210_msg.w!=98 )
            {
                pid_down_angle_stop = pid_down_angle;
                Servo_SetDownAngle(pid_down_angle_stop+0);//Servo_turn_where_target_is()
                DL_GPIO_clearPins(lazer_PORT, lazer_PIN_PIN);

            }
            
        }
            break;
        default:
            break;
    }
}


