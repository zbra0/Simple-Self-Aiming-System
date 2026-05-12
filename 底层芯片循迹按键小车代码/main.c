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
#include "k210_use.h"
 /*pid滤波*/
#define factorp 0.23
#define factori 0.00
#define factord 0.03
#define filter_factor 0.7

/*小车速度*/
#define SPEED_BASE 19
#define forward 0
#define backward 1
/*灰度*/
#define BLACK 500
#define WHITE 3200

extern float Bias_sv,Last_BiasB_sv,Last2_BiasB_sv;
int run_circle=0,expect_circle=9;//跑圈数量
int start_flag=0;//开始按钮
int flag=0;

uint8_t oled_buffer[32],SP_buffer[32];//oled
extern uint16_t RX_NUM;//蓝牙传来的值

volatile uint8_t gEchoData = 0;//串口收
extern char re_str[100];
extern char* stt;

extern int angle_turn;//蓝牙传出的值 偏向角PIDmethod
extern int Blue_teeth_Kp;//蓝牙接收的值
extern int left_speed,right_speed;//装载电机左右轮的速度

float YAW_D;

extern int bias ;//灰度循迹的差速环
extern int targetA,targetB,Motor_Left,Motor_Right ;
extern float CurrentA , CurrentB ; 
extern int in;


float  yaw_base = 0.0,yaw_exp,yaw_bias = 90.0;//角度 speed_dita

//角度pid,要开启MPU6050
    pid_t angle_pid_instance;
    pid_t *angle_pid = &angle_pid_instance;


float speed_exp = 0, speed_dita= 0;//速度pid
int pwm_left,pwm_right;//速度pid输出给电机
float expect_flag = 0,yaw_last = 100.0;//确定基准角
int routflag = 0,kubian_predict_FLG=0;//FLAG

extern int flag_right,flag_stop,flag_judge,special_case_flag;//FLAG

extern int flag_right;

int L_Distance=1500,R_Distance=1500,L_Encoder_speed,R_Encoder_speed,L_Distance_last,R_Distance_last,encoder_dis;//编码器速度和距离
uint8_t ch;//串口重定向
uint16_t disVal;//超声波距离distance value
msg_k210 k210_msg;//k210数字识别的结构体返回值
int L_x,L_y,C_x,C_y,pid_up_angle,pid_down_angle;
long long int circles;
extern float yawdilta_cal_sv;
/*八路灰度*/
unsigned short Anolog[8]={0};
unsigned short white[8]={WHITE,WHITE,WHITE,WHITE,WHITE,WHITE,WHITE,WHITE};
unsigned short black[8]={BLACK,BLACK,BLACK,BLACK,BLACK,BLACK,BLACK,BLACK};
unsigned short Normal[8];
int main(void)
    {
    SYSCFG_DL_init();
    SysTick_Init();//滴答时钟
    NVIC_EnableIRQ(SysTick_IRQn);
    DL_SYSTICK_enableInterrupt();
    __enable_irq(); 
    DL_TimerG_startCounter(PWM_0_INST);//电机速度PWM模块
    DL_TimerG_startCounter(servo_INST);//伺服电机模块
    MPU6050_Init();
    OLED_Init();//OLED屏幕
    pid_init( angle_pid, POSITION_PID, factorp, factori, factord);//DELTA_PID、POSITION_PID可修改,pid模块初始化

    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);//UART2串口
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);//MPU6050INTB1口触发中断和编码电机中断PortB
    NVIC_EnableIRQ(TIMER_Encoder_speed_INST_INT_IRQN);//定时器100ms触发Zero_event
    /*八路灰度初始化*/
    No_MCU_Sensor sensor;
	unsigned char Digtal;
    No_MCU_Ganv_Sensor_Init(&sensor,white,black);
    DL_GPIO_clearPins(GPIO_Button_PORT,GPIO_Button_circle_PIN);
    DL_GPIO_clearPins(GPIO_Button_PORT,GPIO_Button_start_PIN);    
    while (1) 
    {

        sprintf((char *)oled_buffer, "%d    ", expect_circle);
        OLED_ShowString(3*6,0,oled_buffer,8);
        sprintf((char *)oled_buffer, "%d    ", start_flag);
        OLED_ShowString(3*6,1,oled_buffer,8);        

          /*等待解决MPU6050零漂问题*/
            while(expect_flag<=4)
        {
            sprintf((char *)oled_buffer, "%6.2f    ", yaw);
            OLED_ShowString(10*6,0,oled_buffer,8);
            sprintf((char *)oled_buffer, "%6.2f    ", yaw_base);
            OLED_ShowString(10*6,1,oled_buffer,8);
                    
            if((yaw-yaw_last)<0.001 && (yaw-yaw_last)>-0.001)
            {
                yaw_base = yaw;
                expect_flag ++;
                yaw_exp=yaw_base<0?yaw_base+360:yaw_base;    //0~360exp             
            }
            else
            {
                yaw_last = yaw;
            }
            delay_cycles(8000000);
        }
        

        /*循环八路灰度获取Anolog，Normal，Digital*/
        No_Mcu_Ganv_Sensor_Task_Without_tick(&sensor);
		Digtal=Get_Digtal_For_User(&sensor);
		Get_Anolog_Value(&sensor,Anolog);
		Get_Normalize_For_User(&sensor,Normal);
		delay_ms(1);

        if(start_flag)
        {
           track_line();
        }
        //load_pid();

        sprintf((char *)oled_buffer, "%d  ", encoder_dis);
        OLED_ShowString(3*6,3,oled_buffer,8);
        sprintf((char *)oled_buffer, "%4.2f  ", Bias_sv);
        OLED_ShowString(3*6,4,oled_buffer,8);
        sprintf((char *)oled_buffer, "%4.2f   ", Last_BiasB_sv);
        OLED_ShowString(3*6,5,oled_buffer,8);
        sprintf((char *)oled_buffer, "%4.2f   ", Last2_BiasB_sv);
        OLED_ShowString(3*6,6,oled_buffer,8);
        encoder_dis =( L_Distance + R_Distance)/2;//计算编码器均值

    } 
}

/*串口重定向*/
int fputc(int ch, FILE *f) 
{
    uint8_t c = (uint8_t)ch;
    DL_UART_Main_transmitData(UART_0_INST,ch);   // 发送单个字符
    while (DL_UART_isBusy(UART_0_INST));
    return ch;
}
void TIMER_Encoder_speed_INST_IRQHandler(void)// 100ms定时器，发送蓝牙
{
    switch(DL_TimerG_getPendingInterrupt(TIMER_Encoder_speed_INST))
    {
        case DL_TIMER_IIDX_ZERO:
            L_Encoder_speed = (L_Distance-L_Distance_last+15)/3.6;
            R_Encoder_speed = (R_Distance-R_Distance_last+14)/3.6;
            L_Distance_last=L_Distance;
            R_Distance_last=R_Distance;
            break;
        default:
            break;
    }
}

void target_follow()
{
    if(k210_msg.w !=0 ||k210_msg.h !=0)//K210开始传值
    {
        pid_down_angle += Position_PID (L_x,C_x);
        if(pid_down_angle>900)
            pid_down_angle = 900;
        else if (pid_down_angle<-900)
            pid_down_angle = -900;
        Servo_SetDownAngle(pid_down_angle+Servo_turn_where_target_is());//面向墙体基座标+pid 控制
        Servo_SetUpAngle(150);
    }
}
