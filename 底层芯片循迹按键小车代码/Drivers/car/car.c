#include "ti_msp_dl_config.h"
#include "car.h"
#include "pid.h"
#include "Clock.h"
#include "servo.h"
#include "mpu6050.h"
#include "oled_software_i2c.h"

#define first_car 1
#define Judge_method 1
//#define PID_method 1
//#define OLD_car 1
#define SPEED_BASE 19
#define forward 0
#define backward 1

extern int run_circle,expect_circle,start_flag;//跑圈数量
extern unsigned short Normal[8];
extern uint8_t oled_buffer[32],SP_buffer[32];
int flag_right=0;
int flag_stop=1;
int special_case_flag=0;
extern int L_Distance,R_Distance,L_Encoder_speed,R_Encoder_speed,L_Distance_last,R_Distance_last,encoder_dis;
int flag_judge=0;
extern float speed_dita;
extern float yaw_bias;
extern float yaw_last,yaw,yaw_exp;
extern float YAW_D;
float YAWBIAS[4]={90,180,270,0};
int YAWBIAS_count=0;
int flag_BIAS=1;//每次出现只允许BIAS自加一次

int bias ;//灰度循迹的差速环
int targetA,targetB,Motor_Left,Motor_Right ;
float CurrentA , CurrentB ; 
extern int in;



void Motor_LeftSpeed(unsigned long int speed,unsigned long int angle)
{
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, (100-speed)*10, DL_TIMER_CC_0_INDEX);
    if(angle==0)
    {
        DL_GPIO_setPins(GPIO_Motor_PORT,GPIO_Motor_ain2_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PORT,GPIO_Motor_ain1_PIN);
    }
    else 
    {
        DL_GPIO_setPins(GPIO_Motor_PORT,GPIO_Motor_ain1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PORT,GPIO_Motor_ain2_PIN);
    }
}

void Motor_RightSpeed(unsigned long int speed,unsigned long int angle)
{
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, (100-speed)*10, DL_TIMER_CC_1_INDEX);
    if(angle==0)
    {
        DL_GPIO_setPins(GPIO_Motor_PORT,GPIO_Motor_bin1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PORT,GPIO_Motor_bin2_PIN);
    }
    else 
    {
        DL_GPIO_setPins(GPIO_Motor_PORT,GPIO_Motor_bin2_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PORT,GPIO_Motor_bin1_PIN);
    }
}

void Set_Pwm(int Motor_Left,int Motor_Right)
{
    if(Motor_Left>=0)
    {
        Motor_LeftSpeed(Motor_Left,0);
    }
    else
    {
        Motor_LeftSpeed((unsigned long int)(-Motor_Left),1);
    }

        if(Motor_Right>=0)
    {
        Motor_RightSpeed(Motor_Right,0);
    }
    else
    {
        Motor_RightSpeed((unsigned long int)(-Motor_Right),1);
    }
}

float xunji()			//输出差速，左轮速度为middle+x，右轮速度为middle-x
{
    if(Normal[3]<=2000 && Normal[4]<=2000) return 0;
    else if (Normal[2]<=2000 && Normal[3]<=2000) return -1.5;
    else if (Normal[1]<=2000 && Normal[2]<=2000) return -3;
    else if (Normal[0]<=2000 && Normal[1]<=2000) return -6;
    else if (Normal[4]<=2000 && Normal[5]<=2000) return 1.5;
    else if (Normal[5]<=2000 && Normal[6]<=2000) return 3;
    else if (Normal[6]<=2000 && Normal[7]<=2000) return 6;
    else
    {
        if(Normal[3]<=2000)  return -1;
        else if(Normal[4]<=2000)  return 1;
        else if(Normal[2]<=2000)  return -2;
        else if(Normal[5]<=2000)  return 2;
        else if(Normal[1]<=2000)  return -4;
        else if(Normal[6]<=2000)  return 4;
        else if(Normal[0]<=2000)  return -7;
        else if(Normal[7]<=2000)  return 7;
        return 0;
    }
}


void Go_Ahead(){
	Motor_LeftSpeed(18,0);
	Motor_RightSpeed(18,0);
}
void Go_Back(){
	Motor_LeftSpeed(15,1);
	Motor_RightSpeed(15,1);
}

void Turn_Left(){
	Motor_LeftSpeed(15,0);
	Motor_RightSpeed(18,0);
}
void Turn_Right(){
	Motor_RightSpeed(1,0);
	Motor_LeftSpeed(25,0);
}

void Car_stop(){
	Motor_LeftSpeed(1,1);
	Motor_RightSpeed(1,1);
}

void Quarter_turn_right(){
	Motor_RightSpeed(1,0);
	Motor_LeftSpeed(16,0);
    delay_cycles(80000000);   
}

void self_right()
{
	Motor_RightSpeed(12,1);
	Motor_LeftSpeed(17,0);
}

void self_left()
{
	Motor_LeftSpeed(12,1);
	Motor_RightSpeed(17,0);
}

void load_pid()
{
    bias = xunji();
    targetA = SPEED_BASE+bias;
	targetB = SPEED_BASE-bias;
    CurrentA = (float)L_Encoder_speed; //left
	CurrentB = (float)R_Encoder_speed; //right
	Motor_Left  = limit(PID_A(CurrentA,targetA));
	Motor_Right = limit(PID_B(CurrentB,targetB));		//PWM限幅
	Set_Pwm(Motor_Left, Motor_Right);
}

void track_line()
{
     if(encoder_dis>=(1390))
    {   
        if(flag_stop && expect_circle--)
        {
            flag_stop=0;
            yaw_exp+=90.0;
            if(yaw_exp>=360)
                yaw_exp-=360.0;                    
        }    
            if(yaw<0)
                yaw = yaw+360.0;//0~360
            YAW_D=yaw_exp-yaw;
            if(YAW_D>180.0)//-180~180
                YAW_D=360.0-YAW_D;
            else if(YAW_D<-180.0)
                YAW_D=360.0+YAW_D;

            if(YAW_D<5.0 && YAW_D>-5.0)
            {
                L_Distance=0;
                R_Distance=0;
                encoder_dis=0; 
                Go_Ahead();
                delay_cycles(8000000);
            } 
            else
            {
            if(expect_circle)
                {
                    Turn_Right();
                }
            else 
                {
                    Car_stop();
                }
            }
    }        
    else 
    {
        if(expect_circle)
        {
            flag_stop=1;
            load_pid(); 
        }
        else {
        Car_stop();
        }
    }
}
