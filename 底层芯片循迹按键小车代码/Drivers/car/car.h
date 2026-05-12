#ifndef  __CAR_H
#define  __CAR_H
/*转向函数、电机赋值函数*/
void Go_Ahead();
void Go_Back();
void Turn_Left();
void Turn_Right();
void Car_stop();
void Motor_LeftSpeed(unsigned long int speed,unsigned long int direct);
void Motor_RightSpeed(unsigned long int speed,unsigned long int direct);
void track_line(void);
void load_limit_correct(int rate1,int rate2);
void Turn_around();
void self_right();
void self_left();
void Quarter_turn_right();
void Gryo_control();
float xunji();
void Set_Pwm(int Motor_Left,int Motor_Right);
void load_pid();
float Get_Speed_Dita(float YAW_D);
#endif


