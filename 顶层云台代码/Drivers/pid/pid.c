#include "pid.h"
#include "stdio.h"
#include "ti_msp_dl_config.h"
float Blue_teeth_Kp;//蓝牙调试pid
/*需要初始化*/
int Position_PID (int Real,int Target)//电机转速pid控制
{ 	
	 float Position_KP=-0.75,Position_KI=-0.0,Position_KD=0.025;
	 //Position_KD = Blue_teeth_Kp;
	 static float Bias,Pwm,Integral_bias,Last_Bias;
	 Bias=Target-Real;                                  //计算偏差
	 Integral_bias+=Bias;	                                 //求出偏差的积分
     Integral_bias=Integral_bias>20?20:Integral_bias;//积分限幅
	 Pwm=Position_KP*Bias+Position_KI*Integral_bias+Position_KD*(Bias-Last_Bias);       //位置式PID控制器
	 Last_Bias=Bias;                                       //保存上一次偏差 
	 if(Pwm>100)
	 	Pwm = 100;
	if(Pwm<-100)
		Pwm = -100;
	 return (int)Pwm;                                           //增量输出
}

void pid_cal(pid_t *pid)
{
	static float error_sum = 0;
	// 计算当前偏差
	//pid->error[0] = pid->target - pid->now;
    pid->error[0] = Yaw_error_zzk(pid->target, pid->now);
	error_sum += pid->error[0];
	error_sum=error_sum>20000?20000:error_sum;
	// 计算输出
	if(pid->pid_mode == DELTA_PID)  // 增量式
	{
		pid->pout = pid->p * (pid->error[0] - pid->error[1]);
		pid->iout = pid->i * pid->error[0];
		pid->dout = pid->d * (pid->error[0] - 2 * pid->error[1] + pid->error[2]);
		pid->out += pid->pout + pid->iout + pid->dout;
	}
	else if(pid->pid_mode == POSITION_PID)  // 位置式
	{
		pid->pout = pid->p * pid->error[0];
		pid->iout += pid->i * error_sum;//
		pid->dout = pid->d * (pid->error[0] - pid->error[1]);
		pid->out = pid->pout + pid->iout + pid->dout;
	}

	// 记录前两次偏差
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
}

void pid_init(pid_t *pid, uint32_t mode, float p, float i, float d)
{
	pid->pid_mode = mode;
	pid->p = p;
	pid->i = i;
	pid->d = d;
    pid->now = 0;

    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;
    pid->out = 0;
}

void pidout_limit(pid_t *pid)
{
    if(pid->out>=100)	
		pid->out=100;
	if(pid->out<=-100)	
		pid->out=-100;
}
float Yaw_error_zzk(float Target, float Now)//计算角度差值
 {
    static float error;
    if (Target > 0) 
	{
        if (Now <= 0) 
		{
            if (fabs(Now) < (180 - Target)) 
			{
                error = fabs_zzk(Now) + Target;
            } 
			else 
			{
                error = -(180 - Target) - (180 - fabs_zzk(Now));
            }
        } 
		else 
		{
            if (Now > 0) 
			{
                error = Target - Now;
            }
        }
    } 
	else if (Target < 0)
	 {
        if (Now > 0) 
		{
            if (Now > Target + 180)
			{
                error = (180 - Now) + (180 - fabs_zzk(Target));
            }
			 else if (Now < Target + 180) 
			{
                error = -(fabs_zzk(Target) + Now);
            }
        } 
		else if (Now < 0) 
		{
            error = -(fabs_zzk(Target) - fabs_zzk(Now));
        }
    }
    
    return error;
}
 
float fabs_zzk(float value)//绝对值
{
    if (value < 0) 
	{
        return -value;
    } 
	else 
	{
        return value;
    }
}