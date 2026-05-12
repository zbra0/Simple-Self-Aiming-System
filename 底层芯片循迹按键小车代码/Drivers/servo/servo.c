#include "ti_msp_dl_config.h"
extern float yaw;
extern float yaw_base;
float yawdilta_cal_sv;
void Servo_SetUpAngle(float Angle)
{
	if(Angle<100)
		Angle = 100;
	if(Angle>400)
		Angle = 400;
	DL_TimerG_setCaptureCompareValue(servo_INST,Angle*200/180+500,DL_TIMER_CC_0_INDEX);

}

void Servo_SetDownAngle(float Angle)
{
	if(Angle<0)
		Angle = 0;
	if(Angle>2700)
		Angle = 2700;
	DL_TimerG_setCaptureCompareValue(servo_INST,Angle*200/270+500,DL_TIMER_CC_1_INDEX);
}
float Servo_turn_where_target_is()
{
	float yaw_cal,yaw_base_cal,yawdilta_cal;
	yaw_cal=yaw<0?yaw+360:yaw;
	yaw_base_cal=yaw_base<0?yaw_base+360:yaw_base;//统一为0-360
	yawdilta_cal = yaw_base_cal-yaw_cal;
	yawdilta_cal = yawdilta_cal>0?yawdilta_cal:yawdilta_cal+360;
	yawdilta_cal = yawdilta_cal<270?yawdilta_cal:270;
	yawdilta_cal_sv=yawdilta_cal;
	return yawdilta_cal*10;
}
