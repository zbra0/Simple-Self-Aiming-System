#ifndef _MAIN_H_
#define _MAIN_H_

#include "Clock.h"
#include "mpu6050.h"
#include "oled_software_i2c.h"
#include "oled_hardware_i2c.h"
#include "oled_software_spi.h"
#include "oled_hardware_spi.h"
#include "ultrasonic_capture.h"
#include "ultrasonic_gpio.h"
#include "bno08x_uart_rvc.h"
float Servo_turn_where_target_is(void);
void Servo_SetDownAngle(float Angle);
void Servo_SetUpAngle(float Angle);
#endif  /* #ifndef _MAIN_H_ */