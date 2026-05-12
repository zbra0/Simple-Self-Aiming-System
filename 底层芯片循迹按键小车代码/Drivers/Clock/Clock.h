#ifndef __CLOCK_H
#define __CLOCK_H

extern volatile unsigned long tick_ms;
extern volatile uint32_t start_time;


//滴答时钟为一毫秒
int mspm0_delay_ms(unsigned long num_ms);
int mspm0_get_clock_ms(unsigned long *count);
void SysTick_Init(void);









#endif
