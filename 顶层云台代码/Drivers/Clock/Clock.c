#include "ti_msp_dl_config.h"
#include "Clock.h"

volatile unsigned long tick_ms;//系统运行时间（ms）
volatile uint32_t start_time;//运行某函数时此刻的系统时间（ms）
void SysTick_Handler(void)//滴答时钟为一毫秒    编码器速度即每1ms的distance增量
{
    tick_ms++;
}
int mspm0_delay_ms(unsigned long num_ms)
{
    start_time = tick_ms;
    while (tick_ms - start_time < num_ms);
    return 0;
}

int mspm0_get_clock_ms(unsigned long *count)
{
    if (!count)
        return 1;
    count[0] = tick_ms;
    return 0;
}

void SysTick_Init(void)
{
    DL_SYSTICK_config(CPUCLK_FREQ/1000);
    NVIC_SetPriority(SysTick_IRQn, 0);
}
