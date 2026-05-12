#include "kubian_ditect.h"
#include "stdio.h"
#include "ti_msp_dl_config.h"
extern int MSGx,MSGy,MSGw,MSGh;//k210传的值
extern int kubian_predict_FLG;
extern int kunum;
uint16_t level_,left,right;
void count_ku()//控制库的数量kunum
{
    if((MSGw & 0x03) == 0x03 && kubian_predict_FLG && (!!(MSGx & 0x04)))//有FLG且都扫到
    {
        kunum++;
        kubian_predict_FLG = 0;
    }

    else if(!!(MSGx & 0x08))//第二个扫到预警
    {
        kubian_predict_FLG = 1;
    }

}
int ku_dis()//返回到库边的距离
{
    level_=MSGy*256+MSGw,left=0,right=0;
    while(!(level_ & 0x8000) && left<=15 )
    {
        left++;
        level_=level_<<1;
    }
        while(!(level_ & 0x0001) && right<=15 )
    {
        right++;
        level_=level_>>1;
    }
    return (15+left-right);
}