#include "general_interface.h"

void CountBar_Test(void)
{
    Clear_HWCount();
    set_speed(0, 100, 0); // TODO 此处速度为测试速度 后期可以修改
    while (Get_HwBarCount(0) < 1)
    {
        osDelay(5);
    }
    osDelay(200);
    set_speed(0, 0, 0);
    osDelay(500);
}


