
#include "general_interface.h"

void CountBar_Test(int num, int speed)
{
    Clear_HWCount();
    set_imu_status(1);      // TODO 确认陀螺仪开启状态
    set_speed(0, speed, 0); // TODO 传参形式设置速度
    while (Get_HwBarCount(0) < num)
    {
        osDelay(5);
    }
    osDelay(500);
    set_speed(0, 0, 0);
    osDelay(100);
}
