
#include "general_interface.h"

void CountBar_Test(int num, int speed)
{
    Clear_HWCount();
    set_imu_status(1);      // TODO ȷ�������ǿ���״̬
    set_speed(0, speed, 0); // TODO ������ʽ�����ٶ�
    while (Get_HwBarCount(0) < num)
    {
        osDelay(5);
    }
    osDelay(500);
    set_speed(0, 0, 0);
    osDelay(100);
}
