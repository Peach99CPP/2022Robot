#include "general_interface.h"

void CountBar_Test(void)
{
    Clear_HWCount();
    set_speed(0, 100, 0); // TODO �˴��ٶ�Ϊ�����ٶ� ���ڿ����޸�
    while (Get_HwBarCount(0) < 1)
    {
        osDelay(5);
    }
    osDelay(200);
    set_speed(0, 0, 0);
    osDelay(500);
}


