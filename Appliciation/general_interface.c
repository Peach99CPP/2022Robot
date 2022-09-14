/*
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 14:16:44
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-14 16:27:06
 * @FilePath: \MDK-ARMd:\robot\robot\Appliciation\general_interface.c
 * @Description:
 *
 * Copyright (c) 2022 by peach 1831427532@qq.com, All Rights Reserved.
 */

#include "general_interface.h"
/**
 * @description: ͨ�����ú����������������
 * @param {int} idʹ���ĸ�����
 * @param {int} num �����ٸ�����
 * @param {int} speed��ʲô�ٶ�����
 * @return {*}
 */
void Move_CountBar(int id, int num, int speed)
{
    if (id != 0 && id != 2)
    {
        printf("ѡ��ĺ���id����");
        return;
    }
    Init_BarCount(id);
    set_imu_status(1);      // TODO ȷ�������ǿ���״̬
    set_speed(0, speed, 0); // TODO ������ʽ�����ٶ�
    while (Get_BarCount(id) < num)
    {
        osDelay(5);
    }
    osDelay(200);
    set_speed(0, 0, 0);
    osDelay(100);
}
