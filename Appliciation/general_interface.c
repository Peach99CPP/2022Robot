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
 * @description: 通过调用红外俩计算板子数量
 * @param {int} id使用哪个红外
 * @param {int} num 数多少个板子
 * @param {int} speed以什么速度运行
 * @return {*}
 */
void Move_CountBar(int id, int num, int speed)
{
    if (id != 0 && id != 2)
    {
        printf("选择的红外id有误");
        return;
    }
    Init_BarCount(id);
    set_imu_status(1);      // TODO 确认陀螺仪开启状态
    set_speed(0, speed, 0); // TODO 传参形式设置速度
    while (Get_BarCount(id) < num)
    {
        osDelay(5);
    }
    osDelay(200);
    set_speed(0, 0, 0);
    osDelay(100);
}
