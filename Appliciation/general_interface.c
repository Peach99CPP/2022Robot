/*
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 14:16:44
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-21 18:30:45
 * @FilePath: \MDK-ARMd:\robot\robot\Appliciation\general_interface.c
 * @Description:
 *
 * Copyright (c) 2022 by peach 1831427532@qq.com, All Rights Reserved.
 */

#include "general_interface.h"

#define Wait_Dealy_MAX 30000
#define Line_Type 1
#define Encoder_Type 2
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
void Run4WholeGame(int stage)
{
    if (stage == 1)
    {
        move_by_encoder(1, 180);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Move_CountBar(2, 2, 120);
        Wait_Switches(1);
        HWSwitch_Move(0, 1);
        ActionGroup(0, 1);
        move_by_encoder(2, -65); // TODO 此参数需要调试
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Wait_Switches(1);
        while (!Get_Servo_Flag())
            osDelay(10);
        osDelay(2000);
    }
    else if (stage == 2)
    {
        ActionGroup(1, 1);
        move_by_encoder(1, -80);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);

        Move_CountBar(0, 2, -120);
        Turn_angle(1, 180, 0);
        Wait_Switches(3);
        while (!Get_Servo_Flag())
            osDelay(10);
        ActionGroup(6, 1);
        while (!Get_Servo_Flag())
            osDelay(10);
        HWSwitch_Move(1, 1);
        ActionGroup(4, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(5000);
        ActionGroup(7, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        ActionGroup(5, 1);
    }
    else if (stage == 3)
    {
        move_by_encoder(2, 680);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Turn_angle(1, -90, 0);
        move_by_encoder(2, 230);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
    }
}
void RedGame2Test(int stage)
{
    //在红场时需要注意 此时超声波是靠右的
    if (stage == 1)
    {
        move_by_encoder(1, 180);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Move_CountBar(0, 2, -120);
        Wait_Switches(1);
        HWSwitch_Move(2, 1); // TODO 注意此处的dir参数是2 和蓝场时区分开的
        ActionGroup(0, 1);
        move_by_encoder(2, 65); // TODO 注意此处的encoder——val参数值 要让车子往y正 和蓝场时区分开的
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);

        while (!Get_Servo_Flag())
            osDelay(10);
        osDelay(2000);
    }
    else if (stage == 2)
    {
        ActionGroup(1, 1);
        move_by_encoder(1, -80);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);

        Move_CountBar(2, 2, 120);
        Turn_angle(1, 180, 0);
        Wait_Switches(3);
        while (!Get_Servo_Flag())
            osDelay(10);
        ActionGroup(6, 1);
        while (!Get_Servo_Flag())
            osDelay(10);
        HWSwitch_Move(1, 1);
        ActionGroup(4, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        ActionGroup(7, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        ActionGroup(5, 1);
    }
    else if (stage == 3)
    {
        ActionGroup(8, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        move_by_encoder(1, 230);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        move_by_encoder(2, -680);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Turn_angle(1, -90, 0);
    }
}
