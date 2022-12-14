/*
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 14:16:44
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-23 20:01:35
 * @FilePath: \MDK-ARMd:\robot\robot\Appliciation\general_interface.c
 * @Description:
 *
 * Copyright (c) 2022 by peach 1831427532@qq.com, All Rights Reserved.
 */

#include "general_interface.h"

#define Wait_Dealy_MAX 30000
#define Line_Type 1
#define Encoder_Type 2
#define YuanPanDelay 22 * 1000
/**
 * @description: 通过调用红外俩计算板子数量
 * @param {int} id使用哪个红外
 * @param {int} num 数多少个板子
 * @param {int} speed以什么速度运行
 * @return {*}
 */
void Move_CountBar(int id, int num, int speed)
{
    if (id != 0 && id != 2 && id != 9)
    {
        printf("选择的红外id有误");
        return;
    }
    if (id != 9)
    {
        Init_BarCount(id);
        set_imu_status(1);      // TODO 确认陀螺仪开启状态
        set_speed(0, speed, 0); // TODO 传参形式设置速度
        while (Get_BarCount(id) < num)
        {
            osDelay(5);
        }
        osDelay(50);
    }
    else
    {
        id = 2;
        Init_BarCount(id);
        set_imu_status(1);      // TODO 确认陀螺仪开启状态
        set_speed(0, speed, 0); // TODO 传参形式设置速度
        while (Get_BarCount(id) < num)
        {
            osDelay(5);
        }
        osDelay(200);
    }
    set_speed(0, 0, 0);
    osDelay(100);
}
//蓝半场程序
void Run4WholeGame(int stage)
{
    if (stage == 1)
    {
        move_by_encoder(1, 160);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Move_CountBar(2, 2, 120);
        Wait_Switches(1);
        HWSwitch_Move(0, 1);
        move_by_encoder(2, -60); // TODO 9_22 2.27调试为60
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
    }
    else if (stage == 2)
    {
        set_imu_status(0);
        set_speed(40, 0, 0);
        ActionGroup(0, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);

        MV_SendCmd(1, 0); //使能mv
        osDelay(50);
        MV_SendCmd(2, 1); //设置黄色球为目标球
        osDelay(50);
        Set_QueryState(1);
        printf("进入黄色球等待时间\n");
        osDelay(YuanPanDelay); //设置等待时间
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(2, 2); //蓝色
        osDelay(50);
        ActionGroup(2, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        Set_QueryState(1);
        printf("进入目标球等待时间\n");
        osDelay(YuanPanDelay);
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(0, 0);
        Set_InitYaw(0);
        set_speed(0, 0, 0);
        printf("本次拨球结束\n");
    }
    else if (stage == 3)
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
        set_imu_status(0);
        set_speed(-40, 0, 0);
        ActionGroup(4, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(2500);
        ActionGroup(7, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        ActionGroup(5, 1);
        set_speed(0, 0, 0);
        Set_InitYaw(0);
    }
    else if (stage == 4)
    {
        move_by_encoder(2, 680);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Turn_angle(1, -90, 0);
        move_by_encoder(2, 230);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        set_imu_status(0);
        set_speed(0, 0, 0);
        ActionGroup(8, 1);
    }
    else if (stage == 5)
    {
        move_by_encoder(1, 160);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        set_speed(0, 0, 0);
        osDelay(500);
        Move_CountBar(2, 2, 120);
        Wait_Switches(1);
        HWSwitch_Move(0, 1);
        ActionGroup(0, 1);
        move_by_encoder(2, -60); // TODO 9_22 2.27调试为60
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);

        set_imu_status(0);
        set_speed(40, 0, 0);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        MV_SendCmd(1, 0); //使能mv
        osDelay(50);
        Set_FirstFlag(1);
        MV_SendCmd(2, 1); //设置黄色球为目标球
        osDelay(50);
        Set_QueryState(1);
        printf("进入黄色球等待时间\n");
        osDelay(YuanPanDelay); //设置等待时间

        Set_QueryState(0);
        osDelay(100);
        Set_FirstFlag(1); //跳过第一个球
        MV_SendCmd(2, 2); //蓝色
        osDelay(50);
        ActionGroup(2, 1); //切换盖板
        Wait_Servo_Signal(Wait_Dealy_MAX);

        Set_QueryState(1);
        printf("进入目标球等待时间\n");
        osDelay(YuanPanDelay);
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(0, 0);
        Set_InitYaw(0);
        set_imu_status(1); //校正结束 开启陀螺仪
        set_speed(0, 0, 0);
        printf("本次拨球结束\n");
        ActionGroup(1, 1);
        move_by_encoder(1, -80);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        osDelay(500);
        Move_CountBar(0, 2, -120);
        Turn_angle(1, 180, 0);
        Wait_Switches(3);

        Wait_Servo_Signal(Wait_Dealy_MAX); //若机械臂已收起

        ActionGroup(6, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);

        HWSwitch_Move(1, 1);
        Wait_Switches(3); // TODO 测试功能
        set_imu_status(0);
        set_speed(-40, 0, 0);
        ActionGroup(4, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(2000);
        ActionGroup(2, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(500);
        ActionGroup(10, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(1500);
        
        ActionGroup(7, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        ActionGroup(5, 1);
        set_speed(0, 0, 0);
        Set_InitYaw(0);

        move_by_encoder(2, 680);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Turn_angle(1, -90, 0);
        osDelay(1000); //给时间试试 让陀螺仪进行车身修正
        move_by_encoder(2, 230);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        set_imu_status(0);
        set_speed(0, 0, 0);
    }
}
void RedGame2Test(int stage)
{
    //在红场时需要注意 此时超声波是靠右的
    if (stage == 1)
    {
        move_by_encoder(1, 160); // TODO 这里要和后面的stage5对应
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Move_CountBar(0, 2, -120);
        Wait_Switches(1);
        HWSwitch_Move(2, 1); // TODO 注意此处的dir参数是2 和蓝场时区分开的

        move_by_encoder(2, 60); // TODO 注意此处的encoder——val参数值 要让车子往y正 和蓝场时区分开的
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
    }
    else if (stage == 2)
    {
        set_imu_status(0);
        set_speed(40, 0, 0);
        ActionGroup(0, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);

        MV_SendCmd(1, 0); //使能mv
        osDelay(50);
        MV_SendCmd(2, 1); //设置黄色球为目标球
        osDelay(50);
        Set_QueryState(1);
        printf("进入黄球等待时间\n");
        osDelay(YuanPanDelay);
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(2, 0); //红色
        osDelay(50);
        ActionGroup(2, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        Set_QueryState(1);
        printf("进入目标球等待时间\n");
        osDelay(YuanPanDelay);
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(0, 0);
        Set_InitYaw(0);
        printf("结束本次拨球\n");
        set_speed(0, 0, 0);
    }
    else if (stage == 3)
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
        Wait_Switches(3);
        set_imu_status(0);
        set_speed(-40, 0, 0);
        ActionGroup(4, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(2500);
        ActionGroup(7, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        ActionGroup(5, 1);
        set_speed(0, 0, 0);
        Set_InitYaw(0);
    }
    else if (stage == 4)
    {
        ActionGroup(8, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);

        Red_Run2Home(-500, -100);
        Turn_angle(1, -90, 0);
        osDelay(500);
        move_by_encoder(2, 230);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        set_speed(0, 0, 0);
        set_imu_status(0);
    }
    else if (stage == 5)
    {
        // stage1
        move_by_encoder(1, 160);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Move_CountBar(0, 2, -120);
        Wait_Switches(1);
        HWSwitch_Move(2, 1); // TODO 注意此处的dir参数是2 和蓝场时区分开的
        ActionGroup(0, 1);
        move_by_encoder(2, 55); // TODO 23.49 9——22 测试数据
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Wait_Switches(1); // TODO 重新碰撞
        // stage2
        set_imu_status(0);
        set_speed(40, 0, 0);

        Wait_Servo_Signal(Wait_Dealy_MAX);

        MV_SendCmd(1, 0); //使能mv
        osDelay(50);
        Set_FirstFlag(1);
        MV_SendCmd(2, 1); //设置黄色球为目标球
        osDelay(50);
        Set_QueryState(1);
        printf("进入黄球等待时间\n");
        osDelay(YuanPanDelay);

        Set_QueryState(0);
        osDelay(100);
        Set_FirstFlag(1);
        MV_SendCmd(2, 0); //红色
        osDelay(50);
        ActionGroup(2, 1); //转换到目标球的盖板
        Wait_Servo_Signal(Wait_Dealy_MAX);
        Set_QueryState(1);
        printf("进入目标球等待时间\n");
        osDelay(YuanPanDelay);
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(0, 0);
        Set_InitYaw(0);
        printf("结束本次拨球\n");
        set_speed(0, 0, 0);
        // stage3
        ActionGroup(1, 1);
        move_by_encoder(1, -80);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);

        Move_CountBar(9, 2, 120); //特殊情况 使用9来进行处理
        Turn_angle(1, 180, 0);
        Wait_Switches(3);

        Wait_Servo_Signal(Wait_Dealy_MAX);

        ActionGroup(6, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);

        HWSwitch_Move(1, 1);
        Wait_Switches(3);
        set_imu_status(0);
        set_speed(-40, 0, 0);
        ActionGroup(4, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(2000);
        ActionGroup(2, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(500);
        ActionGroup(10, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(1500);

        ActionGroup(5, 1);
        set_speed(0, 0, 0);
        Set_InitYaw(0);
        osDelay(500);
        // stage4
        ActionGroup(8, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);

        Red_Run2Home(-490, -100); // TODO 9_22 16.26确定其为-495
        Turn_angle(1, -90, 0);
        osDelay(500);
        move_by_encoder(2, 210);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        set_speed(0, 0, 0);
        set_imu_status(0);
        ActionGroup(9, 1); //解除折叠
    }
}
void Red_Run2Home(int val, int speed)
{
    ActionGroup(6, 1);
    Wait_Servo_Signal(Wait_Dealy_MAX);
    while (!Get_HW(1)) //要等到它亮起来
    {
        set_speed(0, speed, 0);
    }
    set_imu_status(1);
    set_speed(0, speed, 0);
    while (Get_HW(1))
    {
        osDelay(10);
    }
    move_by_encoder(2, val);
    osDelay(800);
    ActionGroup(7, 1);
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
    osDelay(200);
}
