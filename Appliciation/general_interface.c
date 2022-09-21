/*
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 14:16:44
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-22 02:53:17
 * @FilePath: \MDK-ARMd:\robot\robot\Appliciation\general_interface.c
 * @Description:
 *
 * Copyright (c) 2022 by peach 1831427532@qq.com, All Rights Reserved.
 */

#include "general_interface.h"

#define Wait_Dealy_MAX 30000
#define Line_Type 1
#define Encoder_Type 2
#define YuanPanDelay 16 * 1000
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
//���볡����
void Run4WholeGame(int stage)
{
    if (stage == 1)
    {
        move_by_encoder(1, 180);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Move_CountBar(2, 2, 120);
        Wait_Switches(1);
        HWSwitch_Move(0, 1);
        move_by_encoder(2, -60); // TODO 9_22 2.27����Ϊ60
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
    }
    else if (stage == 2)
    {
        set_imu_status(0);
        set_speed(40,0,0);
        ActionGroup(0, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        
        MV_SendCmd(1, 0); //ʹ��mv
        osDelay(50);
        MV_SendCmd(2, 1); //���û�ɫ��ΪĿ����
        osDelay(50);
        Set_QueryState(1);
        printf("�����ɫ��ȴ�ʱ��\n");
        osDelay(YuanPanDelay);//���õȴ�ʱ��
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(2, 2); //��ɫ
        osDelay(50);
        ActionGroup(2,1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        Set_QueryState(1);
        printf("����Ŀ����ȴ�ʱ��\n");
        osDelay(YuanPanDelay);
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(0, 0);
        Set_InitYaw(0);
        set_speed(0,0,0);
        printf("���β������\n");
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
        set_speed(-40,0,0);
        ActionGroup(4, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        osDelay(2500);
        ActionGroup(7, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        ActionGroup(5, 1);
        set_speed(0,0,0);
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
        set_speed(0,0,0);
        ActionGroup(8,1);
    }
}
void RedGame2Test(int stage)
{
    //�ں쳡ʱ��Ҫע�� ��ʱ�������ǿ��ҵ�
    if (stage == 1)
    {
        move_by_encoder(1, 180);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Move_CountBar(0, 2, -120);
        Wait_Switches(1);
        HWSwitch_Move(2, 1); // TODO ע��˴���dir������2 ������ʱ���ֿ���

        move_by_encoder(2, 60); // TODO ע��˴���encoder����val����ֵ Ҫ�ó�����y�� ������ʱ���ֿ���
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Wait_Switches(1); // TODO ������ײ
    }
    else if (stage == 2)
    {
        set_imu_status(0);
        set_speed(40, 0, 0);
        ActionGroup(0, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);

        MV_SendCmd(1, 0); //ʹ��mv
        osDelay(50);
        MV_SendCmd(2, 1); //���û�ɫ��ΪĿ����
        osDelay(50);
        Set_QueryState(1);
        printf("�������ȴ�ʱ��\n");
        osDelay(YuanPanDelay);
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(2, 0); //��ɫ
        osDelay(50);
        ActionGroup(2, 1);
        Wait_Servo_Signal(Wait_Dealy_MAX);
        Set_QueryState(1);
        printf("����Ŀ����ȴ�ʱ��\n");
        osDelay(YuanPanDelay);
        Set_QueryState(0);
        osDelay(100);
        MV_SendCmd(0, 0);
        Set_InitYaw(0);
        printf("�������β���\n");
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
        move_by_encoder(2, -680);
        Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
        Turn_angle(1, -90, 0);
    }
}
