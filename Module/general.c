/*
 * @Author: rlantic
 * @Date: 2022-03-19 16:12:02
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-20 02:11:04
 * @FilePath: \MDK-ARMd:\robot\robot\Module\general.c
 * @Description:
 *
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved.
 */
#include "general.h  "
#include "cmsis_os.h"

#include "delay.h"
#include "motor.h"
#include "chassis.h"
#include "imu_pid.h"
#include "track_bar_receive.h"
#include "chassis_control.h"
#include "openmv.h"
#include "servo.h"
#include "atk_imu.h"
#include "read_status.h"
#include "servo.h"
#include "general.h"
#include "avoid_obs.h"
//给等待信息的可读性编码
#define Wait_Dealy_MAX 30000
#define Line_Type 1
#define Encoder_Type 2

int debug_function_id = 0;
int function_param = 2;

int target_color = 2;
/**
 * @name: Set_TargetColor
 * @brief: 设置目标球的颜色
 * @param {int} target 1红2蓝
 * @return {*}
 */
void Set_TargetColor(int target)
{
    if (target == 1)
        printf("红色\r\n");
    else if (target == 2)
        printf("蓝色\r\n");
    else
    {
        printf("参数错误\r\n 需要重传参数\r\n");
        return;
    }
    target_color = target;
}
/**
 * @name: Get_TargetColor
 * @brief: 获取目标的颜色
 * @param {*} void
 * @return {*} 1 红2蓝 用于判断
 */
int Get_TargetColor(void)
{
    return target_color;
}
/**********************************************************************
 * @Name    Openmv_Scan_Bar
 * @declaration :寮�鍚疧PENMV鎵?鎻忔潯褰㈠钩鍙?
 * @param   status: [杈撳叆/鍑篯  寮�濮嬭繕鏄?缁撴??
 **			 color: [杈撳叆/鍑篯 瑕佹姄鐨勯?滆壊锛屽弽棣堢粰mv
 * @retval   : 鏃?
 * @author  peach99CPP
 ***********************************************************************/
void Openmv_Scan_Bar(int status, int color)
{
    if (status == 1)
    {
        MV_Start();
        osDelay(100);
        MV_SendCmd(3, color);
        Action_Gruop(5, 1);
    }
    else if (status == 0)
    {
        MV_Stop();
        Action_Gruop(6, 1);
    }
}

// 以下函数均为蓝色半场使用
/**
 * @name: Home2Disc
 * @brief: 起步到圆盘机 蓝区路线
 * @param {int} dir /方向 左1 右2
 * @return {*}
 */
void Home2Disc(int dir)
{
    MV_Stop();                                //关闭相机
    move_by_encoder(2, 25);                   //往前走到线上 todo 参数测试
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //等待完成
    //direct_move(2, 3, 0, 1);                  //走三根线 直接到达目标线
    Wait_OKInf(Line_Type, Wait_Dealy_MAX * 10);
    Turn_angle(1, -90, 1);
    Comfirm_Online(3); // 4 15新修改 记得测试
}
void Home2DiscRed(int dir)
{
    MV_Stop();                                //关闭相机
    move_by_encoder(2, 25);                   //往前走到线上 todo 参数测试
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //等待完成
    //direct_move(2, 3, 0, 1);                  //走三根线 直接到达目标线
    Wait_OKInf(Line_Type, Wait_Dealy_MAX * 10);
    Turn_angle(1, 90, 1);
    Comfirm_Online(3); //不数边线了 直接用确认上线函数
}

void Disc2Plat(void)
{
    move_by_encoder(2, -4);
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
    Turn_angle(1, -90, 1);
    //direct_move(1, 1, 1, 1);
    Wait_OKInf(Line_Type, Wait_Dealy_MAX);
    //direct_move(2, 1, 1, 1);
    Wait_OKInf(Line_Type, Wait_Dealy_MAX);
    move_by_encoder(2, 25);
    Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
    Wait_Switches(3);
    // HWSwitch_Move(6, 1);
}
/**
 * @name: Disc2Ware2plat
 * @brief: 圆盘机到仓库到阶梯
 * @param {*}
 * @return {*}
 */
void Disc2Ware2plat(void)
{
//     // 在这个过程中 车头一直是直对着的 无需转弯
//     move_by_encoder(2, -4); //后退 准备转弯 todo 测试-4参数是否合适 之前测试的-6偏大
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
//     // 待测试 todo 4.11新增 圆盘回仓库的方案
//     Turn_angle(1, -90, 1);
//     direct_move(2, 1, 0, 1);
//     Wait_OKInf(Line_Type, Wait_Dealy_MAX);

//     move_by_encoder(2, 40);
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
//     Turn_angle(1, -90, 0);                    //车屁股对准仓库
//     Wait_Switches(4);                         //用车尾对准仓库挡板
//     Lateral_infrared(1);                      //打开侧面的红外
//     Set_IMUStatus(1);                         //同步红半场
//     Kiss_Ass(1, 1);                           //用侧红外做到对准位置
//     Wait_Switches(4);                         //安全措施 todo 视运行情况增加
//     Ass_Door(1);                              //倒球一系列动作组
//     Lateral_infrared(0);                      //把侧面的红外收起
//     move_by_encoder(2, 6);                    //前进到线上 todo 参数确定
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //确认信息
//     direct_move(2, 1, 0, 1);                  //前进到线上
//     Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//     Turn_angle(1, 90, 1); //转弯
//     Wait_Switches(3);     //对准
//     HWSwitch_Move(6, 1);  //对准位置
//     // todo 完结撒花
// }
// void Disc2Ware2platRed(void)
// {
//     // 在这个过程中 车头一直是直对着的 无需转弯
//     move_by_encoder(2, -4); //后退 准备转弯 todo 测试-4参数是否合适 之前测试的-6偏大
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
//     Turn_angle(1, 90, 1);    //转弯车头对准线
//     direct_move(2, 1, 0, 1); //往右移动到靠近仓库的路口
//     Wait_OKInf(Line_Type, Wait_Dealy_MAX);

//     move_by_encoder(2, 40); //往前走多一点来避开木板缝隙
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX);
//     //  此处错误使用了倒物块的程序 todo 以下为修正程序
//     Turn_angle(1, 90, 0); //车屁股对准仓库
//     Wait_Switches(4);     //用车尾对准仓库挡板
//     Lateral_infrared(1);  //打开侧面的红外
//     Set_IMUStatus(0);
//     Kiss_Ass(1, 1);      //用侧红外做到对准位置
//     Wait_Switches(4);    //安全措施 todo 视运行情况增加
//     Ass_Door(1);         //倒球一系列动作组
//     Lateral_infrared(0); //把侧面的红外收起
//     Set_IMUStatus(1);
//     move_by_encoder(2, 6);                    //前进到线上 todo 参数确定
//     Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //确认信息
//     direct_move(2, 1, 0, 1);                  //前进到线上
//     Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//     Turn_angle(1, 90, 1); //转弯
//     Wait_Switches(3);     //对准
//     HWSwitch_Move(6, 1);  //对准位置
//     // todo 完结撒花
}
/**
 * @name: plat2ware
 * @brief: 从阶梯平台到出发区
 * @param {*}
 * @return {*}
 */
void plat2ware2home(void)
{
    // //此时运行完阶梯平台· 处于阶梯的5号边缘
    // Action_Gruop(4, 1);                //安全措施
    // Wait_Servo_Signal(Wait_Dealy_MAX); //确保上一任务完成
    // move_slantly(3, 120, 1500);        //斜着后退到线上
    // Turn_angle(1, 90, 1);
    // Wait_Switches(4);                      //改变对准方案
    // direct_move(2, 2, 1, 1);               //往前走到线上 到达第一个路口
    // Wait_OKInf(Line_Type, Wait_Dealy_MAX); //等待完成

    // Wait_Switches(1);                         //撞上去 同时修正角度
    // HWSwitch_Move(2, 1);                      //向右移动来确保边缘
    // move_by_encoder(2, -4);                   //后退一点点 大哥求求你别卡了
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //确保完成
    // osDelay(300);
    // move_by_encoder(1, -6);                   //移动到指定位置准备倒 todo 尝试将其倒在B区 运行新方案
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //确保完成
    // Wait_Switches(1);                         //防止在上一任务中脱离挡板
    // Action_Gruop(20, 1);                      //倒下物料
    // Wait_Servo_Signal(Wait_Dealy_MAX);        //确保上一任务完成
    // move_by_encoder(2, -6);                   //倒完向后倒退
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //确保完成
    // Turn_angle(1, 90, 1);                     //转90° 此时车头对着圆盘机
    // move_by_encoder(2, -4);                   //避开木板的缝隙  todo  4.15决定缩小本尺寸 待测试具体参数
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //确保完成
    // Set_SwitchSpeed(120);                     //临时设置高一点
    // Wait_Switches(3);                         //向右撞击挡板修正车身姿态
    // osDelay(200);                             //等一下再重新开一遍
    // Set_SwitchSpeed(80);                      //恢复初始的速度设置
    // Ring_Move();                              //移动过去放圆环
    // move_by_encoder(1, -4);                   //移动出来  todo 412修改参数
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //等待结束
    // Turn_angle(1, 180, 0);                    //转弯准备回家
    // Comfirm_Online(3);
    // Comfirm_Online(1);
    // Avoid_ObsOnwayBack();
    // set_speed(0, 0, 0); //停车
    // Set_InitYaw(0);     //以此时的角度为设定角度
    // Set_IMUStatus(0);   //关闭陀螺仪
}
void plat2ware2homeRed(void)
{
    // //此时运行完阶梯平台· 处于阶梯的5号边缘Action_Gruop(4,1);
    // Action_Gruop(4, 1); //安全措施
    // Wait_Servo_Signal(Wait_Dealy_MAX);
    // move_slantly(3, 120, 1700); //斜着后退到线上
    // Turn_angle(1, 90, 1);
    // direct_move(2, 1, 1, 1);               //往前走到线上 到达第一个路口
    // Wait_OKInf(Line_Type, Wait_Dealy_MAX); //等待完成

    // Wait_Switches(1);                         //撞上去 同时修正角度
    // HWSwitch_Move(2, 1);                      //向右移动来确保边缘
    // move_by_encoder(1, -9);                   //移动到指定位置准备倒 todo 4月10号修正为9
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //确保完成
    // Wait_Switches(1);                         //防止在上一任务中脱离挡板
    // Action_Gruop(20, 1);                      //倒下物料
    // Wait_Servo_Signal(Wait_Dealy_MAX);        //确保上一任务完成
    // move_by_encoder(2, -6);                   //倒完向后倒退
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //确保完成
    // Turn_angle(1, 90, 1);                     //转90° 此时车头对着圆盘机
    // move_by_encoder(2, -4);                   //避开木板的缝隙
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //确保完成
    // Set_SwitchSpeed(120);                     //临时设置高一点
    // Wait_Switches(3);                         //向右撞击挡板修正车身姿态
    // osDelay(200);                             //等一下再重新开一遍
    // Set_SwitchSpeed(80);                      //恢复初始的速度设置
    // Ring_Move();                              //移动过去放圆环
    // move_by_encoder(1, -4);                   //移动出来  todo 参数测试通过
    // Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //等待结束
    // osDelay(100);                             //停止一小会
    // Comfirm_Online(3);
    // Comfirm_Online(2);
    // osDelay(500);
    // Avoid_ObsOnwayBack(); //回家路上实现避障
    // set_speed(0, 0, 0);   //停车
    // Set_InitYaw(0);       //以此时的角度为设定角度
    // Set_IMUStatus(0);     //关闭陀螺仪
}
/**
 * @name: Set_Debug_Param
 * @brief: 设置出发方向
 * @param {int} param  1 左2右
 * @return {*}
 */
void Set_Debug_Param(int param)
{
    // function_param = param;
    // printf("\n设置完参数可以出发\n");
}
/**
 * @name: Set_Debug_Task
 * @brief: 通过创建函数的问题
 * @param {int} id 1从圆盘机出发 2从圆盘机回仓库 3从阶梯回家
 * @return {*}
 */
void Set_Debug_Task(int id)
{
    debug_function_id = id;
    WholeRun();
}
osThreadId WholeGameHandle = NULL;            //任务句柄
void WholeGameTaskFunc(void const *argument); //任务实现函数
bool WholeTaskTask_Exit = 1;                  //是否退出

osThreadId CheckQrUpdateHandle = NULL;       //任务句柄
void QrUpdateTaskFunc(void const *argument); //任务实现函数
bool QrkTask_Exit = 1;                       //是否退出
#define UpdateThreshold 5000

void StartCheckQrUpadte(void)
{
    if (CheckQrUpdateHandle == NULL)
    {
        osDelay(10);
        WholeTaskTask_Exit = 0;
        osThreadDef(QrUpdate, QrUpdateTaskFunc, osPriorityNormal, 0, 256); //定义任务结构体
        WholeGameHandle = osThreadCreate(osThread(QrUpdate), NULL);        //创建任务
    }
}
void Exit_QRcheck(void)
{
    QrkTask_Exit = 1;
}
void QrUpdateTaskFunc(void const *argument)
{
    // static long coount_time = 0;
    // static int current_val = 0;
    // while (!QrkTask_Exit)
    // {
    // recount_symbol:
    //     current_val = Get_UpdateQr();
    //     while (Get_UpdateQr() == current_val && !QrkTask_Exit)
    //     {
    //         osDelay(5);
    //         coount_time += 5;
    //         if (coount_time >= UpdateThreshold)
    //         {
    //             printf("\n\t更新QR标志位\n");
    //             Set_temp_var(false);
    //             Set_QrCount(0);
    //         }
    //     }
    //     osDelay(100);
    //     goto recount_symbol;
    // }
    // CheckQrUpdateHandle = NULL;
    // vTaskDelete(NULL);
}
void WholeRun(void)
{
    if (WholeGameHandle == NULL)
    {
        osDelay(10);
        WholeTaskTask_Exit = 0;
        osThreadDef(WholeGame, WholeGameTaskFunc, osPriorityNormal, 0, 1024); //定义任务结构体
        WholeGameHandle = osThreadCreate(osThread(WholeGame), NULL);          //创建任务
    }
}

void WholeGameTaskFunc(void const *argument)
{
    // if (debug_function_id >= 0 && debug_function_id <= 3)
    // {
    //     Set_TargetColor(2);
    // }
    // else if (debug_function_id >= 5)
    // {
    //     Set_TargetColor(1);
    // }
    // switch (debug_function_id)
    // {
    // case 0:
    //     Set_TargetColor(2); //蓝色
    //     Home2Disc(function_param);
    //     printf("到达圆盘 开始执行圆盘机动作\n");
    //     Disc_Mea();
    //     osDelay(100);
    //     printf("圆盘结束 回仓库后去阶梯\n");
    //     // Disc2Ware2plat();
    //     Disc2Ware2plat();
    //     osDelay(100);
    //     printf("开始运行阶梯\n");
    //     Brick_QR_Mode(5, 1, 1, 1);
    //     osDelay(100);
    //     printf("从阶梯回家\n");
    //     plat2ware2home();
    //     // plat2ware2home();
    //     break;
    // case 1:
    //     Home2Disc(function_param);
    //     break;
    // case 2:
    //     Disc2Ware2plat();
    //     break;
    // case 3:
    //     plat2ware2home();
    //     break;
    // case 4:
    //     Disc_Mea();
    //     break;
    // case 5:
    //     Home2DiscRed(function_param);
    //     break;
    // case 6:
    //     Disc2Ware2platRed();
    //     break;
    // case 7:
    //     plat2ware2homeRed();
    //     break;
    // case 8:
    //     Set_TargetColor(1); //红色
    //     Home2DiscRed(function_param);
    //     printf("到达圆盘 开始执行圆盘机动作\n");
    //     Disc_Mea();
    //     printf("圆盘结束 回仓库后去阶梯\n");
    //     Disc2Ware2platRed();
    //     Brick_QR_Mode(5, 1, 1, 1);
    //     printf("从阶梯回家\n\n");
    //     plat2ware2homeRed();
    //     break;
    // case 9:;
    //     break;
    // case 10:;
    //     break;
    // case 11:
    //     Avoid_ObsOnwayBack();
    //     break;
    // default:
    //     printf("参数错误\n");
    //     break;
    // }
    // printf("调试函数运行完成\n");
    // osDelay(200);
    // WholeGameHandle = NULL;
    // vTaskDelete(NULL);
}

#define Distance_Threshold 0XDB
void Avoid_ObsOnwayBack(void)
{
// #define Blue_Left 1
// #define Blue_Right 2
// #define Red_Left 3
// #define Red_Right 4
//     printf("\n\t结束啦\n");
//     int avoid_dir = function_param;
//     if (avoid_dir == Blue_Left)
//     {
//         direct_move(1, 1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         while (Get_Distance() > Distance_Threshold)
//         {
//             set_speed(0, 120, 0);
//         }
//         osDelay(100);
//         direct_move(1, -1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         direct_move(2, 1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         Turn_angle(1, 180, 1);
//         Comfirm_Online(2);
//         //向左确认上线
//         Set_HomeBack(1);
//         //设置关闭循迹
//         move_by_encoder(1, -28); // todo
//         //通过编码器回到出发区域前
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //等待完成
//         move_by_encoder(2, -14);                  // todo
//         //  往前走入出发区域
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //完成
//         track_status(1, 0);
//         track_status(2, 0);
//         Lateral_infrared(0); //确保侧面的红外被收起
//     }
//     else if (avoid_dir == Blue_Right)
//     {
//         set_speed(0, 120, 0);
//         osDelay(600); //耍赖
//         while (Get_Distance() > Distance_Threshold)
//         {
//             set_speed(0, 120, 0);
//         }
//         osDelay(100);
//         direct_move(1, 1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         Comfirm_Online(3);
//         Turn_angle(1, 180, 0);
//         Comfirm_Online(3);      //看情况添加
//         move_by_encoder(1, 29); //尝试新参数
//         //通过编码器回到出发区域前
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //等待完成
//         move_by_encoder(2, -15);
//         //  往前走入出发区域
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //完成
//         track_status(1, 0);
//         track_status(2, 0);
//         Lateral_infrared(0); //确保侧面的红外被收起
//     }
//     else if (avoid_dir == Red_Left)
//     {
//         set_speed(0, 120, 0);
//         osDelay(500); //耍赖
//         while (Get_Distance() > Distance_Threshold)
//         {
//             set_speed(0, 120, 0);
//         }
//         set_speed(0, 0, 0);
//         direct_move(1, -1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         Comfirm_Online(3);
//         Turn_angle(1, 180, 1);
//         Comfirm_Online(1);                        //确认上线
//         Set_HomeBack(1);                          //设置此时是回家状态
//         move_by_encoder(1, -29);                  //此时车子在右侧
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //等待完成
//         move_by_encoder(2, -16);                  //  往前走入出发区域
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //完成
//     }
//     else if (avoid_dir == Red_Right)
//     {
//         direct_move(1, -1, 0, 1); //往左移动到线上
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         while (Get_Distance() > Distance_Threshold)
//         {
//             set_speed(0, 120, 0);
//         }
//         set_speed(0, 0, 0);
//         direct_move(1, 1, 0, 1);
//         Wait_OKInf(Line_Type, Wait_Dealy_MAX);
//         Comfirm_Online(3);
//         Turn_angle(1, 180, 1);
//         Comfirm_Online(1);                        //确认上线
//         Set_HomeBack(1);                          //设置此时是回家状态
//         move_by_encoder(1, 29);                   //此时车子在右侧
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //等待完成
//         move_by_encoder(2, -16);                  //  往前走入出发区域
//         Wait_OKInf(Encoder_Type, Wait_Dealy_MAX); //完成
//     }
}

