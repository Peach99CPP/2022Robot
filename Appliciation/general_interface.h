/***
 * @Author: peach 1831427532@qq.com
 * @Date: 2022-09-05 14:14:24
 * @LastEditors: peach 1831427532@qq.com
 * @LastEditTime: 2022-09-14 15:06:28
 * @FilePath: \MDK-ARMd:\robot\robot\Appliciation\general_interface.h
 * @Description:
 * @
 * @Copyright (c) 2022 by peach 1831427532@qq.com, All Rights Reserved.
 */
#ifndef __GENERAL_INTERFACE_H_
#define __GENERAL_INTERFACE_H_

#include "motor.h"
#include "chassis.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "track_bar_receive.h"
#include "imu_pid.h"
#include "atk_imu.h"
#include "read_status.h "
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

void Move_CountBar(int id, int num, int speed);

#endif
