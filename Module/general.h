/*
 * @Author: rlantic
 * @Date: 2022-03-14 15:44:26
 * @LastEditors: OBKoro1
 * @LastEditTime: 2022-04-14 09:06:42
 * @FilePath: \MDK-ARMd:\program\motor\motor\Module\general.h
 * @Description:
 *
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved.
 */
/************************************************************************
 *
 * FileName   : general.h
 * Version    : v1.0
 * Author     : 妗冨瓙
 * Date       : 2021-10-10
 * Description:
 *******************************************************************************/

#ifndef __GENERAL_H_
#define __GENERAL_H_

#include "servo.h"
#include "openmv.h"
#include "QR_code.h"

void WholeRun(void);
void WholeGameTaskFunc(void const *argument);

void Set_Debug_Param(int param);
void Set_Debug_Task(int id);

void Openmv_Scan_Bar(int status, int color);

void Home2Disc(int dir);
void Disc2Ware2plat(void);
void plat2ware2home(void);

int Get_TargetColor(void);

void StartCheckQrUpadte(void); //开始更新二维码数据

void Set_TargetColor(int target);

void plat2ware2homeRed(void);
void Disc2Ware2platRed(void);
void Home2DiscRed(int dir);


void Avoid_ObsOnwayBack(void);
#endif
