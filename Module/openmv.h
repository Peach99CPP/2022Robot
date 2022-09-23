/*
 * @Author: rlantic
 * @Date: 2022-03-19 11:35:34
 * @LastEditors: OBKoro1
 * @LastEditTime: 2022-04-15 18:05:46
 * @FilePath: \MDK-ARMd:\program\motor\motor\Module\openmv.h
 * @Description:
 *
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved.
 */
#ifndef __OPENMV_H_
#define __OPENMV_H_

#include "string.h"

#include "main.h"
#include "usart.h"
#include "uart_handle.h"

#define START_BYTE 0XFF
#define END_BYTE 0X99
#define BUFFER_SIZE 5
#define MAX_REC_SIZE 6

typedef struct
{
    UART_HandleTypeDef *mv_uart;
    bool enable_switch;
    uint8_t mv_cmd[BUFFER_SIZE];
    uint8_t rec_buffer[MAX_REC_SIZE];
    uint8_t rec_len;
    uint8_t RX_Status;
} mv_t;
typedef enum
{
    red_color = 1,
    blue_color
} mvcolor_t;
typedef struct
{
    int event;
    int param;
} mvrec_t;

typedef enum
{
    ladder_type = 0,
    bar_type
} mv_type_t;

typedef enum
{
    LowestRing = 27,
    MediumRing,
    HighestRing
} MvRing_t;

void cmd_encode(const uint8_t event_id, uint8_t param);
void MV_SendCmd(const uint8_t event_id, const int param);
void MV_IRQ(void);
void MV_rec_decode(void);

int Get_Stop_Signal(void);
void Disable_StopSignal(void);
void Enable_StopSignal(void);

void OpenMV_ChangeRoi(int roi);

void MV_Decode(void);

void Set_MV_Mode(bool mode);
bool Get_MV_Mode(void);
int Get_DiscStatus(void);

//开始与结束
void MV_Start(void);
void MV_Stop(void);

void Set_AcIDofBar(short change); //增改
int Get_RecCount(void);           //用于判断此时抓取的序号
void Update_rectangle_count(void);
//查询任务
void MV_QueryTaskFunc(void const *argument);
void MV_QueryTask_Start(void);
void Set_QueryState(int state);
void Set_FirstFlag(int state);
int Get_FirstFlag(void);

#endif
