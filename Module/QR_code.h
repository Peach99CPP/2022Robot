/*
 * @Author: rlantic
 * @Date: 2022-03-14 15:44:26
 * @LastEditors: OBKoro1
 * @LastEditTime: 2022-04-13 08:56:06
 * @FilePath: \MDK-ARMd:\program\motor\motor\Module\QR_code.h
 * @Description:
 *
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved.
 */
#ifndef __Q_R_CODE_H_
#define __Q_R_CODE_H_

#include "main.h"
#include <string.h>
#include <stdbool.h>
typedef enum
{
    init_status = 0,
    red,
    blue,
} QRcolor_t;
typedef struct
{
    UART_HandleTypeDef *QR_uart;
    bool enable_switch;
    uint8_t rec_len;
    uint8_t RX_OK;
    uint8_t RX_data[20];
    QRcolor_t color;
} QR_t;

void QR_Mode_Init(bool status, QRcolor_t target_color);
void QR_receive(void);
void QR_decode(void);

void Set_QR_Target(QRcolor_t color);
void Set_QR_Status(bool status);

int Get_QRColor(void);
bool Return_QRMode(void);
void DeInit_QRColor(void);

bool Get_temp_var(void);
void Set_temp_var(bool state);

void Set_QrCount(int val);

void SetUpdateQR(void);

int Get_UpdateQr(void);

#endif
