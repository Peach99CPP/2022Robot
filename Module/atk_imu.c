
#include "atk_imu.h"
#include "delay.h"

/* 模块主动上传的数据(串口解析后) */
attitude_t attitude; /*!< 姿态角 */
quaternion_t quaternion;
gyroAcc_t gyroAccData;
ioStatus_t iostatus;

#ifdef IMU901
mag_t magData;
baro_t baroData;
#endif

/* 模块寄存器参数值 */
regValue_t imu901Param;

/* 串口接收解析成功的数据包 */
atkp_t rxPacket;

/* 陀螺仪加速度量程表 */
const uint16_t gyroFsrTable[4] = {250, 500, 1000, 2000};
const uint8_t accFsrTable[4] = {2, 4, 8, 16};
float one_target;
/**
 * @brief  接收串口数据解包流程
 */
static enum { waitForStartByte1,
              //waitForStartByte2,
              waitForMsgID,
              waitForDataLength,
              waitForData,
              waitForChksum1,
} rxState = waitForStartByte1;

/**
 * @brief  imu901模块串口数据解析（串口接收的每一个数据需传入处理）
 *	@note	此函数需要实时调用
 * @param  ch: 串口接收的单个数据
 * @retval uint8_t: 0 无包 1 有效包
 */
uint8_t imu901_unpack(uint8_t ch)
{
    static uint8_t cksum = 0, dataIndex = 0;

    switch (rxState)
    {
    case waitForStartByte1://数据开头
        if (ch == UP_BYTE1)
        {
            rxState = waitForMsgID;
            rxPacket.startByte1 = ch;
        }

        cksum = ch;
        break;

  
    case waitForMsgID://数据ID
        rxPacket.msgID = ch;
        rxState = waitForData;
        cksum += ch;
        dataIndex = 0;
        break;

  

    case waitForData://正式录入有效数据
        rxPacket.data[dataIndex] = ch;//dataIndex从0开始，作为计算角度的数据
        dataIndex++;
        cksum += ch;

 
        if(dataIndex == 8)
        {
            rxState = waitForChksum1;
        }
        break;

    case waitForChksum1:
        if (cksum == ch) /*!< 校准正确返回1 */
        {
            rxPacket.checkSum = cksum;
            return 1;
        }
        else /*!< 校验错误 */
        {
            rxState = waitForStartByte1;
        }

        rxState = waitForStartByte1;
        break;

    default:
        rxState = waitForStartByte1;
        break;
    }

    return 0;
}

/**
 * @brief  ATKP数据包解析
 * @param  packet: atkp数据包
 * @retval None
 */
void atkpParsing(atkp_t *packet)
{
    /* 姿态角 */
    if (packet->msgID == UP_ATTITUDE)
    {
        int16_t data = (int16_t)(packet->data[1] << 8) | packet->data[0];
        attitude.roll = (float)(data / 32768.0 * 180.0);//在相关乘除中强调数据格式

        data = (int16_t)(packet->data[3] << 8) | packet->data[2];
        attitude.pitch = (float)(data / 32768.0 * 180.0);

        data = (int16_t)(packet->data[5] << 8) | packet->data[4];
        attitude.yaw = (float)(data / 32768.0 * 180.0);
    }
    /*****已经在上位机设置了发送的内容，所以下面的内容不会运行*****/
    /* 四元数 */
    else if (packet->msgID == UP_QUAT)
    {
        int16_t data = (int16_t)(packet->data[1] << 8) | packet->data[0];
        quaternion.q0 = (float)data / 32768;

        data = (int16_t)(packet->data[3] << 8) | packet->data[2];
        quaternion.q1 = (float)data / 32768;

        data = (int16_t)(packet->data[5] << 8) | packet->data[4];
        quaternion.q2 = (float)data / 32768;

        data = (int16_t)(packet->data[7] << 8) | packet->data[6];
        quaternion.q3 = (float)data / 32768;
    }

    /* 陀螺仪加速度数据 */
    else if (packet->msgID == UP_GYROACCDATA)
    {
        gyroAccData.acc[0] = (int16_t)(packet->data[1] << 8) | packet->data[0];
        gyroAccData.acc[1] = (int16_t)(packet->data[3] << 8) | packet->data[2];
        gyroAccData.acc[2] = (int16_t)(packet->data[5] << 8) | packet->data[4];

        gyroAccData.gyro[0] = (int16_t)(packet->data[7] << 8) | packet->data[6];
        gyroAccData.gyro[1] = (int16_t)(packet->data[9] << 8) | packet->data[8];
        gyroAccData.gyro[2] = (int16_t)(packet->data[11] << 8) | packet->data[10];

        gyroAccData.faccG[0] = (float)gyroAccData.acc[0] / 32768 * accFsrTable[imu901Param.accFsr]; /*!< 4代表4G，上位机设置好的量程 */
        gyroAccData.faccG[1] = (float)gyroAccData.acc[1] / 32768 * accFsrTable[imu901Param.accFsr];
        gyroAccData.faccG[2] = (float)gyroAccData.acc[2] / 32768 * accFsrTable[imu901Param.accFsr];

        gyroAccData.fgyroD[0] = (float)gyroAccData.gyro[0] / 32768 * gyroFsrTable[imu901Param.gyroFsr]; /*!< 2000代表2000°/S，上位机设置好的量程 */
        gyroAccData.fgyroD[1] = (float)gyroAccData.gyro[1] / 32768 * gyroFsrTable[imu901Param.gyroFsr];
        gyroAccData.fgyroD[2] = (float)gyroAccData.gyro[2] / 32768 * gyroFsrTable[imu901Param.gyroFsr];
    }
#ifdef IMU901
    /* 磁场数据 */
    else if (packet->msgID == UP_MAGDATA)
    {
        magData.mag[0] = (int16_t)(packet->data[1] << 8) | packet->data[0];
        magData.mag[1] = (int16_t)(packet->data[3] << 8) | packet->data[2];
        magData.mag[2] = (int16_t)(packet->data[5] << 8) | packet->data[4];

        int16_t data = (int16_t)(packet->data[7] << 8) | packet->data[6];
        magData.temp = (float)data / 100;
    }

    /* 气压计数据 */
    else if (packet->msgID == UP_BARODATA)
    {
        baroData.pressure = (int32_t)(packet->data[3] << 24) | (packet->data[2] << 16) |
                            (packet->data[1] << 8) | packet->data[0];

        baroData.altitude = (int32_t)(packet->data[7] << 24) | (packet->data[6] << 16) |
                            (packet->data[5] << 8) | packet->data[4];

        int16_t data = (int16_t)(packet->data[9] << 8) | packet->data[8];
        baroData.temp = (float)data / 100;
    }
#endif

    /* 端口状态数据 */
    else if (packet->msgID == UP_D03DATA)
    {
        iostatus.d03data[0] = (uint16_t)(packet->data[1] << 8) | packet->data[0];
        iostatus.d03data[1] = (uint16_t)(packet->data[3] << 8) | packet->data[2];
        iostatus.d03data[2] = (uint16_t)(packet->data[5] << 8) | packet->data[4];
        iostatus.d03data[3] = (uint16_t)(packet->data[7] << 8) | packet->data[6];
    }
}

#ifdef REG_Action
/**
 * @brief  写寄存器
 * @param  reg: 寄存器列表地址
 * @param  data: 数据
 * @param  datalen: 数据的长度只能是 1或2
 * @retval None
 */
void atkpWriteReg(enum regTable reg, uint16_t data, uint8_t datalen)
{
    uint8_t buf[7];

    buf[0] = 0x55;
    buf[1] = 0xAF;
    buf[2] = reg;
    buf[3] = datalen; /*!< datalen只能是1或者2 */
    buf[4] = data;

    if (datalen == 2)
    {
        buf[5] = data >> 8;
        buf[6] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5];
        imu901_uart_send(buf, 7);
    }
    else
    {
        buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
        imu901_uart_send(buf, 6);
    }
}

/**
 * @brief  发送读寄存器命令
 * @param  reg: 寄存器列表地址
 * @retval None
 */
static void atkpReadRegSend(enum regTable reg)
{
    uint8_t buf[7];

    buf[0] = 0x55;
    buf[1] = 0xAF;
    buf[2] = reg | 0x80;
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    imu901_uart_send(buf, 6);
}

/**
 * @brief  读寄存器
 * @param  reg: 寄存器地址
 * @param  data: 读取到的数据
 * @retval uint8_t: 0读取失败（超时） 1读取成功
 */
uint8_t atkpReadReg(enum regTable reg, int16_t *data)
{
    uint8_t ch;
    uint16_t timeout = 0;

    atkpReadRegSend(reg);

    while (1)
    {
        if (imu901_uart_receive(&ch, 1)) /*!< 获取串口fifo一个字节 */
        {
            if (imu901_unpack(ch)) /*!< 有有效数据包 */
            {
                if (rxPacket.startByte2 == UP_BYTE2) /*!< 主动上传包 */
                {
                    atkpParsing(&rxPacket);
                }
                else if (rxPacket.startByte2 == UP_BYTE2_ACK) /*!< 读寄存器应答包 */
                {
                    if (rxPacket.dataLen == 1)
                        *data = rxPacket.data[0];
                    else if (rxPacket.dataLen == 2)
                        *data = (rxPacket.data[1] << 8) | rxPacket.data[0];

                    return 1;
                }
            }
        }
        else
        {
            delay_ms(1);
            timeout++;

            if (timeout > 200) /*!< 超时返回 */
                return 0;
        }
    }
}

#endif
/**
 * @brief  模块初始化
 * @param  None
 * @retval None
 */
void imu901_init(void)
{
    ;
    /**
     *	 写入寄存器参数（测试）
     *	 这里提供写入引用例子，用户可以在这写入一些默认参数，
     *  如陀螺仪加速度量程、带宽、回传速率、PWM输出等。
     */
#ifdef REG_Action
    int16_t data;
    atkpWriteReg(REG_GYROFSR, 3, 1);
    atkpWriteReg(REG_ACCFSR, 1, 1);
    atkpWriteReg(REG_SAVE, 0, 1); /* 发送保存参数至模块内部Flash，否则模块掉电不保存 */

    /* 读出寄存器参数（测试） */
    atkpReadReg(REG_GYROFSR, &data);
    imu901Param.gyroFsr = data;

    atkpReadReg(REG_ACCFSR, &data);
    imu901Param.accFsr = data;

    atkpReadReg(REG_GYROBW, &data);
    imu901Param.gyroBW = data;

    atkpReadReg(REG_ACCBW, &data);
    imu901Param.accBW = data;
#endif
}
//使用ATK_601时  以上内容无需修改

/***********************自行编写的函数*************************************/
#include "imu_pid.h"                    //数据的运用
extern int if_OsRunning(void);          //用于获取系统运行状态
#define ABS(X) (((X) > 0) ? (X) : -(X)) //宏定义实现绝对值
uint32_t *reg_ptr;                      //数据寄存器指针

#define HAL 1
#define STD 1

bool imu_state = false;
void Set_IMUFinishedInf(bool state)
{
    imu_state = state;
}
bool Get_IMUFinishedInf(void)
{
    return imu_state;
}
ATK_IMU_t imu =
    {
        /*移植时只需要修改以下结构体变量即可*/

        .imu_uart = &huart3,        //串口号
        .yaw_ptr = &(attitude.yaw), //解析出来的原始数据的指针
        .target_angle = 0,          // pid的目标角度
        .init_angle = 0,            //初始化角度，补偿上电时的初始角度
        .enable_switch = 1,         //使能开关
        .get_angle = Get_Yaw        //函数指针，返回经过限幅和相对0的角度
};

#define ERROR_THRESHILD 0.05 //增大漂移限制
#define INIT_TIMES 150 //TODO 增大稳定程度

#define BUFFER_SIZE 11

uint8_t imu_cmd[12];

void Set_IMUStatus(int status)
{
    set_imu_status(status);
}

/**********************************************************************
 * @Name    IMU_IRQ
 * @declaration :陀螺仪的中断处理函数，处理DMA收到的数据,在DMA 开启的情况下，放在HAL_UART_RxCpltCallback中
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/

void IMU_IRQ(void)
{
    static uint8_t rec;
    rec = imu.imu_uart->Instance->RDR;
    if (imu901_unpack(rec))     //接收完成
        atkpParsing(&rxPacket); //开始解码，得到姿态角
}

/**********************************************************************
 * @Name    ATK_IMU_Init
 * @declaration : 初始化陀螺仪，开启传输，获取补偿角
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/

void ATK_IMU_Init(void)
{
    __HAL_UART_ENABLE_IT(imu.imu_uart, UART_IT_RXNE); //开启DMA
    Set_InitYaw(0);                                   //获取初始化角度，用于补偿上电时的角度,并设置当前角度为0度
    Set_IMUFinishedInf(true);                         // 7用于反馈此时陀螺仪已经初始化完成 可以进行任务运行
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
}

/**********************************************************************
 * @Name    Set_InitYaw
 * @declaration : 设置当前角度为xx度
 * @param   target: [输入/出]  想设置的角度值
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Set_InitYaw(int target)
{
    bool os_running = 0;
    Set_IMUStatus(false); //先把陀螺仪关掉，否则容易在修改过程中异常
    if (if_OsRunning())
        os_running = 1;
    imu.target_angle = angle_limit(target); //同步修改

    float last_yaw, current_yaw;
    int init_times = 0;

    current_yaw = last_yaw = *imu.yaw_ptr; //获取当前原生数据
    while (init_times < INIT_TIMES)        //陀螺仪未达到稳定
    {
        if (fabs(current_yaw - last_yaw) < ERROR_THRESHILD) //偏移很小
            init_times++;
        else
            init_times = 0; //飘了，清0，重置
        //更新数值
        last_yaw = current_yaw;
        current_yaw = *imu.yaw_ptr;
        // 10ms一次查询,根据系统运行结果执行延时函数。系统未运行时使用阻塞式，系统运行时使用系统的延时函数
        if (os_running)
            osDelay(10);
        else
            HAL_Delay(10);
    }
    //陀螺仪稳定，开始获取数据
    imu.init_angle = angle_limit(-angle_limit(current_yaw) + angle_limit(target));
    //恢复陀螺仪的使能状态
    Set_IMUStatus(true);
}

/**********************************************************************
 * @Name    Get_Yaw
 * @declaration :获取经过限幅的相对于上电位置的Yaw角
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/

float Get_Yaw(void)
{
    float angle = *(imu.yaw_ptr) + imu.init_angle; //获取当前原生数据加上补偿角
    return angle_limit(angle);
}
float Get_Raw_Yaw(void)
{
	float raw_yaw= *(imu.yaw_ptr);
	return angle_limit(raw_yaw);
}
/**********************************************************************
 * @Name    angle_limit
 * @declaration :
 * @param   angle: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

float angle_limit(float angle)
{
    //把传进来的角度限制在正负180范围

  limit_label:
    while (angle > 180)
        angle -= 360;
    while (angle < -180)
        angle += 360;
    if (ABS(angle) > 180)
        goto limit_label; //意义不大，但是避免出错
    
    return angle;
}

/**********************************************************************
 * @Name    Get_IMUStatus
 * @declaration : 获取陀螺仪使能状态的接口
 * @param   None
 * @retval   : 陀螺仪使能状态
 * @author  peach99CPP
 ***********************************************************************/
int Get_IMUStatus(void)
{
    return imu.enable_switch;
}
/*******************************END OF FILE************************************/
