#include "openmv.h"
#include "servo.h"
#include "chassis.h"
#include "QR_code.h"
#include "read_status.h"
#include "general.h"
#include "uart_handle.h"

#define STOP_SIGNAL 0XAABB
short Handle_Flag = 0;
int mv_param;
short AcIDofBar = 1;
int temp_ = 0;          //动作组计数器
short mv_stop_flag = 0; //判断MV舵控的工作状态
int Disc_Count = 0;
int rectangle_count = 0;

osThreadId MV_QuertThreadHandle;
bool MV_QueryTask_Exit = true;
bool MV_Query_Stae = false;
uint8_t Rest_QueryTimes = 0;
int YuanPan_first_flag = 1;

void Set_FirstFlag(int state)
{
    YuanPan_first_flag = state;
}
int Get_FirstFlag(void)
{
    return YuanPan_first_flag;
}
/**
 * @description:  开启MV的任务
 * @return {*}
 */
void MV_QueryTask_Start(void)
{
    if (MV_QueryTask_Exit)
    {
        MV_QueryTask_Exit = false;
        MV_Query_Stae = true;
        osThreadDef(MV_QuertThreadHandle, MV_QueryTaskFunc, osPriorityHigh, 0, 256);
        MV_QuertThreadHandle = osThreadCreate(osThread(MV_QuertThreadHandle), NULL);
    }
}
void Exit_MV_QueryTask(void)
{
    MV_QueryTask_Exit = true;
}
void MV_QueryTaskFunc(void const *argument)
{
    while (!MV_QueryTask_Exit)
    {
        //设置开始定时发送
        if (MV_Query_Stae)
        {
            MV_SendCmd(9, 0); //向openmv发送查询指令
            osDelay(100);     // 10HZ
        }
        else
        {
            osDelay(10); //高频率刷新
        }
    }
    vTaskDelete(NULL);
}
void Set_QueryState(int state)
{
    MV_Query_Stae = state;
    if (state == 1)
    {
        if (MV_QueryTask_Exit)
        {
            MV_QueryTask_Start();
        }
    }
}

void Update_rectangle_count(void)
{
    rectangle_count += 1;
}
int Get_RecCount(void)
{
    return rectangle_count;
}
mvrec_t mv_rec; // mv的结构体
mv_t MV =       //为结构体赋初值
    {
        .mv_uart = &huart4,
        .enable_switch = true,
        .mv_cmd = {0},
        .rec_buffer = {0},
        .rec_len = 0,
        .RX_Status = 0}; //初始化变量

volatile int disc_countval = 0, color_val = 0;

void Set_AcIDofBar(short change)
{
    AcIDofBar = change;
}
void Disc_Report(void)
{
    disc_countval++;
    printf("\n\t圆盘机拨球,当前目标%d,次数：%d \t\n", color_val, disc_countval);
    if ((disc_countval >= 8 && color_val == 0) || (disc_countval >= 4 && color_val == 1))
    {
        disc_countval = 0;
        color_val++;
    }
}

void Set_MV_Mode(bool mode)
{
    MV.enable_switch = mode;
}
bool Get_MV_Mode(void)
{
    return MV.enable_switch;
}
/**********************************************************************
 * @Name    cmd_encode
 * @declaration : 根据协议编码发送的内容
 * @param   event_id: [输入/出]  事件的类型
 **			 param: [输入/出]     参数，16位数字
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void cmd_encode(const uint8_t event_id, uint8_t param)
{

    //定义通讯协议
    MV.mv_cmd[0] = START_BYTE; //帧头
    MV.mv_cmd[1] = event_id;   //触发的事件id
    MV.mv_cmd[2] = param;
    MV.mv_cmd[3] = (uint8_t)(event_id + param); //和校验
    MV.mv_cmd[4] = END_BYTE;                    //帧尾
}
void MV_SendCmd(const uint8_t event_id, const int param)
{
    cmd_encode(event_id, param);                                 //根据获得的参数编码cmd数组
    HAL_UART_Transmit(MV.mv_uart, MV.mv_cmd, BUFFER_SIZE, 0xff); //将cmd发送出去
    memset(MV.mv_cmd, 0, sizeof(MV.mv_cmd));                     //将cmd数组重新初始化
}

/**********************************************************************
 * @Name    MV_IRQ
 * @declaration :  openmv通讯的中断处理函数
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void MV_IRQ(void)
{
    uint8_t rec_data = MV.mv_uart->Instance->RDR;
    if (MV.RX_Status == 0)
    {
        if (rec_data == START_BYTE)
        {
            MV.RX_Status = 1; //读到帧头，做标记，直接退出
            MV.rec_len = 0;
            return;
        }
    }
    else if (MV.RX_Status == 1)
    {
        if (rec_data == END_BYTE && MV.rec_len == BUFFER_SIZE - 2)
        {
            MV_rec_decode();
            MV.RX_Status = 0;
        }
        else
        {
            MV.rec_buffer[MV.rec_len++] = rec_data; //存入数组
            if (MV.rec_len == MAX_REC_SIZE)
            {
                MV.RX_Status = 0; //防止因为出错导致卡死
                MV.rec_len = 0;
                memset(MV.rec_buffer, 0, sizeof(MV.rec_buffer));
            }
        }
    }
}

/**********************************************************************
 * @Name    MV_rec_decode
 * @declaration : 判断接收完成后，对接收的内容进行解码
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void MV_rec_decode(void)
{
    if (MV.rec_buffer[0] + MV.rec_buffer[1] == MV.rec_buffer[2])
    {
        mv_rec.event = MV.rec_buffer[0];
        mv_rec.param = MV.rec_buffer[1];
        MV_Decode();
        mv_rec.event = 0, mv_rec.param = 0; //使用完就清除
        MV.rec_len = 0;                     //重置
        MV.RX_Status = 0;
    }
    //处理完之后记得重新初始化结构体中的rec_len和RX_status变量，避免出错
}

/****上面是底层实现，下面是上层的应用****/

/**********************************************************************
 * @Name    MV_Decode
 * @declaration :根据自己定义的参数含义执行命令
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void MV_Decode(void)
{
#define Catch_ 1
#define MVPID_THRESHOLD 10
#define pid_p 0.5
#define Ball_Signal 0x01
#define Rectangle_Signal 0x02
#define Ring_Signal 0x03
#define BAR_Signal 0x09
#define BAR_Action 30
#define MV_BACK 0x66
#define TarDiscId 0x08
#define YelDiscId 0x09
#define TarDisc 40
#define YelDisc 41

#define MV_Blob 0X02
    if (Get_Servo_Flag()) //空闲，可以接收指令 此时openmv和舵控都准备好执行指令
    {
        if (mv_rec.event == MV_Blob)
        {
            if (Get_FirstFlag())
            {
                printf("运行屁用没有动作组,跳过1S\n");
                Set_FirstFlag(0);
                ActionGroup(15, 1);
            }
            else
            {
                ActionGroup(3, 1);
            }
            // printf("拨球\n");
        }
    }

#if use_old
    if (Get_MV_Mode()) //只有此时mv是对信号响应，才进入下面的逻辑判断
    {
        if (Get_Servo_Flag()) //空闲，可以接收指令 此时openmv和舵控都准备好执行指令
        {
            if (mv_rec.event == MV_Blob)
            {
                // ActionGroup(3, 1);
                printf("拨球\n");
            }

            static int mode;
            mode = Get_TargetColor(); // 1红2蓝
            if (mv_rec.event == Ball_Signal)
            {
                Set_HeightAvailable(false); //在下面的操作中对高度红外的读取进行屏蔽
                Disable_ServoFlag();        //标记此时舵控正在运行过程中 防止此时车子那边还在运行循环
                Enable_StopSignal();        //使能停车信号，让动作那边执行停车操作
                printf("要抓球\r\n");
                switch (Get_Height()) //获取当前的高度信息，根据高度不同执行不同的动作组
                {
                case LowestHeight:
                    Action_Gruop(Lowest, 1);
                    temp_++;
                    break;
                case MediumHeight:
                    Action_Gruop(Medium, 1);
                    temp_++;
                    break;
                case HighestHeight:
                    Action_Gruop(Highest, 1);
                    temp_++;
                    break;
                default:;
                }
            }
            else if (mv_rec.event == Ring_Signal)
            {
                Set_QR_Status(false);       //关闭此时二维码那边的响应操作 todo 防止误响应
                Set_HeightAvailable(false); //在下面的操作中对高度红外的读取进行屏蔽
                Disable_ServoFlag();        //标记此时舵控正在运行过程中 防止此时车子那边还在运行循环
                Enable_StopSignal();        //使能停车信号,此时进入停车状态，等待动作组执行完毕
                printf("发现圆环了\r\n");
                switch (Get_Height()) //获取当前的高度信息，根据高度不同执行不同的动作组
                {
                case LowestHeight:
                    Action_Gruop(LowestRing, 1);
                    temp_++;
                    break;
                case MediumHeight:
                    Action_Gruop(MediumRing, 1);
                    temp_++;
                    break;
                case HighestHeight:
                    Action_Gruop(HighestRing, 1);
                    temp_++;
                    break;
                default:;
                }
            }
            else if (mv_rec.event == Rectangle_Signal)
            {
                Set_QR_Status(false);       //关闭此时二维码那边的响应操作 todo 防止误响应
                Set_HeightAvailable(false); //在下面的操作中对高度红外的读取进行屏蔽
                Disable_ServoFlag();        //标记此时舵控正在运行过程中，本函数在传输舵控指令中也会被调用，
                Enable_StopSignal();        //使能停车信号，让动作那边执行停车操作
                printf("扫到矩形\r\n");     //打印调试信息
                Update_rectangle_count();   //更新数值
                int height = Get_Height();
                switch (height) //根据当前高度来执行不同的动作组
                {
                case LowestHeight:
                    Action_Gruop(Lowest, 1);
                    temp_ += 1;
                    break;
                case MediumHeight:
                    Action_Gruop(Medium, 1);
                    temp_ += 1;
                    break;
                case HighestHeight:
                    Action_Gruop(Highest, 1);
                    temp_ += 1;
                    break;
                default:;
                }
            }
            else if (mv_rec.event == TarDiscId)
            {
                // if (++Disc_Count < 3)
                // {
                //     printf("\t\n过滤本次拨球\n");
                //     return;
                // }
                Disable_ServoFlag(); //只需要清除舵机标志位即可 不需要停车
                if (mode == 1)
                {
                    printf("红色球\n");
                    Action_Gruop(46, 1);
                }
                else if (mode == 2)
                {
                    printf("蓝色球\n");
                    Action_Gruop(TarDisc, 1); //执行拨球动作组
                }
            }
            else if (mv_rec.event == YelDiscId)
            {
                // if (++Disc_Count < 3)
                // {
                //     printf("\t\n过滤本次拨球\n");
                //     return;
                // }
                Disable_ServoFlag(); //只需要清除舵机标志位即可 不需要停车
                if (mode == 1)
                {
                    printf("红半场黄球\n");
                    Action_Gruop(47, 1);
                }
                else if (mode == 2)
                {
                    printf("蓝半场黄球\n");
                    Action_Gruop(YelDisc, 1); //执行拨球动作组
                }
            }
            else if (mv_rec.event == BAR_Signal) //自从修改爪子的形状之后  拨球的行为直接由舵控进行控制 因此需要对MV发回来的数据会进行解析处理
            {
                //主要区别在于此处不需要停车进行等待，所以只需清除舵控的标志位即可。
                Disable_ServoFlag();         //只需要清除舵机标志位即可 不需要停车
                Action_Gruop(BAR_Action, 1); //执行拨球动作组
                Disc_Report();
            }
            else
            {
                printf("收到回应指令 id: %d\tparam: %d\n", mv_rec.event, mv_rec.param); //针对回应指令做特殊处理
            }
        }
    }
#endif
}

/**********************************************************************
 * @Name    Get_Stop_Signal
 * @declaration :返回此时是否停止的信号
 * @param   None
 * @retval   : 是否应该停车，停车则为1
 * @author  peach99CPP
 ***********************************************************************/
int Get_Stop_Signal(void)
{
    return mv_stop_flag;
}

/**********************************************************************
 * @Name    Enable_StopSignal
 * @declaration :使能停车的标志位
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Enable_StopSignal(void)
{
    mv_stop_flag = 1;
}

/**********************************************************************
 * @Name    Disable_StopSignal
 * @declaration : 清除停车标志位
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Disable_StopSignal(void)
{
    mv_stop_flag = 0;
}

/**********************************************************************
 * @Name    MV_Start
 * @declaration : Mv开始响应命令
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void MV_Start(void)
{
    Set_MV_Mode(true);
    MV_SendCmd(1, 0);
}

/**********************************************************************
 * @Name    MV_Stop
 * @declaration : 向MV发送停止信号
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void MV_Stop(void)
{
    Set_MV_Mode(false);
    MV_SendCmd(0, 0);
}
void OpenMV_ChangeRoi(int roi)
{
    MV_SendCmd(11, roi);
    osDelay(100);
}
