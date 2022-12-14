#include "QR_code.h"
#include "servo.h"
#include "openmv.h"
#include "read_status.h "

uint8_t head_cmd[4] = {'H', 'E', 'A', 'D'}; //协议的开头
uint8_t tail_cmd[4] = {'T', 'A', 'I', 'L'};
#define QR_BUFFER_SIZE 9                //一帧数据有多少个字节
#define BUFFER_END (QR_BUFFER_SIZE - 1) //数据的结尾，下标形式

QRcolor_t Target_Color = init_status;

extern UART_HandleTypeDef huart3; //避免出现不必要的警告

int count_QR_Times = 0; //记录二维码抓取次数
bool temp_var = 0;
int update_Qr = 0;
bool Get_temp_var(void)
{
    return temp_var;
}
void Set_temp_var(bool state)
{
    temp_var = false;
}
void SetUpdateQR(void)
{
    update_Qr++;
}
void Set_QrCount(int val)
{
    count_QR_Times = val;
}
int Get_UpdateQr(void)
{
    return update_Qr;
}
QR_t QR =
    {
        .QR_uart = &huart3,
        .enable_switch = false,
        .rec_len = 0,
        .RX_OK = 0,
        .RX_data = {0},
        .color = init_status}; //创建结构体并赋值=初值

/**********************************************************************
 * @Name    QR_receive
 * @declaration : 将该函数放在串口的IRQhandler中进行执行，用于处理数据的接收
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void QR_receive(void)
{
    static uint8_t rec, head_flag = 0;
    if (__HAL_UART_GET_FLAG(QR.QR_uart, UART_FLAG_RXNE))
    {
        rec = QR.QR_uart->Instance->RDR;
        if (!QR.RX_OK) //接收未完成
        {
            if (rec == 'H' && head_flag == 0)
            {
                head_flag = 1;
                QR.RX_data[QR.rec_len++] = rec; //确认收到头
                return;
            }
            if (head_flag)
            {
                QR.RX_data[QR.rec_len++] = rec;
                if (QR.rec_len >= QR_BUFFER_SIZE) //读取到的个数达到要求
                {
                    QR.RX_OK = 1; //接收完成
                    printf("二维码接收完成，进入解码阶段\n");
                    QR_decode(); //开始解析
                    head_flag = 0;
                }
            }
        }
    }
}

/**********************************************************************
 * @Name    QR_decode
 * @declaration : 该函数用于将上一函数收到的数据进行解析吗，得到其中的数据
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void QR_decode(void)
{
#define MAX_QR_TIMES 2
#define INF_INDEX 4
    //判断帧头帧尾是否检验通过
    if (QR.RX_data[0] == head_cmd[0] &&
        QR.RX_data[1] == head_cmd[1] &&
        QR.RX_data[2] == head_cmd[2] &&
        QR.RX_data[3] == head_cmd[3] &&
        QR.RX_data[BUFFER_END - 3] == tail_cmd[0] &&
        QR.RX_data[BUFFER_END - 2] == tail_cmd[1] &&
        QR.RX_data[BUFFER_END - 1] == tail_cmd[2] &&
        QR.RX_data[BUFFER_END] == tail_cmd[3])
    {
        //解析本次获得的数据
        if (QR.RX_data[INF_INDEX] == 'R' || QR.RX_data[INF_INDEX] == 'r')
        {
            QR.color = red; //红色
            printf("红色色块\n");
        }
        else if (QR.RX_data[INF_INDEX] == 'B' || QR.RX_data[INF_INDEX] == 'b')
        {
            QR.color = blue; //蓝色
            printf("蓝色色块\n");
        }
        else
        {
            QR.color = init_status; //接收到了其他数据，此状态既是初始状态又是错误标记
            printf("收到错误数据，请check\n");
        }
    }
    if ((QR.color == Target_Color) && Return_QRMode() && (QR.color != init_status)) //仅在设置了对其有响应才会执行 颜色要对 并且开启响应
    {
        Set_MV_Mode(false);   //临时关闭openmv信号的处理
        if (Get_Servo_Flag()) //确保舵控可用
        {
            Set_HeightAvailable(false); //禁用高度变换
            Disable_ServoFlag();        // todo 此处需要确保先disable防止后面
            Enable_StopSignal();        //使能停车信号，让动作那边执行停车操作
            printf("\n\t二维码发现目标物块\t\n");
            switch (Get_Height()) //获取当前的高度信息，根据高度不同执行不同的动作组
            {
            case LowestHeight:
                Action_Gruop(1, 1);
                break;
            case MediumHeight:
                Action_Gruop(2, 1);
                break;
            case HighestHeight:
                Action_Gruop(3, 1);
            default:;
            }
            Update_rectangle_count(); //二维码也是矩形 所以更新数据
            SetUpdateQR();            //更新二维码数据
            if (++count_QR_Times >= MAX_QR_TIMES)
            {
                count_QR_Times = 0;
                // Set_temp_var(true);
                // Set_HeightAvailable(true);
                // Enable_StopSignal();
            }
        }
        else
        {
            printf("舵控繁忙\n");
        }
    }
    QR.rec_len = 0;                            //清除计数器
    QR.RX_OK = 0;                              //清空标志位
    memset(QR.RX_data, 0, sizeof(QR.RX_data)); //把缓存的内容全部清除
}
/******以下是数据对外的接口********/

bool Return_QRMode(void)
{
    return QR.enable_switch;
}

/**********************************************************************
 * @Name    Get_QRColor
 * @declaration :获取当前二维码的目标颜色
 * @param   None
 * @retval   : void
 * @author  peach99CPP
 ***********************************************************************/
int Get_QRColor(void)
{
    if (QR.color != init_status)
    {
        if (QR.color == red)
        {
            return red;
        }
        else if (QR.color == blue)
        {
            return blue;
        }
    }
    return init_status;
}

void QR_Mode_Init(bool status, QRcolor_t target_color)
{
    Set_QR_Status(status);
    Set_QR_Target(target_color);
}
/**********************************************************************
 * @Name    Set_QR_Status
 * @declaration : 设置是否对二维码内容响应
 * @param   status: [输入/出] 是否开启
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Set_QR_Status(bool status)
{
    //设置是否对二维码反馈信息执行动作组响应
    QR.enable_switch = status; // todo 此时修正了重置目标的bug
}

/**********************************************************************
 * @Name    Set_QR_Target
 * @declaration : 设置二维码的目标颜色
 * @param   color: [输入/出] 1 代表红色  2代表蓝色
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Set_QR_Target(QRcolor_t color)
{
    if (Return_QRMode())
    {
        if (color != init_status)
        {

            if (color == 1)
                Target_Color = red;
            else if (color == 2)
                Target_Color = blue;
            else
                Target_Color = init_status;
        }
    }
    else
        printf("\n二维码未使能，运行Set_QR_Status\n");
}

/**********************************************************************
 * @Name    DeInit_QRColor
 * @declaration :重新初始化二维码的颜色
 * @param   None
 * @retval   :无
 * @author  peach99CPP
 ***********************************************************************/
void DeInit_QRColor(void)
{
    QR.color = init_status;
}
