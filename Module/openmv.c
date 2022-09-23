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
int temp_ = 0;          //�����������
short mv_stop_flag = 0; //�ж�MV��صĹ���״̬
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
 * @description:  ����MV������
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
        //���ÿ�ʼ��ʱ����
        if (MV_Query_Stae)
        {
            MV_SendCmd(9, 0); //��openmv���Ͳ�ѯָ��
            osDelay(100);     // 10HZ
        }
        else
        {
            osDelay(10); //��Ƶ��ˢ��
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
mvrec_t mv_rec; // mv�Ľṹ��
mv_t MV =       //Ϊ�ṹ�帳��ֵ
    {
        .mv_uart = &huart4,
        .enable_switch = true,
        .mv_cmd = {0},
        .rec_buffer = {0},
        .rec_len = 0,
        .RX_Status = 0}; //��ʼ������

volatile int disc_countval = 0, color_val = 0;

void Set_AcIDofBar(short change)
{
    AcIDofBar = change;
}
void Disc_Report(void)
{
    disc_countval++;
    printf("\n\tԲ�̻�����,��ǰĿ��%d,������%d \t\n", color_val, disc_countval);
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
 * @declaration : ����Э����뷢�͵�����
 * @param   event_id: [����/��]  �¼�������
 **			 param: [����/��]     ������16λ����
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void cmd_encode(const uint8_t event_id, uint8_t param)
{

    //����ͨѶЭ��
    MV.mv_cmd[0] = START_BYTE; //֡ͷ
    MV.mv_cmd[1] = event_id;   //�������¼�id
    MV.mv_cmd[2] = param;
    MV.mv_cmd[3] = (uint8_t)(event_id + param); //��У��
    MV.mv_cmd[4] = END_BYTE;                    //֡β
}
void MV_SendCmd(const uint8_t event_id, const int param)
{
    cmd_encode(event_id, param);                                 //���ݻ�õĲ�������cmd����
    HAL_UART_Transmit(MV.mv_uart, MV.mv_cmd, BUFFER_SIZE, 0xff); //��cmd���ͳ�ȥ
    memset(MV.mv_cmd, 0, sizeof(MV.mv_cmd));                     //��cmd�������³�ʼ��
}

/**********************************************************************
 * @Name    MV_IRQ
 * @declaration :  openmvͨѶ���жϴ�����
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void MV_IRQ(void)
{
    uint8_t rec_data = MV.mv_uart->Instance->RDR;
    if (MV.RX_Status == 0)
    {
        if (rec_data == START_BYTE)
        {
            MV.RX_Status = 1; //����֡ͷ������ǣ�ֱ���˳�
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
            MV.rec_buffer[MV.rec_len++] = rec_data; //��������
            if (MV.rec_len == MAX_REC_SIZE)
            {
                MV.RX_Status = 0; //��ֹ��Ϊ�����¿���
                MV.rec_len = 0;
                memset(MV.rec_buffer, 0, sizeof(MV.rec_buffer));
            }
        }
    }
}

/**********************************************************************
 * @Name    MV_rec_decode
 * @declaration : �жϽ�����ɺ󣬶Խ��յ����ݽ��н���
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void MV_rec_decode(void)
{
    if (MV.rec_buffer[0] + MV.rec_buffer[1] == MV.rec_buffer[2])
    {
        mv_rec.event = MV.rec_buffer[0];
        mv_rec.param = MV.rec_buffer[1];
        MV_Decode();
        mv_rec.event = 0, mv_rec.param = 0; //ʹ��������
        MV.rec_len = 0;                     //����
        MV.RX_Status = 0;
    }
    //������֮��ǵ����³�ʼ���ṹ���е�rec_len��RX_status�������������
}

/****�����ǵײ�ʵ�֣��������ϲ��Ӧ��****/

/**********************************************************************
 * @Name    MV_Decode
 * @declaration :�����Լ�����Ĳ�������ִ������
 * @param   None
 * @retval   : ��
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
    if (Get_Servo_Flag()) //���У����Խ���ָ�� ��ʱopenmv�Ͷ�ض�׼����ִ��ָ��
    {
        if (mv_rec.event == MV_Blob)
        {
            if (Get_FirstFlag())
            {
                printf("����ƨ��û�ж�����,����1S\n");
                Set_FirstFlag(0);
                ActionGroup(15, 1);
            }
            else
            {
                ActionGroup(3, 1);
            }
            // printf("����\n");
        }
    }

#if use_old
    if (Get_MV_Mode()) //ֻ�д�ʱmv�Ƕ��ź���Ӧ���Ž���������߼��ж�
    {
        if (Get_Servo_Flag()) //���У����Խ���ָ�� ��ʱopenmv�Ͷ�ض�׼����ִ��ָ��
        {
            if (mv_rec.event == MV_Blob)
            {
                // ActionGroup(3, 1);
                printf("����\n");
            }

            static int mode;
            mode = Get_TargetColor(); // 1��2��
            if (mv_rec.event == Ball_Signal)
            {
                Set_HeightAvailable(false); //������Ĳ����жԸ߶Ⱥ���Ķ�ȡ��������
                Disable_ServoFlag();        //��Ǵ�ʱ����������й����� ��ֹ��ʱ�����Ǳ߻�������ѭ��
                Enable_StopSignal();        //ʹ��ͣ���źţ��ö����Ǳ�ִ��ͣ������
                printf("Ҫץ��\r\n");
                switch (Get_Height()) //��ȡ��ǰ�ĸ߶���Ϣ�����ݸ߶Ȳ�ִͬ�в�ͬ�Ķ�����
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
                Set_QR_Status(false);       //�رմ�ʱ��ά���Ǳߵ���Ӧ���� todo ��ֹ����Ӧ
                Set_HeightAvailable(false); //������Ĳ����жԸ߶Ⱥ���Ķ�ȡ��������
                Disable_ServoFlag();        //��Ǵ�ʱ����������й����� ��ֹ��ʱ�����Ǳ߻�������ѭ��
                Enable_StopSignal();        //ʹ��ͣ���ź�,��ʱ����ͣ��״̬���ȴ�������ִ�����
                printf("����Բ����\r\n");
                switch (Get_Height()) //��ȡ��ǰ�ĸ߶���Ϣ�����ݸ߶Ȳ�ִͬ�в�ͬ�Ķ�����
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
                Set_QR_Status(false);       //�رմ�ʱ��ά���Ǳߵ���Ӧ���� todo ��ֹ����Ӧ
                Set_HeightAvailable(false); //������Ĳ����жԸ߶Ⱥ���Ķ�ȡ��������
                Disable_ServoFlag();        //��Ǵ�ʱ����������й����У��������ڴ�����ָ����Ҳ�ᱻ���ã�
                Enable_StopSignal();        //ʹ��ͣ���źţ��ö����Ǳ�ִ��ͣ������
                printf("ɨ������\r\n");     //��ӡ������Ϣ
                Update_rectangle_count();   //������ֵ
                int height = Get_Height();
                switch (height) //���ݵ�ǰ�߶���ִ�в�ͬ�Ķ�����
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
                //     printf("\t\n���˱��β���\n");
                //     return;
                // }
                Disable_ServoFlag(); //ֻ��Ҫ��������־λ���� ����Ҫͣ��
                if (mode == 1)
                {
                    printf("��ɫ��\n");
                    Action_Gruop(46, 1);
                }
                else if (mode == 2)
                {
                    printf("��ɫ��\n");
                    Action_Gruop(TarDisc, 1); //ִ�в�������
                }
            }
            else if (mv_rec.event == YelDiscId)
            {
                // if (++Disc_Count < 3)
                // {
                //     printf("\t\n���˱��β���\n");
                //     return;
                // }
                Disable_ServoFlag(); //ֻ��Ҫ��������־λ���� ����Ҫͣ��
                if (mode == 1)
                {
                    printf("��볡����\n");
                    Action_Gruop(47, 1);
                }
                else if (mode == 2)
                {
                    printf("���볡����\n");
                    Action_Gruop(YelDisc, 1); //ִ�в�������
                }
            }
            else if (mv_rec.event == BAR_Signal) //�Դ��޸�צ�ӵ���״֮��  �������Ϊֱ���ɶ�ؽ��п��� �����Ҫ��MV�����������ݻ���н�������
            {
                //��Ҫ�������ڴ˴�����Ҫͣ�����еȴ�������ֻ�������صı�־λ���ɡ�
                Disable_ServoFlag();         //ֻ��Ҫ��������־λ���� ����Ҫͣ��
                Action_Gruop(BAR_Action, 1); //ִ�в�������
                Disc_Report();
            }
            else
            {
                printf("�յ���Ӧָ�� id: %d\tparam: %d\n", mv_rec.event, mv_rec.param); //��Ի�Ӧָ�������⴦��
            }
        }
    }
#endif
}

/**********************************************************************
 * @Name    Get_Stop_Signal
 * @declaration :���ش�ʱ�Ƿ�ֹͣ���ź�
 * @param   None
 * @retval   : �Ƿ�Ӧ��ͣ����ͣ����Ϊ1
 * @author  peach99CPP
 ***********************************************************************/
int Get_Stop_Signal(void)
{
    return mv_stop_flag;
}

/**********************************************************************
 * @Name    Enable_StopSignal
 * @declaration :ʹ��ͣ���ı�־λ
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Enable_StopSignal(void)
{
    mv_stop_flag = 1;
}

/**********************************************************************
 * @Name    Disable_StopSignal
 * @declaration : ���ͣ����־λ
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Disable_StopSignal(void)
{
    mv_stop_flag = 0;
}

/**********************************************************************
 * @Name    MV_Start
 * @declaration : Mv��ʼ��Ӧ����
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void MV_Start(void)
{
    Set_MV_Mode(true);
    MV_SendCmd(1, 0);
}

/**********************************************************************
 * @Name    MV_Stop
 * @declaration : ��MV����ֹͣ�ź�
 * @param   None
 * @retval   : ��
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
