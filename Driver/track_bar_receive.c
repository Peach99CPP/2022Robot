#include "track_bar_receive.h "
#include "usart.h"
#include "atk_imu.h"
#include <string.h>
#include "uart_handle.h"
#include "time_cnt.h"

extern int edge_status[3];
extern volatile uint32_t TIME_ISR_CNT;

int SW_data[9]; //�ᴥ����
int HW_data[4];
uint16_t SW_rec, HW_rec; //���⣬�ᴥ��������յ�������?
static bool hw_update_countLimit = false;
int US_data_H; //�������ź�
int US_data_L;
int US_data;
int US_data_H2; //�������ź�
int US_data_L2;
int US_data2;
#define START_BYTE 0xff
#define END_BYTE 0x99
#define MAX_LINE 3
#define BUFF_SIZE 16
#define LINE_DELAY (50 / 10)
#define MAX_time 500
#define BAR_CHANGE 0 //�Ƿ�Ҫ���̿��Ƶĸı�

int dma_count, times_counts = 0;
int count_avoid = 0;                         //Ҳ�Ƿ�ֹ����
bool SW_head, SW_right, SW_left, SW_end = 0; //���ִ������Ŀ���״̬
extern int ten_status[4];
//Ȩ��
float track_weight[8] = {4, 3, 2, 1,
                         -1, -2, -3, -4};

uint8_t rec_data, now_id = 0;            //�����������ڽ����ж���ʹ��
trackbar_t y_bar, x_leftbar, x_rightbar; //����Ѱ����ṹ�����
ALLData_t Boo, HW, SW, Boo2;

//��ʼ��PID����
pid_paramer_t track_pid_param = // pid����
    {
        .integrate_max = 50,
        .kp = 5,
        .ki = 0,
        .kd = 0,
        .control_output_limit = 300};

//��ʼ���ṹ��
Track_RXRows_t Track_Row =
    {
        .track_uart = &huart6,
        .current_index = 0,
        .done_flag = 0,
        .start_flag = 0,
        .rec_data = {0}};

void Update_TRckerCounter(void)
{
    times_counts = times_counts >= 32708 ? 0 : times_counts + 1;
    dma_count--;
}
int Get_TRckerCounter(void)
{
    return times_counts;
}
/**********************************************************************
 * @Name    Clear_Line
 * @declaration : ����ṹ��ı�־λ
 * @param   bar: [����/��] ��������
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void Clear_Line(trackbar_t *bar)
{
    bar->line_flag = 0;
    bar->line_num = 0;
}

void Clear_Boo(ALLData_t *some)
{
    some->end = 0;
    some->count = 0;
    some->Hight_num = 0;
    some->Low_num = 0;
    some->flag[0] = 0;
}
void turn_uint16(ALLData_t *some)
{
    some->end = Get_Uint8_Transform(some->Hight_num, some->Low_num);
}
int Get_Current_RowID(void)
{
    return now_id;
}

/**********************************************************************
 * @Name    Get_EmptyRow_ID
 * @declaration : ��ȡ���е��±�
 * @param   now_id: [����/��]  ����ֵ  ���ȸ��ݵ�ǰֵѡ������һ��
 * @retval   : �����±�
 * @author  peach99CPP
 ***********************************************************************/
int Get_EmptyRow_ID(int now_id)
{

    //���ȴ���һ�µ�ǰ�����ݣ�������һ���Ƿ����?
    if ((now_id + 1 == MAX_ROW_SIZE))
        now_id = 0;
    else
    {
        now_id += 1; //û�е���߽�?��ֱ��ȡ��һ��
    }

    if (Track_Row.rec_data[now_id][0] == 0) //�����һ�����ã���ֱ����? ���������±���
        return now_id;
    else
    {
        //������ά����
        for (uint8_t i = 0; i < MAX_ROW_SIZE; ++i)
        {
            if (Track_Row.rec_data[i][0] == 0)
            {
                return i;
            }
        }
    }
    return 0X01; //���⿨��
}

/**********************************************************************
 * @Name    Get_AvaibleRow_ID
 * @declaration :��ȡ�����ݵ��к��±�
 * @param   None
 * @retval   : װ������Ч���ݵ��±�
 * @author  peach99CPP
 ***********************************************************************/
int Get_AvaibleRow_ID(int now_row)
{
    //���ȸ��ݵ�ǰ�±�Ѱ��
    if (now_row == MAX_ROW_SIZE) //�Ѿ����ﾡͷ
    {
        now_row = 0;                                //��ô��һ������0
        if (Track_Row.rec_data[now_row][1] == 0X01) //����Ƿ����
            return now_row;
    }
    else
    {
        //�󲿷�����
        now_row += 1;                               //ֱ��ȡ��һλ
        if (Track_Row.rec_data[now_row][1] == 0X01) //�ж��Ƿ��Ѿ�װ��������
            return now_row;
        else //��������δ�ҵ� ���ȱ���
        {
            for (uint8_t i = 0; i < MAX_ROW_SIZE; ++i) //��ͷ������β
            {
                if (Track_Row.rec_data[i][1] == 0X01) //���?
                {
                    return i;
                }
            }
        }
    }
    return 0X01; //ʵ���Ҳ����ˣ���㷵��һ������ֹ������?
}

/**********************************************************************
 * @Name    set_track_pid
 * @declaration : ����Ѱ�����PID����API
 * @param   kp: [����/��] �Ŵ�10����p
 **			 ki: [����/��] �Ŵ�10����i
 **			 kd: [����/��] �Ŵ�10����d
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void set_track_pid(int kp, int ki, int kd)
{
    track_pid_param.kp = kp / 10.0;
    track_pid_param.ki = ki / 10.0;
    track_pid_param.kd = kd / 10.0;
}

/**********************************************************************
 * @Name    track_bar_init
 * @declaration : ��Ѱ������Ҫ����ر������г�ʼ��������?
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
void track_bar_init(void) //��صĳ�ʼ������?
{
    dma_count = 0;
    //���������ĳ�ʼ��
    y_bar.id = forward_bar; //��ע����
    y_bar.line_num = 0;
    y_bar.num = 0;
    y_bar.line_num = 0;
    y_bar.data.expect = 0;  //ѭ����Ŀ��ֵ��Ϊ0
    y_bar.if_switch = true; //ʹ��

    x_leftbar.id = left_bar;
    x_leftbar.line_num = 0;
    x_leftbar.num = 0;
    x_leftbar.line_num = 0;
    x_leftbar.data.expect = 0;
    x_leftbar.if_switch = true;

    x_rightbar.id = right_bar;
    x_rightbar.line_num = 0;
    x_rightbar.num = 0;
    x_rightbar.line_num = 0;
    x_rightbar.data.expect = 0;
    x_rightbar.if_switch = true;
    __HAL_UART_ENABLE_IT(Track_Row.track_uart, UART_IT_RXNE);
}

/**********************************************************************
 * @Name    track_decode
 * @declaration :��DMA�յ������ݽ��н������?ʵ��ѭ������ܵĺ��Ĵ���
 * @param   None
 * @retval   : ��
 * @author  peach99CPP
 ***********************************************************************/
#if BAR_CHANGE == 1
void track_decode(void)
{
    /***��غ�?��****/
#define EDGE_THRESHOLD 5 //�ڱ�Ե����ģʽ�£����ŵ�����ʱΪ��Ч����
#define NUM_THRESHOLD 6  //�Ǳ�Ե�߼����£��жϵ����ߵ�����
#define MIN_NUM 2        //��ѹ��֮�󣬹����˲Ż����һ���ߣ����ݵƵ��������м���?
#define EDGE_VAL 7       //��Ե����״̬�µ�ѭ����������ֵ

    Update_TRckerCounter();                         //������صļ�����?
    static uint8_t led_num = 0;                     //����������ı���?
    static uint8_t AVaiable_Row;                    //���õ��±�
    AVaiable_Row = Get_AvaibleRow_ID(AVaiable_Row); //����λ�ø���
    float track_value = 0, temp_val;                //��صı�������?
    if (AVaiable_Row != 0XFF)
    {
        //����ĺͼ��鿴�������
        if ((uint8_t)(Track_Row.rec_data[AVaiable_Row][1] + Track_Row.rec_data[AVaiable_Row][3] + Track_Row.rec_data[AVaiable_Row][5]) == Track_Row.rec_data[AVaiable_Row][6]) //��У��
        {
            for (uint8_t bar_id = 1; bar_id <= BUFF_SIZE - 2; bar_id += 2)
            {
                track_value = 0;
                temp_val = 0;
                led_num = 0;
                for (uint8_t i = 0; i < 8; ++i)
                {
                    temp_val = (bool)(((Track_Row.rec_data[AVaiable_Row][bar_id] << i) & 0x80)) * track_weight[i]; //���ݵ���������?�صõ�����ֵ
                    if (temp_val != 0)
                        led_num++; //�������ĵ�����
                    track_value += temp_val;
                }
                switch (Track_Row.rec_data[AVaiable_Row][bar_id - 1]) //�ж�Ѱ����ID
                {
                case 1:
                    y_bar.data.feedback = track_value; //��ֵ
                    y_bar.num = led_num;               //�õ��Ƶ�����
                    if (y_bar.num >= NUM_THRESHOLD || (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL))
                    {
                        /*�����������?
                         *һ�ǵ����ڷǱ�Եʱ����ʱ�Ƶ������Ƚ϶�
                         *�����ڱ�Ե��ʱ����ʱ��������������Ҫ�Ӷ����жϣ���������ʱ�ķ���ֵ��һ����˵��ʱ�ķ���ֵ��Ƚϴ󡣿����Դ��?�ж�����
                         */
                        y_bar.line_flag = 1; //��ʱ������
                    }
                    if (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL) //��Ե���ߵ�����£����⴦��?
                        y_bar.data.feedback = 0;                                                               //Ҫ���ڶ��ߵ��ж�֮����0��Ϊ�˷�ֹ��ʱ����ƫ��
                    if (y_bar.line_flag && y_bar.num <= MIN_NUM)
                    {
                        //ʹ�ô˻���Ϊ�˱�����ͣ�������϶������ߵ�����һֱ�ظ�����
                        y_bar.line_flag = 0; //�������?
                        y_bar.line_num++;    //����Ŀ��һ
                    }
                    break;
                case 2:
                    x_leftbar.data.feedback = track_value;
                    x_leftbar.num = led_num;

                    if (x_leftbar.num >= NUM_THRESHOLD || (edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL))
                    {
                        x_leftbar.line_flag = 1; //��ǵ�������?
                    }
                    if (edge_status[1] && x_leftbar.num >= EDGE_THRESHOLD && ABS(x_leftbar.data.feedback) >= EDGE_VAL)
                        x_leftbar.data.feedback = 0;
                    if (x_leftbar.line_flag && x_leftbar.num <= MIN_NUM) //������Ϊ������ͣ�������µ��ظ���������
                    {
                        x_leftbar.line_flag = 0;
                        x_leftbar.line_num++;
                    }
                    break;
                case 3:
                    x_rightbar.data.feedback = track_value;
                    x_rightbar.num = led_num;
                    if (x_rightbar.num >= NUM_THRESHOLD || (edge_status[2] && x_rightbar.num >= EDGE_THRESHOLD && ABS(x_rightbar.data.feedback) >= EDGE_VAL))
                    {
                        x_rightbar.line_flag = 1;
                    }
                    if (edge_status[2] && x_rightbar.num >= EDGE_THRESHOLD && ABS(x_rightbar.data.feedback) >= EDGE_VAL)
                        x_leftbar.data.feedback = 0;
                    if (x_rightbar.line_flag && x_rightbar.num <= MIN_NUM)
                    {
                        x_rightbar.line_flag = 0;
                        x_rightbar.line_num++;
                    }
                    break;
                default: //ɶҲ����
                    ;
                }
            }
        }
        memset(Track_Row.rec_data[AVaiable_Row], 0, sizeof(Track_Row.rec_data[AVaiable_Row])); //�����λ�õ���������?
    }
}
#endif
/**********************************************************************
 * @Name    Track_RX_IRQ
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Track_RX_IRQ(void)
{

    if (__HAL_UART_GET_IT(Track_Row.track_uart, UART_IT_RXNE)) //���յ�����
    {
        rec_data = Track_Row.track_uart->Instance->RDR; //��ȡ��������
        if (Track_Row.done_flag == 0)                   //δ�������?
        {

            if (Track_Row.start_flag == 0) //��û���յ����ֽ�
            {
                if (rec_data == START_BYTE) //�յ��ˣ���Ŀ��
                {
                    Track_Row.current_index = 0; //�����±� ׼������
                    Track_Row.start_flag = 1;    //���ñ�־λ
                    return;                      //������ж��ڴ��޹�? ֱ���˳�
                }
            }
            else //�Ѿ��յ������ֽڣ���ʱ�������Ļ��н�����ɵĴ���?
            {

                if (rec_data == END_BYTE && Track_Row.current_index >= 11) //���յ�ĩβ�ֽ�
                {
                    now_id = Get_EmptyRow_ID(now_id); //����װ�ض�����±�?
                    Track_Row.done_flag = 1;          //��־�������?
                    dma_count++;                      //��������
                    Track_Row.current_index = 0;      //�����±�
                    Track_Row.start_flag = 0;         //��־Ϊδ�յ����ֽ�
                    Track_Row.done_flag = 0;          //��־Ϊδ����
                }
                else //��������
                {
                    Track_Row.current_index += 1;
                    Track_Row.rec_data[now_id][Track_Row.current_index] = rec_data;                //װ�ؽ�����
                    if (Track_Row.current_index + 2 >= MAX_TRACK_REC_SIZE)                         //��ֹ������ɵĿ���?
                    {                                                                              //��2��Ϊ����ǰ
                        Track_Row.current_index = 0;                                               //����±�? ���¿�ʼ����
                        Track_Row.done_flag = 0;                                                   //��־λ����
                        Track_Row.start_flag = 0;                                                  //��־λ����
                        memset(Track_Row.rec_data[now_id], 0, sizeof(Track_Row.rec_data[now_id])); //������ȫ�����?
                    }
                }
            }
        }
    }
}

/**********************************************************************
 * @Name    track_pid_cal
 * @declaration : Ѱ����pid���㺯��
 * @param   bar: [����/��] ��һ��Ѱ����
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
float track_pid_cal(trackbar_t *bar)
{
    if (bar->if_switch == true) //ʹ�ܣ�����pidֵ�����з���
    {
        return pos_pid_cal(&bar->data, &track_pid_param);
    }
    return 0; //δʹ�ܣ������ı�
}

/**********************************************************************
 * @Name    track_status
 * @declaration :����Ѱ����״̬
 * @param   id: [����/��]  ����1Ϊ��ֱ ��2Ϊˮƽ
 **			 status: [����/��]  ������ر�?
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void track_status(int id, int status)
{
    if (id == 1) // y����
        y_bar.if_switch = status;
    else if (id == 2) // x����
    {
        x_leftbar.if_switch = status;
        x_rightbar.if_switch = status;
    }
}

/**********************************************************************
 * @Name    Get_Trcker_Num
 * @declaration :
 * @param   bar: [����/��]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
int Get_Trcker_Num(trackbar_t *bar)
{
    return bar->num;
}

//���񣬷�ֹѭ������
void BarTextTask(void const *argument)
{
    while (1)
    {
        if (!Track_Row.start_flag)
            osDelay(10);
        if (!Track_Row.done_flag && Track_Row.start_flag)
        {
            osDelay(5);
            count_avoid += 5;
            if (count_avoid >= MAX_time)
            {
                Track_Row.start_flag = 0;
            }
        }
    }
}

void Chassis_decode(void)
{

#define EDGE_THRESHOLD 5 //�ڱ�Ե����ģʽ�£����ŵ�����ʱΪ��Ч����
#define NUM_THRESHOLD 6  //�Ǳ�Ե�߼����£��жϵ����ߵ�����
#define MIN_NUM 2        //��ѹ��֮�󣬹����˲Ż����һ���ߣ����ݵƵ��������м���?
#define EDGE_VAL 7       //��Ե����״̬�µ�ѭ����������ֵ

    Update_TRckerCounter();                         //������صļ�����?
    static uint8_t led_num = 0;                     //����������ı���?
    static uint8_t AVaiable_Row;                    //���õ��±�
    AVaiable_Row = Get_AvaibleRow_ID(AVaiable_Row); //����λ�ø���
    float track_value = 0, temp_val;                //��صı�������?

    if (AVaiable_Row != 0XFF)
    {
        //����ĺͼ��鿴�������
        if ((uint8_t)(Track_Row.rec_data[AVaiable_Row][2] + Track_Row.rec_data[AVaiable_Row][4] + Track_Row.rec_data[AVaiable_Row][6] + Track_Row.rec_data[AVaiable_Row][8] + Track_Row.rec_data[AVaiable_Row][10] + Track_Row.rec_data[AVaiable_Row][12] + Track_Row.rec_data[AVaiable_Row][14]) == Track_Row.rec_data[AVaiable_Row][15]) //��У��
        {
            for (uint8_t bar_id = 1; bar_id <= BUFF_SIZE - 2; bar_id += 2)
            {
                track_value = 0;
                temp_val = 0;
                led_num = 0;
                switch (Track_Row.rec_data[AVaiable_Row][bar_id]) //�ж�Ѱ����ID
                {
                case 1:
                    for (uint8_t i = 0; i < 8; ++i)
                    {
                        temp_val = (bool)(((Track_Row.rec_data[AVaiable_Row][2] << i) & 0x80)) * track_weight[i]; //���ݵ���������?�صõ�����ֵ
                        if (temp_val != 0)
                            led_num++; //�������ĵ�����
                        track_value += temp_val;
                    }
                    y_bar.data.feedback = track_value; //��ֵ
                    y_bar.num = led_num;               //�õ��Ƶ�����
                    if (y_bar.num >= NUM_THRESHOLD || (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL))
                    {
                        /*�����������?
                         *һ�ǵ����ڷǱ�Եʱ����ʱ�Ƶ������Ƚ϶�
                         *�����ڱ�Ե��ʱ����ʱ��������������Ҫ�Ӷ����жϣ���������ʱ�ķ���ֵ��һ����˵��ʱ�ķ���ֵ��Ƚϴ󡣿����Դ��?�ж�����
                         */
                        y_bar.line_flag = 1; //��ʱ������
                    }
                    if (edge_status[0] && y_bar.num >= EDGE_THRESHOLD && ABS(y_bar.data.feedback) >= EDGE_VAL) //��Ե���ߵ�����£����⴦��?
                        y_bar.data.feedback = 0;                                                               //Ҫ���ڶ��ߵ��ж�֮����0��Ϊ�˷�ֹ��ʱ����ƫ��
                    if (y_bar.line_flag && y_bar.num <= MIN_NUM)
                    {
                        //ʹ�ô˻���Ϊ�˱�����ͣ�������϶������ߵ�����һֱ�ظ�����
                        y_bar.line_flag = 0; //�������?
                        y_bar.line_num++;    //����Ŀ��һ
                    }
                    // printf("%f,%d\r\n", y_bar.data.feedback, y_bar.num);
                    break;
                case 2:

                    SW_rec = Track_Row.rec_data[AVaiable_Row][4]; //���ݵ���������?�صõ�����ֵ
                    for (uint8_t i = 0; i < 8; i++)
                    {
                        if (SW_rec & (0x1 << i))
                        {
                            SW_data[i] = 1;
                        }
                    }
                    for (int i = 0; i < 8; i++)
                    {
                        go_sw(&SW, i);
                    }

                    break;
                case 3:

                    HW_rec = Track_Row.rec_data[AVaiable_Row][6];
                    for (uint8_t i = 0; i < 4; i++)
                    {
                        if (HW_rec & (0x1 << i))
                        {
                            HW_data[i] = 1;
                        }
                    }
                    for (int i = 0; i < 4; i++)
                    {
                        go_HW(&HW, i);
                    }
                    // printf("%d   ,%d    ,%d\r\n",HW.data[0],HW.data[1],HW.data[2]);
                    break;

                case 4:

                    US_data_H = Track_Row.rec_data[AVaiable_Row][8];
                    Boo.Hight_num = US_data_H;
                    break;
                case 5:
                    US_data_L = Track_Row.rec_data[AVaiable_Row][10]; //���ݵ���������?�صõ�����ֵ
                    // printf("%d\r\n",US_data_L);
                    Boo.Low_num = US_data_L;
                    turn_uint16(&Boo);       //�����������ݴ�����16��������ȡBoo.end��Ϊ���
                    count_bar(&Boo, &HW, 0); //��ľ��
                    break;
                case 6:

                    US_data_H2 = Track_Row.rec_data[AVaiable_Row][12]; //���ݵ���������?�صõ�����ֵ
                                                                       // printf("%d\r\n",US_data_H);
                    Boo2.Hight_num = US_data_H2;
                    break;
                case 7:
                    US_data_L2 = Track_Row.rec_data[AVaiable_Row][14]; //���ݵ���������?�صõ�����ֵ
                    // printf("%d\r\n",US_data_L);
                    Boo2.Low_num = US_data_L2;
                    turn_uint16(&Boo2);
                    // count_bar(&Boo2, &HW, 1); //��ľ��

                    break;

                default: //ɶҲ����
                    ;
                }
            }
        }
        memset(Track_Row.rec_data[AVaiable_Row], 0, sizeof(Track_Row.rec_data[AVaiable_Row])); //�����λ�õ���������?
        all_data_clear();
    }
}

void all_data_clear(void) //������0
{

    memset(SW_data, 0, sizeof(SW_data));
    memset(HW_data, 0, sizeof(HW_data));
    US_data = 0;
}

int Get_SW(int id) //�������������ᴥ��������
{
    if (SW.data[id] == 1)
    {
        return 1;
    }
    else if (SW.data[id] == 0)
    {
        return 0;
    }
    else
        return 0;
}
void go_sw(ALLData_t *sw, int id) //����չ���ᴥ�������ݴ����ᴥ���ؽṹ��
{
    if (SW_data[id] == 1)
    {
        sw->data[id] = 1;
    }
    else if (SW_data[id] == 0)
    {
        sw->data[id] = 0;
    }
}

int Get_US_NUM(void) //����ľ�����
{
    return Boo.count;
}
uint16_t Get_Uint8_Transform(uint8_t High, uint8_t Low) //�˽���תʮ������
{
    uint16_t data;
    data = High << 8;
    data |= Low;
    return data;
}

int Get_Bar_light(void)
{
    return y_bar.num;
}
int Get_Bar_num(void)
{
    return y_bar.line_num;
}

int Get_HW(int id) //ȡ�������ݣ�0������ľ�壬1���Ƕ�׼�ֿ��
{
    if (HW.data[id] == 1)
    {
        return 1;
    }
    else if (HW.data[id] == 0)
    {
        return 0;
    }
    else
        return 0;
}
void go_HW(ALLData_t *hw, int id) //����չ�����ݴ�������ṹ����������
{
    if (HW_data[id] == 1)
    {
        hw->data[id] = 1;
        hw->flag[id] = 1;
    }
    else if (HW_data[id] == 0)
    {
        hw->data[id] = 0;
        hw->flag[id] = 0;
    }
}
osThreadId BeeTaskHandle = NULL;            //������
void BeeTaskTaskFunc(void const *argument); //����ʵ�ֺ���
bool bee_wenwenwen;

void Feng_On(int sw) //ʹ�÷������ĺ�����һ�����þ��ܴ���
{
    bee_wenwenwen = sw;

    osThreadDef(BeeTask, BeeTaskTaskFunc, osPriorityHigh, 0, 1024); //��������ṹ��?
    BeeTaskHandle = osThreadCreate(osThread(BeeTask), NULL);        //��������
}
void BeeTaskTaskFunc(void const *argument) //����������
{

    while (bee_wenwenwen)
    {
        FengFeng();
        bee_wenwenwen = 0;
        goto EXIT_TASK;
    }
EXIT_TASK:
    BeeTaskHandle = NULL;
    vTaskDelete(NULL);
}

void FengFeng(void) //������
{
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    osDelay(2000);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_SET);
}

void count_bar(ALLData_t *boo, ALLData_t *hw, int id) //������ľ�����
{

    if (!hw_update_countLimit)
    {
        if (hw->flag[id] == 1)
        {

            boo->count++;
            hw_update_countLimit = true;
        }
    }
    else if (hw->flag[id] == 0)
    {
        hw_update_countLimit = false;
    }

#if use_old
    if (boo->flag[id] == 0)
    {

        if ((boo->end) < 0x2BC && (boo->end) > 0x00 && hw->flag[id] == 1)
        {
            printf("0xAD%d\r\n", boo->end);
            boo->flag[id] = 1;
        }
        else
            boo->flag[id] = 0;
    }
    else
    {
        if (hw->flag[id] == 0)
        {
            boo->count++;
            printf("0xLL%d\r\n", boo->count);
            boo->flag[id] = 0;
        }
        else
            boo->flag[id] = 1;
    }
#endif
}
int Get_HwBarCount(int id)
{
    return  Boo.count;
}
void Clear_HWCount(void)
{
    Boo.count = 0;
    Boo2.count = 0;
}
