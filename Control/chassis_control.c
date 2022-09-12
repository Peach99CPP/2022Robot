#include "chassis.h"
#include "cmsis_os.h"
#include "chassis_control.h"
#include "time_cnt.h"
#include "track_bar_receive.h"
#include "imu_pid.h"
#include "atk_imu.h"
#include "motor.h"
#include "imu_pid.h"
#include "Wait_BackInf.h"
#include "uart_handle.h"
#include "stdbool.h"
uint32_t time;

#define LINE_FACTOR 50

#define MAX_SPEED 250
#define MIN_SPEED 50
#define LINE_ERROR_ENCODER 150

#define Min_boo1 0xC8
#define Max_boo1 0x1F4
#define Min_speed1 150

double encoder_factor = 10;
double en_dir, en_val;
int dir, lines, boo_num, boo_dir;
int count_line_status = 1, encodermove_status = 1, booo_status = 1;
int edge_status[3] = {0};
double bias = 0, variation;
bool home_back = false;

int far_flag = 0;
// short error;将局部变量改成了全局变量
//  返回false 说明不是回家状态
bool Get_IfBackHome(void)
{
    return home_back;
}
//  1 说明回家状态
void Set_HomeBack(short if_home)
{
    if (if_home == 1)
    {
        home_back = true;
    }
    else
        home_back = false;
}
osThreadId Line_Handle = NULL;
void LineTask(void const *argument);

osThreadId Boo_Handle = NULL;
void BooTask(void const *argument);

osThreadId Encoder_Handle = NULL;
void EncoderTask(void const *argument);

void move_slantly(int dir, int speed, uint16_t delay)
{
    set_imu_status(true);
    track_status(1, 0);
    track_status(2, 0);
    int x_factor, y_factor;
    switch (dir)
    {
    case 1:
        x_factor = 1, y_factor = 1;
        break;
    case 2:
        x_factor = -1, y_factor = 1;
        break;
    case 3:
        x_factor = -1, y_factor = -1;
        break;
    case 4:
        x_factor = 1, y_factor = -1;
        break;
    default:
        x_factor = 0, y_factor = 0;
    }
    set_speed(x_factor * speed, y_factor * speed, 0);
    osDelay(delay);
    // todo 下面的更改需要进行进一步的测试以确保性能
    set_speed(0, 0, 0);   //停车
    set_imu_status(true); //关闭陀螺仪
    track_status(1, 1);   //开启循迹板来矫正姿态
    track_status(2, 1);   //两个向都开启
    osDelay(800);         //给一定的时间
    Set_InitYaw(0);       //设置角度
}

/**
 * @description:  开启一个任务用于数线
 * @param {int} direct 方向：竖向为2 横向为1
 * @param {int} line_num 数几条线
 * @param {int} edge_if 是否是在边缘（对阈值情况进行特殊处理）
 * @param {int} imu_if  是否开启陀螺仪
 * @return {*} 无需返回值
 */
void direct_move(int direct, int line_num, int imu_if, int edge_if)
{

    static int delay_time;
    if (count_line_status)
    {
    START_LINE:
        set_imu_status(imu_if);
        Clear_Line(&y_bar);
        if (edge_if)
            edge_status[0] = 1;
        else
            edge_status[0] = 0;

        count_line_status = 0;

        dir = direct;
        lines = line_num;
        osThreadDef(line_task, LineTask, osPriorityRealtime, 0, 256);
        Line_Handle = osThreadCreate(osThread(line_task), NULL);
    }
    else
    {
        //当上一次数线任务进行中时，此时不会再度调起数线任务，而是进行等待。
        //当等待时间过长时，直接取消本次数线运动。
        delay_time = 0;
        while (!count_line_status)
        {
            delay_time++;
            osDelay(100);
            if (delay_time >= 200)
            {
                printf("\n数线任务等待时间过长, 已经退出\n");
                return;
            }
        }
        goto START_LINE;
    }
}
void LineTask(void const *argument)
{
    int if_need_zero = 0; //

    osDelay(10); //切换任务 输出信息
    while (1)
    {
        static int speed_set = 0;
        short error;
        y_bar.if_switch = true;
        // if_need_zero = 1;
        // Turn_angle(1, 180, 0);
        // lines *= -1;在y轴负半轴的时候用
        if (lines > 0)
        {
            do
            {
                error = lines - y_bar.line_num;
                if (error == 0)
                {
                    printf("\n\t 数线完成\n");
                    set_speed(0, MIN_SPEED, 0);
                    Comfirm_Online(3);
                    goto EXIT_TASK;
                }
                set_speed(0, -speed_set, 0);
                osDelay(5);
            } while (error >= 0);
        }
        if (lines < 0)
        {
            if_need_zero = 1;
            Turn_angle(1, 180, 0);
            lines *= -1;
            do
            {
                error = lines - y_bar.line_num;
                if (error == 0)
                {
                    printf("\n\t 数线完成\n");
                    set_speed(0, MIN_SPEED, 0);
                    Comfirm_Online(3);
                    goto EXIT_TASK;
                }
                set_speed(0, speed_set, 0);
                osDelay(5);
            } while (error >= 0);
        }
    }
EXIT_TASK:

    set_speed(0, 0, 0);
    if (if_need_zero)
        Turn_angle(1, 180, 1);
    else
    {
        y_bar.if_switch = true;
        osDelay(1500);
        y_bar.if_switch = false;
    }
    count_line_status = 1;
    vTaskDelete(NULL);
    Line_Handle = NULL;

    // printf("%d",go_exit);
}
/**
 * @name: move_by_encoder
 * @brief: 用于开启编码器运行任务的函数
 * @param {int} direct 方向 竖直为2  水平为1
 * @param {int} val 参照平面坐标系的XY轴正负，要走的路程
 * @return {*} 无
 */
void move_by_encoder(int direct, int val)
{
    static int encoder_delay;
    val = (double)(val / 10.0);
    if (encodermove_status)
    {
    START_ENCODER:
        set_imu_status(true);

        en_dir = direct;
        en_val = val;

        encoder_sum = 0;

        osThreadDef(encodermove, EncoderTask, osPriorityRealtime, 0, 256);
        Encoder_Handle = osThreadCreate(osThread(encodermove), NULL);

        encodermove_status = 0;
    }
    else
    {
        encoder_delay = 0;
        while (!encodermove_status)
        {
            encoder_delay++;
            osDelay(100);
            if (encoder_delay >= 20)
            {
                set_speed(0, 0, 0);
                return;
            }
        }
        goto START_ENCODER;
    }
}
void get_encoder_factor(double factor)
{
    encoder_factor = factor;
}
void EncoderTask(void const *argument)
{
#define ENOCDER_DIVIDE_FACTOR 1.0 / 50.0
#define ENCODE_THRESHOLD 0.5
    static int pn;
    clear_motor_data();
    time = TIME_ISR_CNT;
    if (en_val < 0)
    {
        pn = -1;
        en_val *= -1;
    }
    else
    {
        en_val += 1;
        pn = 1;
    }

    if (en_dir == 1)
    {
        if (!Get_IfBackHome())
        {
            y_bar.if_switch = false;
            x_leftbar.if_switch = true;
            x_rightbar.if_switch = true;
        }
        else
        {
            y_bar.if_switch = false;
            x_leftbar.if_switch = false;
            x_rightbar.if_switch = false;
        }
        while (1)
        {
            if ((TIME_ISR_CNT - time > 10) && ((en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD))
                goto Encoder_Exit;
            bias = fabs(en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR));
            variation = bias * encoder_factor;
            variation = Limit_Speed(variation)*1.5;
            set_speed(variation * pn, 0, 0);
            osDelay(5);
        }
    }
    else if (en_dir == 2)
    {

        if (!Get_IfBackHome())
        {
            y_bar.if_switch = true;
            x_leftbar.if_switch = false;
            x_rightbar.if_switch = false;
        }
        else
        {
            y_bar.if_switch = false;
            x_leftbar.if_switch = false;
            x_rightbar.if_switch = false;
        }

        while (1)
        {
            if (TIME_ISR_CNT - time > 300)
                goto Encoder_Exit;
            if ((TIME_ISR_CNT - time > 10) && ((en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR)) < ENCODE_THRESHOLD))
                goto Encoder_Exit;
            bias = fabs(en_val - (encoder_sum * ENOCDER_DIVIDE_FACTOR));
            variation = bias * encoder_factor;
            variation = Limit_Speed(variation);
            set_speed(0, variation * pn, 0);
            osDelay(5);
            // TODO 考虑是否在此处关闭陀螺仪
        }
    }
Encoder_Exit:
    set_speed(0, 0, 0);
    track_status(1, 0);
    track_status(2, 0);
    osDelay(800);
    encodermove_status = 1;
    vTaskDelete(NULL);
    Encoder_Handle = NULL;
}

void car_shaking(int direct)
{
    while (y_bar.num == 0)
    {
        w_speed_set(40);
        osDelay(100);
        if (y_bar.num != 0)
            break;
        w_speed_set(-40);
        osDelay(100);
    }
}

int get_count_line_status(void)
{
    return count_line_status;
}

int get_enocdermove_status(void)
{
    return encodermove_status;
}
int get_Boomove_status(void)
{
    return booo_status;
}
int Limit_Speed(int speed)
{
    if (speed > 0)
    {
        if (speed > MAX_SPEED)
            speed = MAX_SPEED;
        if (speed < MIN_SPEED)
            speed = MIN_SPEED;
    }
    else
    {
        if (speed < -MAX_SPEED)
            speed = -MAX_SPEED;

        if (speed > -MIN_SPEED)
            speed = -MIN_SPEED;
    }
    return speed;
}
void Comfirm_Online(int dir)
{
#define LOW_SPEED_TO_CONFIRM 120
#define Track_Time 10
    Set_IMUStatus(1);
    track_status(1, 1);
    track_status(2, 1);
    if (dir == 1)
    {
        if (Get_Trcker_Num(&y_bar) <= 2)
        {
            set_speed(-LOW_SPEED_TO_CONFIRM, 0, 0);
            while (Get_Trcker_Num(&y_bar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            osDelay(Track_Time);
        }
    }
    else if (dir == 2)
    {
        if (Get_Trcker_Num(&y_bar) <= 5)
        {
            set_speed(0, -LOW_SPEED_TO_CONFIRM, 0);
            while (Get_Trcker_Num(&y_bar) <= 5)
                osDelay(10);
            osDelay(1000);
            set_speed(0, 0, 0);
            osDelay(Track_Time);
        }
    }
    else if (dir == 3)
    {
        if (Get_Trcker_Num(&x_leftbar) <= 2 && Get_Trcker_Num(&x_rightbar) <= 2)
        {
            set_speed(0, LOW_SPEED_TO_CONFIRM, 0);
            while (Get_Trcker_Num(&x_leftbar) <= 2 && Get_Trcker_Num(&x_rightbar) <= 2)
                osDelay(10);
            set_speed(0, 0, 0);
            osDelay(Track_Time);
        }
    }
    track_status(1, 0);
}
/**
 * @name: Wait_OKInf
 * @brief: 用于获取上一个任务是否运行结束
 * @param {int} type 判断的类型
 * @param {long} wait_time 最高的等到时间
 * @return {*}
 */
long Wait_OKInf(int type, long wait_time)
{
    long temp_time = 100;
    if (type == 1)
    {
        osDelay(temp_time);
        while (!get_count_line_status() && temp_time < wait_time)
        {
            temp_time += 5;
            osDelay(5);
        }
        if (get_count_line_status())
        {
            goto EXIT_WAIT;
        }
    }
    else if (type == 2)
    {
        osDelay(temp_time);
        while (!get_enocdermove_status() && temp_time < wait_time)
        {
            temp_time += 5;
            osDelay(5);
        }
        if (get_enocdermove_status())
        {
            goto EXIT_WAIT;
        }
    }
    else if (type == 3)
    {
        osDelay(temp_time);
        while (!(get_Boomove_status()) && temp_time < wait_time)
        {
            temp_time += 5;
            osDelay(5);
        }
        if (get_Boomove_status())
        {
            goto EXIT_WAIT;
        }
    }
    osDelay(100);
EXIT_WAIT:
    return temp_time;
}

/**
 * @name:  inte_move
 * @brief: 运动的集成
 * @param {int} type  类型 1 为数线 2为输编码器
 * @param {int} dir   方向 1为横向 2为竖向
 * @param {int} val    数  需要的数值 分别对应数线的数值和编码器数值
 * @param {int} edge    在数线模式下使用 用于区分是否为边缘状态
 * @param {int} imu_if  过程中是否开启陀螺仪
 * @param {long} wait_time  最长的等待时间
 * @return {*}              返回是否运行成功  当返回false代表有错误产生
 */
bool inte_move(int type, int dir, int val, int edge, int imu_if, long wait_time)
{
    if (type == 1)
    {
        direct_move(dir, val, imu_if, edge);
        Wait_OKInf(type, wait_time);
        return true;
    }
    else if (type == 2)
    {
        move_by_encoder(dir, val);
        Wait_OKInf(type, wait_time);
        return true;
    }

    printf("传入的参数类型出错 \n");
    return false;
}

void BOO_Begin(int direct, int num)
{
    static int delay_time;
    if (booo_status)
    {
    START_BOO:
        booo_status = 0;
        boo_dir = direct;
        boo_num = num;

        osThreadDef(Boo_task, BooTask, osPriorityRealtime, 0, 256);
        Boo_Handle = osThreadCreate(osThread(Boo_task), NULL);
    }
    else
    {
        delay_time = 0;
        while (!booo_status)
        {
            delay_time++;
            osDelay(100);
            if (delay_time >= 200)
            {
                printf("\n超声波等待时间过长, 已经退出\n");
                return;
            }
        }
        goto START_BOO;
    }
}

void BooTask(void const *argument)
{
    osDelay(10);
    int far_num;
    while (1)
    {
        short error;

        // far_bong=Get_US();
        // printf("%d\r\n",far_bong);
        int speed = 0;
        if (boo_dir == 1)
        {
            do
            {

                far_num = Get_US_NUM();
                // printf("0xFF%d\r\n",far_num);
                error = boo_num - far_num;

                speed = error * Min_speed1;

                if (error == 0)
                {

                    printf("\n\t 超声波over over over!");

                    goto BOO_TASK;
                }
                set_speed(0, speed, 0);
                osDelay(5);

            } while (error >= 0);
        }
        if (boo_dir == -1)
        {
            do
            {

                far_num = Get_US_NUM();
                printf("%d\r\n", far_num);
                error = boo_num - far_num;

                speed = Min_speed1 * error;

                if (error == 0)
                {
                    printf("\n\t 超声波over over over!");

                    goto BOO_TASK;
                }
                set_speed(0, -speed, 0);
                osDelay(5);

            } while (error >= 0);
        }
        if (boo_dir == 2)
        {

            do
            {
                far_num = Get_US_NUM();
                printf("%d\r\n", far_num);
                error = boo_num - far_num;
                speed = Min_speed1 * error;
                if (error == 0)
                {
                    printf("\n\t 超声波over over over!");

                    goto BOO_TASK;
                }
                set_speed(speed, 0, 0);
                osDelay(5);

            } while (error >= 0);
        }
    }
BOO_TASK:
    set_speed(0, 0, 0);
    Clear_Boo(&Boo);
    booo_status = 1;
    vTaskDelete(NULL); //用于取消任务，在结束的时候
    Boo_Handle = NULL;
}
