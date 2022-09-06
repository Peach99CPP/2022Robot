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
#define MAX_VAL 7000
#define Wait_Dealy_MAX 30000
#define Line_Type 1
#define Encoder_Type 2
#define BOO_Type 3
#define Wait_Dealy_MAX2 3000
#define Wait_Delay_Max1 20000
//#define DEBUG_MODE
int debug_motor_id = 0, switch_status = 0;
int debug_speed = 0;
int begin_status = 1;
int id;
pid_data_t motor_data[5];
pid_paramer_t motor_param;
pid_paramer_t motor_param1;
//�����޷��ſ��������ó������ж���
float param_[5] = {4000,
                   9900,
                   140,
                   10,
                   0};    //这里调的是左边轮子的PID
float param1_[5] = {4000, //���3��PIDֵ����
                    9900,
                    127,
                    4,
                    0}; //这里调的是右边轮子的PID
motor_t motor1, motor2, motor3, motor4;

/**********************************************************************
 * @Name    motor_init
 * @����˵�� init for motor_t
 * @param   None
 * @����ֵ  void
 * @author  peach99CPP
 ***********************************************************************/

void motor_init(void)
{

    /*****************���1*****************/
    motor1.Encoder_IO.Port = MOTOR1_ENCODER_GPIO_Port; //���ñ�����GPIO_PORT
    motor1.Encoder_IO.Pin = MOTOR1_ENCODER_Pin;        //����PIN

    motor1.IC.Tim = &htim5;                              //ȷ���ñ������Ķ�ʱ��
    motor1.IC.Active_Channel = HAL_TIM_ACTIVE_CHANNEL_1; //������ͨ����ͬ����˼��ֻ����Ϊ�˱��ʱ���������������ò�ͬ����
    motor1.IC.Channel = TIM_CHANNEL_1;                   //

    motor1.PWM.Tim = &htim1;              // PWM������TIM
    motor1.PWM.Channel_A = TIM_CHANNEL_2; //����PWMͨ��
    motor1.PWM.Channel_B = TIM_CHANNEL_1; //

    HAL_TIM_PWM_Start(motor1.PWM.Tim, motor1.PWM.Channel_A); // PWMʹ��
    HAL_TIM_PWM_Start(motor1.PWM.Tim, motor1.PWM.Channel_B); //

    HAL_TIM_IC_Start_IT(motor1.IC.Tim, motor1.IC.Channel); // IC����ʹ��
    __HAL_TIM_ENABLE_IT(motor1.IC.Tim, TIM_IT_UPDATE);
    set_motor(1, 0);

    /*****************���2*****************/
    motor2.Encoder_IO.Port = MOTOR2_ENCODER_GPIO_Port; //���ñ�����GPIO_PORT
    motor2.Encoder_IO.Pin = MOTOR2_ENCODER_Pin;        //����PIN

    motor2.IC.Tim = &htim5;                              //ȷ���ñ������Ķ�ʱ��
    motor2.IC.Active_Channel = HAL_TIM_ACTIVE_CHANNEL_3; //������ͨ����ͬ����˼��ֻ����Ϊ�˱��ʱ���������������ò�ͬ����
    motor2.IC.Channel = TIM_CHANNEL_3;                   //

    motor2.PWM.Tim = &htim1;              // PWM������TIM
    motor2.PWM.Channel_A = TIM_CHANNEL_4; //����PWMͨ��
    motor2.PWM.Channel_B = TIM_CHANNEL_3; //

    HAL_TIM_PWM_Start(motor2.PWM.Tim, motor2.PWM.Channel_A); // PWMʹ��
    HAL_TIM_PWM_Start(motor2.PWM.Tim, motor2.PWM.Channel_B); //

    HAL_TIM_IC_Start_IT(motor2.IC.Tim, motor2.IC.Channel); // IC����ʹ��
    __HAL_TIM_ENABLE_IT(motor2.IC.Tim, TIM_IT_UPDATE);
    set_motor(2, 0);

    /*****************���3*****************/
    motor3.Encoder_IO.Port = MOTOR3_ENCODER_GPIO_Port; //���ñ�����GPIO_PORT
    motor3.Encoder_IO.Pin = MOTOR3_ENCODER_Pin;        //����PIN

    motor3.IC.Tim = &htim3;                              //ȷ���ñ������Ķ�ʱ��
    motor3.IC.Active_Channel = HAL_TIM_ACTIVE_CHANNEL_1; //������ͨ����ͬ����˼��ֻ����Ϊ�˱��ʱ���������������ò�ͬ����
    motor3.IC.Channel = TIM_CHANNEL_1;                   //

    motor3.PWM.Tim = &htim2;              // PWM������TIM
    motor3.PWM.Channel_A = TIM_CHANNEL_3; //����PWMͨ��
    motor3.PWM.Channel_B = TIM_CHANNEL_4; //

    HAL_TIM_PWM_Start(motor3.PWM.Tim, motor3.PWM.Channel_A); // PWMʹ��
    HAL_TIM_PWM_Start(motor3.PWM.Tim, motor3.PWM.Channel_B); //

    HAL_TIM_IC_Start_IT(motor3.IC.Tim, motor3.IC.Channel); // IC����ʹ��
    __HAL_TIM_ENABLE_IT(motor3.IC.Tim, TIM_IT_UPDATE);
    set_motor(3, 0);

    /*****************���4*****************/
    motor4.Encoder_IO.Port = MOTOR4_ENCODER_GPIO_Port; //���ñ�����GPIO_PORT
    motor4.Encoder_IO.Pin = MOTOR4_ENCODER_Pin;        //����PIN

    motor4.IC.Tim = &htim3;                              //ȷ���ñ������Ķ�ʱ��
    motor4.IC.Active_Channel = HAL_TIM_ACTIVE_CHANNEL_3; //������ͨ����ͬ����˼��ֻ����Ϊ�˱��ʱ���������������ò�ͬ����
    motor4.IC.Channel = TIM_CHANNEL_3;                   //

    motor4.PWM.Tim = &htim2;                                 // PWM������TIM
    motor4.PWM.Channel_A = TIM_CHANNEL_2;                    //����PWMͨ��
    motor4.PWM.Channel_B = TIM_CHANNEL_1;                    //
    HAL_TIM_PWM_Start(motor4.PWM.Tim, motor4.PWM.Channel_A); // PWMʹ��
    HAL_TIM_PWM_Start(motor4.PWM.Tim, motor4.PWM.Channel_B); //

    HAL_TIM_IC_Start_IT(motor4.IC.Tim, motor4.IC.Channel); // IC����ʹ��
    __HAL_TIM_ENABLE_IT(motor4.IC.Tim, TIM_IT_UPDATE);
    set_motor(4, 0);
}

/**********************************************************************
 * @Name    Motor_PID_Init
 * @����˵�� Init for Motor PID param
 * @param   : [����/��] void
 * @����ֵ  void
 * @author  peach99CPP
 ***********************************************************************/

void Motor_PID_Init(void)
{
    motor_param.integrate_max = param_[0];
    motor_param.control_output_limit = param_[1];
    motor_param.kp = param_[2];
    motor_param.ki = param_[3];
    motor_param.kd = param_[4];
}
void Motor_PID1_Init(void)
{
    motor_param1.integrate_max = param1_[0];
    motor_param1.control_output_limit = param1_[1];
    motor_param1.kp = param1_[2];
    motor_param1.ki = param1_[3];
    motor_param1.kd = param1_[4];
}
/************************************************************
 *@name:read_encoder
 *@function:�õ���������ֵ
 *@param:��Ҫ��ȡ�ĵ�����
 *@return:��������ֵ
 **************************************************************/
float read_encoder(int motor_id)
{
    double temp_num = encoder_val[motor_id];
//	if(motor_id == 2)
//	{
//		temp_num*=-1;
//	}
//    encoder_val[motor_id] = 0;
#ifdef DEBUG_MODE
    printf("\r\nmotor%d sppeed =%d\r\n", motor_id, (int)temp_num);
#endif
    //�ڵ����з�������������� �ڶϵ��ת�����ݲ���õ�����
    return (int)temp_num;
}

/************************************************************
 *@name:set_motor
 *@function�����Ƶ���Ķ�ʱ���ڣ��������ת��
 *@motor_id:���Ƶ�Ŀ�������
 *@control_val������Ŀ���ֵֵ
 *@return: ��
 **************************************************************/
void set_motor(int motor_id, int control_val)
{
    volatile uint32_t *ChannelA_ptr;
    volatile uint32_t *ChannelB_ptr;
    int pos_flag; //��־λ����
    //�޷�
    if (control_val > MAX_VAL)
    {
        control_val = MAX_VAL;
    }
    else if (control_val < -(MAX_VAL))
    {
        control_val = -MAX_VAL;
    }
    //�����ж�
    if (control_val > 0)
    {
        pos_flag = 1;
    }
    else if (control_val < 0 || control_val == 0)
    {
        pos_flag = 0;
        control_val *= (-1); //�ȼ��ڽ���ABS����
    }

    ChannelA_ptr = get_motor_channelA_ptr(motor_id);
    ChannelB_ptr = get_motor_channelB_ptr(motor_id);
    if (ChannelA_ptr != NULL && ChannelB_ptr != NULL) //����ָ��������
    {
        if (pos_flag)
        {
            *ChannelA_ptr = control_val;
            *ChannelB_ptr = 0;
        }
        else
        {
            *ChannelB_ptr = control_val;
            *ChannelA_ptr = 0;
        }
    }
}

/**********************************************************************
 * @Name    get_motor_channelA_ptr
 * @declaration :get the ptr of register of channelA
 * @param   motor_id: [����/��]
 * @retval   : ptr
 * @author  peach99CPP
 ***********************************************************************/

volatile uint32_t *get_motor_channelA_ptr(int motor_id)
{
    volatile uint32_t *ptr;
    switch (motor_id)
    {
    case 1:

        if (motor1.PWM.Channel_A == TIM_CHANNEL_1)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR1);
        }
        else if (motor1.PWM.Channel_A == TIM_CHANNEL_2)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR2);
        }
        else if (motor1.PWM.Channel_A == TIM_CHANNEL_3)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR3);
        }
        else if (motor1.PWM.Channel_A == TIM_CHANNEL_4)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR4);
        }
        break;
    case 2:
        if (motor2.PWM.Channel_A == TIM_CHANNEL_1)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR1);
        }
        else if (motor2.PWM.Channel_A == TIM_CHANNEL_2)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR2);
        }
        else if (motor2.PWM.Channel_A == TIM_CHANNEL_3)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR3);
        }
        else if (motor2.PWM.Channel_A == TIM_CHANNEL_4)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR4);
        }
        break;
    case 3:
        if (motor3.PWM.Channel_A == TIM_CHANNEL_1)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR1);
        }
        else if (motor3.PWM.Channel_A == TIM_CHANNEL_2)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR2);
        }
        else if (motor3.PWM.Channel_A == TIM_CHANNEL_3)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR3);
        }
        else if (motor3.PWM.Channel_A == TIM_CHANNEL_4)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR4);
        }
        break;
    case 4:
        if (motor4.PWM.Channel_A == TIM_CHANNEL_1)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR1);
        }
        else if (motor4.PWM.Channel_A == TIM_CHANNEL_2)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR2);
        }
        else if (motor4.PWM.Channel_A == TIM_CHANNEL_3)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR3);
        }
        else if (motor4.PWM.Channel_A == TIM_CHANNEL_4)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR4);
        }
        break;
    default:
        ptr = NULL;
    }
    return ptr;
}
/**********************************************************************
 * @Name    get_motor_channelB_ptr
 * @declaration :get the ptr of register of channelB
 * @param   motor_id: [����/��]
 * @retval   : ptr
 * @author  peach99CPP
 ***********************************************************************/

volatile uint32_t *get_motor_channelB_ptr(int motor_id)
{
    volatile uint32_t *ptr;
    switch (motor_id)
    {
    case 1:

        if (motor1.PWM.Channel_B == TIM_CHANNEL_1)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR1);
        }
        else if (motor1.PWM.Channel_B == TIM_CHANNEL_2)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR2);
        }
        else if (motor1.PWM.Channel_B == TIM_CHANNEL_3)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR3);
        }
        else if (motor1.PWM.Channel_B == TIM_CHANNEL_4)
        {
            ptr = &(motor1.PWM.Tim->Instance->CCR4);
        }
        break;
    case 2:
        if (motor2.PWM.Channel_B == TIM_CHANNEL_1)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR1);
        }
        else if (motor2.PWM.Channel_B == TIM_CHANNEL_2)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR2);
        }
        else if (motor2.PWM.Channel_B == TIM_CHANNEL_3)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR3);
        }
        else if (motor2.PWM.Channel_B == TIM_CHANNEL_4)
        {
            ptr = &(motor2.PWM.Tim->Instance->CCR4);
        }
        break;
    case 3:
        if (motor3.PWM.Channel_B == TIM_CHANNEL_1)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR1);
        }
        else if (motor3.PWM.Channel_B == TIM_CHANNEL_2)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR2);
        }
        else if (motor3.PWM.Channel_B == TIM_CHANNEL_3)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR3);
        }
        else if (motor3.PWM.Channel_B == TIM_CHANNEL_4)
        {
            ptr = &(motor3.PWM.Tim->Instance->CCR4);
        }
        break;
    case 4:
        if (motor4.PWM.Channel_B == TIM_CHANNEL_1)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR1);
        }
        else if (motor4.PWM.Channel_B == TIM_CHANNEL_2)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR2);
        }
        else if (motor4.PWM.Channel_B == TIM_CHANNEL_3)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR3);
        }
        else if (motor4.PWM.Channel_B == TIM_CHANNEL_4)
        {
            ptr = &(motor4.PWM.Tim->Instance->CCR4);
        }
        break;
    default:
        ptr = NULL;
    }
    return ptr;
}

/**********************************************************************
 * @Name    show_speed
 * @declaration :used to show motor speed
 * @param   : [����/��] none
 * @retval   : void
 * @author  peach99CPP
 ***********************************************************************/

void show_speed(void)
{
    if (debug_motor_id == 0 || !switch_status)
        return;
    printf("%.2lf,  %d,\r\n", read_encoder(debug_motor_id), (int)motor_target[debug_motor_id]);
}

/**********************************************************************
 * @Name    clear_motor_data
 * @declaration : clear pid output for increment pid
 * @param   None
 * @retval   :void
 * @author  peach99CPP
 ***********************************************************************/

void clear_motor_data(void)
{
    //���PID����ֵ,�ָ��������ֵ
    for (uint8_t i = 1; i <= 4; ++i)
    {
        pid_clear(&motor_data[i]);
        encoder_val[i] = 0;
        set_motor(i, 0);
        status_flag[i] = 0;
    }
    //�������ò���������
    __HAL_TIM_SET_CAPTUREPOLARITY(motor1.IC.Tim, motor1.IC.Channel, TIM_ICPOLARITY_RISING);
    __HAL_TIM_SET_CAPTUREPOLARITY(motor2.IC.Tim, motor2.IC.Channel, TIM_ICPOLARITY_RISING);
    __HAL_TIM_SET_CAPTUREPOLARITY(motor3.IC.Tim, motor3.IC.Channel, TIM_ICPOLARITY_RISING);
    __HAL_TIM_SET_CAPTUREPOLARITY(motor4.IC.Tim, motor4.IC.Channel, TIM_ICPOLARITY_RISING);

    encoder_sum = 0;
}
/******
 * ���е������ʱ���õĺ�������
 * ���е������ʱ,USMART��ĺ궨��ǵ�����Ϊ1
 * ���裺 ���ú�
 *
 */

/**********************************************************************
 * @Name    set_debug_motor
 * @declaration :set the status and motor id
 * @param   status: [����/��]
 **			 motor_id: [����/��]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void set_debug_motor(int status, int motor_id)
{
    switch_status = status;
    debug_motor_id = motor_id;
}
/**********************************************************************
 * @Name    motor_debug
 * @declaration : run debug in main function
 * @param   None
 * @retval   :void
 * @author  peach99CPP
 ***********************************************************************/

void motor_debug(void)
{
    if (debug_speed != 0) //ȷ������debugģʽ�£���Ҫ�����õ�����
    {
        set_speed(0, debug_speed, 0);
        osDelay(1000);
        set_speed(0, 0, 0);
        osDelay(1000);
    }
    else
        osDelay(5);
}

/**********************************************************************
 * @Name    set_debug_speed
 * @declaration :  set debug speed
 * @param   speed: [����/��]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void set_debug_speed(int speed)
{
    chassis.enable_switch = true;
    debug_speed = speed;
}
/**********************************************************************
 * @Name    set_motor_pid
 * @����˵�� ki kd kp param changg interface
 * @param   kp: [����/��] kp param
 **			 ki: [����/��] ki param
 **			 kd: [����/��] kd param
 * @����ֵ void
 * @author  peach99CPP
 ***********************************************************************/

void set_motor_pid(int kp, int ki, int kd)
{
    clear_motor_data();
    motor_param.kp = kp;
    motor_param.ki = (int)(ki/1000.0);
    motor_param.kd = kd;
}

void set_motor_pid1(int kp, int ki, int kd)
{
    clear_motor_data();
    motor_param1.kp = kp;
    motor_param1.ki = (int)(ki/1000.0);
    motor_param1.kd = kd;
}

/**********************************************************************
 * @Name    set_motor_maxparam
 * @����˵�� change max value
 * @param   integrate_max: [����/��]  mav value of I
 **			 control_output_limit: [����/��]  general max value
 * @����ֵ  void
 * @author  peach99CPP
 ***********************************************************************/

void set_motor_maxparam(int integrate_max, int control_output_limit)
{
    clear_motor_data();
    motor_param.control_output_limit = control_output_limit;
    motor_param.integrate_max = integrate_max;
}

void set_motor_maxparam1(int integrate_max, int control_output_limit)
{
    clear_motor_data();
    motor_param1.control_output_limit = control_output_limit;
    motor_param1.integrate_max = integrate_max;
}

void go_two_bar(void) //�ߵ�Բ�̻�Ȼ��ײԲ�̻�
{
    long time_now;
    move_by_encoder(1, 200);
    time_now = Wait_OKInf(Encoder_Type, Wait_Dealy_MAX2); //������
    printf("编码器向右走完成\n");
    printf("%ld\n", time_now);
    set_speed(0, 0, 0);
    osDelay(2000);
    // track_status(1,0);
    printf("停一下子\n");
    BOO_Begin(1, 2); //��ǰ������ľ��
    time_now = Wait_OKInf(BOO_Type, Wait_Delay_Max1);
    printf("超声波数木板成功\n");
    printf("%ld\n", time_now);
    move_by_encoder(2, -95);
    time_now = Wait_OKInf(Encoder_Type, Wait_Dealy_MAX2); //������һ���
    printf("编码器让车子后退成功\n");
    printf("%ld\n", time_now);
    move_by_encoder(1, 120);
    time_now = Wait_OKInf(Encoder_Type, Wait_Dealy_MAX2); //������ײ�ᴥ����
    printf("编码器让车子往右走撞木板成功\n");
    printf("%ld\n", time_now);
    Wait_Switches(3); //ײ����
    printf("轻触开关使姿态矫正完成\n");
    set_speed(-200, 0, 0); //这里改成编码器的话陀螺仪会在下一步转错
    osDelay(2000);
    //	move_by_encoder(1,180);
    //	Wait_OKInf(Encoder_Type, Wait_Dealy_MAX2);//������
    printf("车身往左走\n");
    Turn_angle(1, 180, 0); //ת180��
    printf("转180度成功\n");
    osDelay(500);
}

void go_warehouse(void) //这里是走到仓库,用红外来定位到仓库中心，这里需要将底下的红外拖出来，因为还没有用舵控
{
    //这个地方可加舵控，把红外拉出来
    while (1) //���ú����ߵ��ֿ�����м�
    {
        int count_bar;
        bool flag_bar;
        set_speed(0, 100, 0);
        if (HW.data[1] == 1) //先红外扫到木板
        {
            flag_bar = 1;
        }
        if (flag_bar)
        {
            if (HW.data[1] == 0) //扫到木板后出来
            {
                count_bar++; //作为木板计数值
            }
        }
        osDelay(5);
    }
    set_speed(0, 0, 0); //��ʱ����ֿ���м�
}
osThreadId Go_Handle = NULL;
void GoTask(void const *argument);

void begin_all(int status) //这里创建任务本来是想要用来跑全程的，现在直接用debug_task.c
{

    static int delay_time;
    if (begin_status)
    {
    START_Go:
        begin_status = 0;
        id = status;
        osThreadDef(Go_task, GoTask, osPriorityRealtime, 0, 256);
        Go_Handle = osThreadCreate(osThread(Go_task), NULL);
    }
    else
    {
        delay_time = 0;
        while (!begin_status)
        {
            delay_time++;
            osDelay(100);
            if (delay_time >= 200)
            {
                printf("\n任务等待时间过长, 已经退出\n");
                return;
            }
        }
        goto START_Go;
    }
}
void GoTask(void const *argument)
{

    while (1)
    {
        if (id == 1)
        {
            go_two_bar();
            goto EXIT_TASK;
        }
    }
EXIT_TASK:
    set_speed(0, 0, 0);

    begin_status = 1;
    vTaskDelete(NULL);
    Go_Handle = NULL;

    // printf("%d",go_exit);
}
