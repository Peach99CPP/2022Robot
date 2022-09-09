/* ************************************************************
 *
 * FileName   : imu_pid.c
 * Version    : v1.0
 * Author     : peach99CPP
 * Date       : 2021-09-11
 * Description:  ������Ӧ��API
 ******************************************************************************
 */
//���ݻ�ȡ
#include "imu_pid.h"
//��������
#include "atk_imu.h"
//ѭ����ͷ�ļ�
#include "track_bar_receive.h "
// printdf���ڵ�ͷ�ļ�
#include "uart_handle.h"
#include "chassis.h"

/*****��Ϊ�ڱ�ĵط��Ѿ��иú궨�壬�������ظ����
 #define ABS(X)  (((X) > 0)? (X) : -(X))
******/
pid_data_t imu_data, anglekeep_data;
pid_paramer_t imu_para =
    {
        .kp = 6.0,
        .ki = 0.1,
        .kd = 0,
        .integrate_max = 70,
        .control_output_limit = 500};

extern ATK_IMU_t imu;
float delta;
int if_completed;

/**********************************************************************
 * @Name    imu_correct_val
 * @declaration : imu pidʵ�ֵĺ��ĺ���
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

float imu_correct_val(void)
{
  static float now_angle;
  // static float i;
  // static float raw_angle; //���̱�����Ϊ�����ظ�������ʹ�þ�̬����
  //�жϴ�ʱת���״̬

  if (!imu.enable_switch)
    return 0; //δʹ����ֱ�ӷ���0�������޸�
  else
  {
    // raw_angle = Get_Raw_Yaw();
    imu_data.expect = imu.target_angle; //���ú�pid��Ŀ��
    now_angle = imu.get_angle();        //��ȡ�Ƕ���ֵ
    now_angle = angle_limit(now_angle);
    // i=now_angle;

    //ȡ����·��
    if (now_angle - imu.target_angle > 180)
      now_angle -= 360;
    if (now_angle - imu.target_angle < -180)
      now_angle += 360;
    imu_data.feedback = now_angle;
    if (fabs(angle_limit(now_angle - imu.target_angle)) < 3)
      if_completed = 1;
    else
      if_completed = 0;
    //��ȡPIDֵ
    delta = pos_pid_cal(&imu_data, &imu_para); // pid����
                                               // printf("%.2f,%.2f,%.2f,%.2f\r\n",now_angle  ,imu.target_angle   ,imu_data.err,i)
    return delta;                              //���ؼ���ֵ
  }
}

/**********************************************************************
 * @Name    set_imu_angle
 * @declaration :
 * @param   angle: [����/��]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void set_imu_angle(int angle)
{
  imu.enable_switch = 1;                 //Ĭ�Ͽ�������
  imu.target_angle = angle_limit(angle); //���޷���ĽǶȣ������ṹ��
}

/**********************************************************************
 * @Name    set_imu_param
 * @declaration :
 * @param   p: [����/��]
 **			 i: [����/��]
 **			 d: [����/��]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void set_imu_param(int p, int i, int d)
{
  // USMART�������ò���
  imu_para.kp = (p / 10.0);
  imu_para.ki = (i / 100.0);
  imu_para.kd = (d / 10.0);
}
/**********************************************************************
 * @Name    set_imu_status
 * @declaration :
 * @param   status: [����/��]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void set_imu_status(int status)
{
  //�ı������ǵ�ʹ��״̬
  imu.enable_switch = status;
}

/**********************************************************************
  * @Name    Turn_angle
  * @declaration : ת��ĺ���ʵ�֣���ʵ�־��ԽǶȵ�ת�����ԽǶȵ�ת��
  * @param   mode:   ת�������
                    relative(1): ��ԽǶ�
                    absolute(2): ���ԽǶ�
**	@param	 angle:   �Ƕ���ֵ
    @param   track_enabled ת���Ƿ���Ҫ����ѭ����
  * @retval   : ��
  * @author  peach99CPP
***********************************************************************/
void Turn_angle(int mode, int angle, int track_enabled)
{

  if (imu.enable_switch)
  {
    //�޷�
    angle = angle_limit(angle);
    //��ԽǶ�ģʽ
    if (mode == 1)
    {
      imu.target_angle = angle_limit(imu.get_angle() + angle);
      // imu.target_angle = angle_limit(angle);
    }
    //���ԽǶ�ģʽ
    else if (mode == 2)
      imu.target_angle = angle;
    osDelay(20); //ȷ��������ж�����Ч�� �����ᱻ����ִ�ж�����
    while (!get_turn_status())
      osDelay(2); //ת���������˳��ú���
    //������ѭ����ر�
    set_speed(0, 0, 0);
    if (track_enabled)
    {
      x_leftbar.if_switch = true;
      x_rightbar.if_switch = true;
      y_bar.if_switch = true;
      osDelay(1500);
      x_leftbar.if_switch = false;
      x_rightbar.if_switch = false;
      y_bar.if_switch = false;
    }
    else
    {
      //����ͣ��һ��
      osDelay(1500);
    }
  }
  else
    printf("������δ����,�޷�ת��\r\n");
}
int get_turn_status(void)
{
  return if_completed;
}
