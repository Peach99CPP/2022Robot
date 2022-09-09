/* ************************************************************
 *
 * FileName   : imu_pid.c
 * Version    : v1.0
 * Author     : peach99CPP
 * Date       : 2021-09-11
 * Description:  陀螺仪应用API
 ******************************************************************************
 */
//数据获取
#include "imu_pid.h"
//数据运用
#include "atk_imu.h"
//循迹版头文件
#include "track_bar_receive.h "
// printdf所在的头文件
#include "uart_handle.h"
#include "chassis.h"

/*****因为在别的地方已经有该宏定义，故无需重复添加
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
 * @declaration : imu pid实现的核心函数
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

float imu_correct_val(void)
{
  static float now_angle;
  // static float i;
  // static float raw_angle; //过程变量，为避免重复声明，使用静态变量
  //判断此时转弯的状态

  if (!imu.enable_switch)
    return 0; //未使能则直接返回0，不做修改
  else
  {
    // raw_angle = Get_Raw_Yaw();
    imu_data.expect = imu.target_angle; //设置好pid的目标
    now_angle = imu.get_angle();        //获取角度数值
    now_angle = angle_limit(now_angle);
    // i=now_angle;

    //取最优路径
    if (now_angle - imu.target_angle > 180)
      now_angle -= 360;
    if (now_angle - imu.target_angle < -180)
      now_angle += 360;
    imu_data.feedback = now_angle;
    if (fabs(angle_limit(now_angle - imu.target_angle)) < 3)
      if_completed = 1;
    else
      if_completed = 0;
    //获取PID值
    delta = pos_pid_cal(&imu_data, &imu_para); // pid传参
                                               // printf("%.2f,%.2f,%.2f,%.2f\r\n",now_angle  ,imu.target_angle   ,imu_data.err,i)
    return delta;                              //返回计算值
  }
}

/**********************************************************************
 * @Name    set_imu_angle
 * @declaration :
 * @param   angle: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void set_imu_angle(int angle)
{
  imu.enable_switch = 1;                 //默认开启开关
  imu.target_angle = angle_limit(angle); //将限幅后的角度，传给结构体
}

/**********************************************************************
 * @Name    set_imu_param
 * @declaration :
 * @param   p: [输入/出]
 **			 i: [输入/出]
 **			 d: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void set_imu_param(int p, int i, int d)
{
  // USMART调试设置参数
  imu_para.kp = (p / 10.0);
  imu_para.ki = (i / 100.0);
  imu_para.kd = (d / 10.0);
}
/**********************************************************************
 * @Name    set_imu_status
 * @declaration :
 * @param   status: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/

void set_imu_status(int status)
{
  //改变陀螺仪的使能状态
  imu.enable_switch = status;
}

/**********************************************************************
  * @Name    Turn_angle
  * @declaration : 转弯的函数实现，可实现绝对角度的转弯和相对角度的转弯
  * @param   mode:   转弯的类型
                    relative(1): 相对角度
                    absolute(2): 绝对角度
**	@param	 angle:   角度数值
    @param   track_enabled 转后是否需要开启循迹版
  * @retval   : 无
  * @author  peach99CPP
***********************************************************************/
void Turn_angle(int mode, int angle, int track_enabled)
{

  if (imu.enable_switch)
  {
    //限幅
    angle = angle_limit(angle);
    //相对角度模式
    if (mode == 1)
    {
      imu.target_angle = angle_limit(imu.get_angle() + angle);
      // imu.target_angle = angle_limit(angle);
    }
    //绝对角度模式
    else if (mode == 2)
      imu.target_angle = angle;
    osDelay(20); //确保下面的判断是有效的 而不会被立马执行而跳过
    while (!get_turn_status())
      osDelay(2); //转动结束再退出该函数
    //开两秒循迹后关闭
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
      //单纯停车一会
      osDelay(1500);
    }
  }
  else
    printf("陀螺仪未开启,无法转弯\r\n");
}
int get_turn_status(void)
{
  return if_completed;
}
