#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "main.h"
#include "chassis.h"
#include "chassis_control.h"
#include "motor.h"
#include "imu_pid.h"
#include "read_status.h "
#include "avoid_obs.h"
#include "atk_imu.h"
#include "track_bar_receive.h"
#include "servo.h"
#include "openmv.h"
#include "general.h"
#include "general_interface.h"
#define DUBUG_MOTOR 0
int if_OsRunning(void);
void Set_OSRunningFlag(int status);

// todo  出发前做好检查
#define Blue_Route 1
#define Red_Route 0
#define Wait_Dealy_MAX 30000
#define Line_Type 1
#define Encoder_Type 2
#define BOO_Type 3
#define some 1

// todo 在这里定义红蓝半场
int Os_RunningFlag = 0;
uint8_t cmd[3] = {0xff, 0x00, 0x99};
bool if_debug_finished = false;

#define IF_Run 1
void Startdebug(void const *argument)
{
    Set_OSRunningFlag(true);

    //   motor_debug();
    //红场是4号跟6号
    while (!(Get_SW(3) && Get_SW(5)))
        osDelay(10);
    printf("比赛开始\n");
#ifdef Run_Red
    RedGame2Test(5);
#endif
#ifdef Run_Blue
    Run4WholeGame(5);
#endif
    osDelay(100);
    printf("结束\n");
    vTaskDelete(NULL);
}

void Global_Debug(void)
{
    printf("finish \n");
}
void Go_Home(int color)
{
    if (color != 1 && color != 2)
        return;
    move_by_encoder(2, -80);
    if (color == 1)
        Turn_angle(1, 90, 0);
    else
        Turn_angle(1, -90, 0);
    // direct_move(2, 2, 0, 1);
    move_by_encoder(2, 20);
}
int if_OsRunning(void)
{
    return Os_RunningFlag;
}
void Set_OSRunningFlag(int status)
{
    Os_RunningFlag = status;
}

void Goto_Warehouse(void)
{
    ;
}

osThreadId GameTaskHandle = NULL;            //任务句柄
void GameTaskTaskFunc(void const *argument); //任务实现函数
bool GameTaskTask_Exit = 1;                  //是否退出

void Game_On(void)
{
    if (GameTaskTask_Exit && GameTaskHandle == NULL)
    {
        GameTaskTask_Exit = 0;
        osThreadDef(GameTask, GameTaskTaskFunc, osPriorityHigh, 0, 1024); //定义任务结构体
        GameTaskHandle = osThreadCreate(osThread(GameTask), NULL);        //创建任务
    }
}
void GameTaskTaskFunc(void const *argument)
{
    while (!GameTaskTask_Exit)
    {
        Global_Debug();
    }
    GameTaskHandle = NULL;
    vTaskDelete(NULL);
}
