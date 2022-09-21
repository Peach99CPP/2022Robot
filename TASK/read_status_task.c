/* ************************************************************
 *
 * FileName   : read_status.c
 * Version    : v1.0
 * Author     : peach99CPP
 * Date       : 2021-09-12
 * Description:
 ******************************************************************************
 */
#include "read_status.h "

#include "chassis.h"
#include "atk_imu.h"
#include "openmv.h"
#include "servo.h"
#include "general.h"
#include "track_bar_receive.h"
#include "Wait_BackInf.h"
#include "imu_pid.h"
#define Height_HW1 5
#define RED_TARGET 1
#define BLUE_TARGET 0
// 17 18 19最低 最高 中间
osThreadId Read_Swicth_tasHandle;             //任务句柄
void Read_Swicth(void const *argument);       //函数声明
osThreadId Height_UpadteTask;                 //任务句柄
void HeightUpdate_Task(void const *argument); //函数声明

ScanDir_t Height_Mode = Primary_Head;
Height_t Current_Height = PrimaryHeight;
Game_Color_t Current_Color = Not_Running;

int Time_constant_before = 800, Time_constant_after = 1000;

void Trans_Cons(int val1, int val2)
{
    if (val1 >= 0)
        Time_constant_before = val1;
    if (val2 >= 0)
        Time_constant_after = val2;
    printf("\n\t当前方案为延时 %dms后进行高度变换\n\t变换后延时%dms后继续前进\t\n", Time_constant_before, Time_constant_after);
}

int Height_id = 1;
int read_task_exit = 1, Height_task_exit = 1; //任务退出标志
short Height_Flag = 0;

bool QR_Brick = true; //是否位于三种高度 模式

short swicth_status[8]; //开关状态，只在内部进行赋值
short HW_Switch[10];    //红外开关的状态
int MIN_ = 50;
int VERTICAL = 40;
int MIN_SPEED = 80; //速度继续增大 防止在侧向移动时动力不足 撞击挡板使用

bool update_finish = true;
bool HeightAvailable = true;

#define Wait_Servo_Done 20000 //等待动作组完成的最大等待时间

#define SWITCH(x) swicth_status[(x)-1] //为了直观判断开关编号
#define HW_SWITCH(X) HW_Switch[(X)-1]  // 0到3下标就是红外开关的位置

#define Height_SWITCH(x) HW_Switch[(x) + 4 - 1] //高度的开关，第4和第5下标分配给高度红外

#define Side_SWITCH(X) HW_Switch[(X) + 6 - 1] //侧边红外的安装位置,有两个，分配下标为6 和7

bool Get_HeightAvailable(void)
{
    return HeightAvailable;
}
void Set_HeightAvailable(bool Switch_Status)
{
    HeightAvailable = Switch_Status;
}

void Recover_EnableStatus(void)
{
    Set_MV_Mode(true);
    Set_QR_Status(true);
}

/**********************************************************************
 * @Name    Get_Update_Result
 * @declaration :获取高度更新的结果
 * @param   None
 * @retval   : 是否更新完成
 * @author  peach99CPP
 ***********************************************************************/
bool Get_Update_Result(void)
{
    return update_finish;
}

/**********************************************************************
 * @Name    Set_Update_Status
 * @declaration : 设置高度更新值
 * @param   status: [输入/出]  设置后的状态
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Set_Update_Status(bool status)
{
    update_finish = status;
}

/**********************************************************************
 * @Name    Wait_Update_finish
 * @declaration : 等待更新结束 用这个卡住任务 否则不会停车
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Wait_Update_finish(void)
{
    while (Get_Update_Result() != true)
    {
        set_speed(0, 0, 0);
        osDelay(5);
    }
}

/**********************************************************************
 * @Name    Set_NeedUp
 * @declaration : 设置是否为二维码模式
 * @param   if_on: [输入/出] 是或否
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Set_NeedUp(bool if_on) //在此处设置是否需要动态变换高度
{
    QR_Brick = if_on;
}
/**********************************************************************
 * @Name    Return_IFQR
 * @declaration : 返回是否处于二维码工作模式
 * @param   None
 * @retval   : true则为二维码模式
 * @author  peach99CPP
 ***********************************************************************/
bool Return_IFQR(void)
{
    return QR_Brick;
}

/**********************************************************************
 * @Name    QR_Mode_Height
 * @declaration : 在二维码模式下执行高度变换
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void QR_Mode_Height(void)
{
    if (Return_IFQR())
    {
        Inf_Servo_Height(Get_Height()); //根据高度执行动作组
    }
}
/**********************************************************************
 * @Name    Return_ReverseID
 * @declaration : 获取相对的开关编号
 * @param   id: [输入/出] 当前的开关编号
 * @retval   : 同侧的相对编号
 * @author  peach99CPP
 ***********************************************************************/
int Return_ReverseID(int id)
{
    if (id == 1)
        return 2;
    else if (id == 2)
        return 1;
    else if (id == 5)
        return 6;
    else if (id == 6)
        return 5;
    else
        return 1; //避免出错，返回0容易引起数组越界 todo最好在此分支增加一个错误报告的打印数据
}

void Inf_Servo_Height(int now_height)
{
    // if (Return_IFQR()) //在复赛决赛部分需要
    // {
    //     static int last_height = PrimaryHeight;
    //     Set_Update_Status(false);      //此时禁用高度变换那边的响应
    //     if (last_height != now_height) //避免重复提醒
    //     {
    //         last_height = now_height;
    //         printf("高度发生改变\n");
    //     }
    //     set_speed(0, 0, 0);                 //先停车 todo  此处运行之后
    //     Wait_Servo_Signal(Wait_Servo_Done); //确保上一个命令完成才可以执行后续的指令
    //     if (now_height == LowestHeight)     //根据当前高度进行角度的更新操作
    //     {
    //         // todo 当获知具体位置时决定是否开启
    //         Action_Gruop(toLowest, 1); //运行动作组
    //     }
    //     else if (now_height == MediumHeight)
    //         Action_Gruop(toMedium, 1);
    //     else if (now_height == HighestHeight)
    //         Action_Gruop(toHighest, 1);
    //     else
    //         osDelay(100);                   //初始状态
    //     Wait_Servo_Signal(Wait_Servo_Done); //等待信号
    //     if (now_height == LowestHeight)
    //     {
    //         osDelay(Time_constant_after); // todo 调试时的特殊需要
    //         // TODO  变换成功后再开启二维码的接收处理工作
    //     }
    //     Set_Update_Status(true);
    //     Set_QR_Status(true);
    // }
    // Set_HeightAvailable(true); //在下面的操作中对高度红外的读取进行屏蔽
}
/**********************************************************************
 * @Name    Judge_Side
 * @declaration :
 * @param   color_mode: [输入/出]
 **			 dir: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Judge_Side(int dir)
{
    if (dir == 5)
    {
        Current_Height = MediumHeight;
    }
    else if (dir == 6)
    {
        Current_Height = LowestHeight;
    }
}

/**********************************************************************
 * @Name    Start_HeightUpdate
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Start_HeightUpdate(void)
{
    if (Height_task_exit)
    {
        Height_task_exit = 0;
        Height_Flag = 0;
        osThreadDef(Height_UpadteTask, HeightUpdate_Task, osPriorityHigh, 0, 256);
        Height_UpadteTask = osThreadCreate(osThread(Height_UpadteTask), NULL);
    }
}

/**********************************************************************
 * @Name    HeightUpdate_Task
 * @declaration :
 * @param   argument: [输入/出]
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void HeightUpdate_Task(void const *argument)
{
    Height_Flag = 0;
    static short temp_roi_chnage_flag = 1;
    // todo 后续增加参数或者其他赋值的变量来指定使用哪个红外（使用双高度红外的情况下）
    while (!Height_task_exit)
    {
        if (Current_Height == LowestHeight)
        {
            Height_id = 2; //使用2号来判断高度模式 /
            // todo 此处记得检查是否已经设置好对应高度的切换动作组
            printf("\n************Current Height:LowestHeight***************\n");
            while (!Height_task_exit)
            {
                if (Get_Height_Switch(Height_id) == on && Height_Flag == 0 && Get_Servo_Flag() == true && Get_HeightAvailable())
                {
                    Height_Flag = 1;
                    Current_Height = HighestHeight;
                    printf("\n************Current Height:HighestHeight***************\n");
                    Inf_Servo_Height(Current_Height);
                }
                if (Height_Flag == 1 && Get_Servo_Flag() == true)
                {
                    if (Get_Height_Switch(Height_id) == off && Get_HeightAvailable())
                    {
                        Current_Height = MediumHeight;
                        Height_Flag = 2;
                        printf("\n************Current Height:MediumHeight***************\n");
                        Inf_Servo_Height(Current_Height);
                    }
                }
                osDelay(1);
            }
        }
        else if (Current_Height == MediumHeight)
        {
            Height_id = 1; // todo临时修改
            printf("\n************Current Height:MediumHeight***************\n");
            while (!Height_task_exit)
            {
                if (Get_Servo_Flag() == true && Get_Height_Switch(Height_id) == on && Height_Flag == 0 && Get_HeightAvailable())
                {
                    Height_Flag = 1;
                    Height_id = 1; // todo 此处需要检查下，因为一开始使用了临时的红外编号
                    Current_Height = HighestHeight;
                    Set_QR_Status(false); //变换高度过程中禁用二维码的接收处理
                    Inf_Servo_Height(Current_Height);
                    printf("\n************Current Height:HighestHeight***************\n");
                }
                if (Height_Flag == 1 && Get_Servo_Flag() == true)
                {
                    if (Get_Height_Switch(Height_id) == off && Get_HeightAvailable())
                    {
                        // todo 尝试方案一  延时进行高度变换
                        Set_QR_Status(false); //变换高度过程中禁用二维码的接收处理
                        osDelay(Time_constant_before);
                        Current_Height = LowestHeight;
                        Height_Flag = 2;
                        Inf_Servo_Height(Current_Height);
                        printf("\n************Current Height:LowestHeight***************\n");
                    }
                    //以下在接收端经不被处理
                    if (Get_Height_Switch(2) == on && temp_roi_chnage_flag == 1)
                    {
                        temp_roi_chnage_flag = 0;
                        OpenMV_ChangeRoi(2);
                    }
                }

                osDelay(1);
            }
        }
        osDelay(5);
    }
    Height_Flag = 0;
    Current_Height = PrimaryHeight;
    vTaskDelete(NULL);
}

/**********************************************************************
 * @Name    Exit_Height_Upadte
 * @declaration :
 * @param   None
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Exit_Height_Upadte(void)
{
    Height_task_exit = 1;
}

/**********************************************************************
 * @Name    Start_Read_Switch
 * @declaration : 启动轻触开关任务
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Start_Read_Switch(void)
{
    if (read_task_exit)
    {
        read_task_exit = 0; //标记任务开启
        //初始化数组
        memset(swicth_status, 0, sizeof(swicth_status));
        memset(HW_Switch, 0, sizeof(HW_Switch));

        /* definition and creation of Read_Swicth_tas */
        osThreadDef(Read_Swicth_tas, Read_Swicth, osPriorityAboveNormal, 0, 128);
        Read_Swicth_tasHandle = osThreadCreate(osThread(Read_Swicth_tas), NULL);
    }
}

/**********************************************************************
 * @Name    Read_Swicth
 * @declaration : 任务函数实现核心函数
 * @param   argument: [输入/出] 无意义
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Read_Swicth(void const *argument)
{
    while (!read_task_exit)
    {
        //因为GPIO口被配置成pullup，所以只有在低时才是轻触开关导通状态
        if (Get_SW(1) == 1)
            SWITCH(1) = on; //开关状态枚举
        else
            SWITCH(1) = off;

        if (Get_SW(2) == 1)
            SWITCH(2) = on;
        else
            SWITCH(2) = off;

        if (Get_SW(3) == 1)
            SWITCH(3) = on;
        else
            SWITCH(3) = off;

        if (Get_SW(4) == 1)
            SWITCH(4) = on;
        else
            SWITCH(4) = off;

        if (Get_SW(5) == 1)
            SWITCH(5) = on;
        else
            SWITCH(5) = off;

        if (Get_SW(6) == 1)
            SWITCH(6) = on;
        else
            SWITCH(6) = off;

        if (Get_SW(7) == 1)
            SWITCH(7) = on;
        else
            SWITCH(7) = off;

        if (Get_SW(8) == 1)
            SWITCH(8) = on;
        else
            SWITCH(8) = off;
        //红外开关部分
        if (HAL_GPIO_ReadPin(HW_S1_GPIO_Port, HW_S1_Pin) == GPIO_PIN_SET)
            HW_SWITCH(1) = off;
        else
            HW_SWITCH(1) = on;

        if (HAL_GPIO_ReadPin(HW_S2_GPIO_Port, HW_S2_Pin) == GPIO_PIN_SET)
            HW_SWITCH(2) = off;
        else
            HW_SWITCH(2) = on;

        if (HAL_GPIO_ReadPin(HW_S3_GPIO_Port, HW_S3_Pin) == GPIO_PIN_SET)
            HW_SWITCH(3) = off;
        else
            HW_SWITCH(3) = on;
        if (HAL_GPIO_ReadPin(HW_S4_GPIO_Port, HW_S4_Pin) == GPIO_PIN_SET)
            HW_SWITCH(4) = off;
        else
            HW_SWITCH(4) = on;
        //定高红外
        if (HAL_GPIO_ReadPin(HW_Height1_GPIO_Port, HW_Height1_Pin) == GPIO_PIN_SET)
            Height_SWITCH(1) = off;
        else
            //侧边红外
            Height_SWITCH(1) = on;

        if (HAL_GPIO_ReadPin(HW_Height2_GPIO_Port, HW_Height2_Pin) == GPIO_PIN_SET)
            Height_SWITCH(2) = off;
        else
            //侧边红外
            Height_SWITCH(2) = on;

        if (HAL_GPIO_ReadPin(Side_HW1_GPIO_Port, Side_HW1_Pin) == GPIO_PIN_SET)
            Side_SWITCH(1) = off;
        else
            Side_SWITCH(1) = on;
        if (HAL_GPIO_ReadPin(Side_HW2_GPIO_Port, Side_HW2_Pin) == GPIO_PIN_SET)
            Side_SWITCH(2) = off;
        else
            Side_SWITCH(2) = on;

        osDelay(10); //对请求的频率不高,所以可以10ms来单次刷新
    }
    memset(swicth_status, err, sizeof(swicth_status)); //清空到未初始状态，用于标记此时任务未运行
    vTaskDelete(NULL);                                 //从任务列表中移除该任务
    Read_Swicth_tasHandle = NULL;                      //句柄置空
}

/**********************************************************************
 * @Name    Exit_Swicth_Read
 * @declaration : 退出查询开关状态的任务
 * @param   None
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Exit_Swicth_Read(void)
{
    read_task_exit = 1; //此变量为1 将使得任务函数循环条件不满足
}

/**********************************************************************
 * @Name    Get_Switch_Status
 * @declaration : 获取指定ID开关的通断状态
 * @param   id: [输入/出] 电机编号（1-8）
 * @retval   : 该开关的通断状态
 * @author  peach99CPP
 ***********************************************************************/
int Get_Switch_Status(int id)
{
    if (read_task_exit)
        return err; //确保当前任务处在进行中
    if (id < 1 || id > 10)
        return off; // todo当有开关数量更新时记得修改此处的值
    //上面两个是为了避免出错的判断条件
    return SWITCH(id); //返回对应开关的状态
}

/**********************************************************************
 * @Name    Get_HW_Status
 * @declaration :获取指定ID号红外开关的状态
 * @param   id: [输入/出] 红外开关编号
 * @retval   : 状态
 * @author  peach99CPP
 ***********************************************************************/
int Get_HW_Status(int id)
{
    // todo当有开关数量更新时记得修改此处的值
    if (read_task_exit || (id < 1 || id > 8)) //输入值限制避免出错
        return err;
    return HW_SWITCH(id);
}

/**********************************************************************
 * @Name    Get_Side_Switch
 * @declaration :获取侧边边界开关的状态
 * @param   id: [输入/出] 开关编号
 * @retval   : 状态  扫到了就为on
 * @author  peach99CPP
 ***********************************************************************/
int Get_Side_Switch(int id)
{
    if (id < 0 || id > 3) // todo当有开关数量更新时记得修改此处的值
        return off;
    return Side_SWITCH(id);
}
int Get_Height_Switch(int id)
{
    if (id < 1 || id > 2)
        return off; // todo 记得在更新元器件数量后更新次此处的限制范围
    return Height_SWITCH(id);
}

/**********************************************************************
 * @Name    Get_Height
 * @declaration : 获取此时高度以计算应该调用的动作组编号
 * @param   None
 * @retval   : 动作组编号
 * @author  peach99CPP
 ***********************************************************************/
int Get_Height(void)
{
    return Current_Height;
}

/**
 * @name: Set_SwitchSpeed
 * @brief: 设置撞击挡板的速度
 * @param {int} speed  想要以什么速度
 * @return {*}
 */
void Set_SwitchSpeed(int speed)
{
    MIN_SPEED = speed;
}
/**********************************************************************
 * @Name    Wait_Switches
 * @declaration :碰撞轻触开关的实现全过程
 * @param   dir: [输入/出] 方向 1正Y 3 正X 4 负Y
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Wait_Switches(int dir)
{
    /*关于运行时速度的变量,不宜过高否则不稳定*/
    int Switch_Factor = 40;

    // if (read_task_exit)
    //     Start_Read_Switch();

    //    track_status(1, 0); //关闭循迹版，避免造成方向上的影响
    //    track_status(2, 0);

    volatile static short flag1, flag2;
    short x_pn, y_pn;
    int w1, w2, w1_factor, w2_factor;
    w1_factor = 1, w2_factor = -1;

    //关于参数的解析，根据方向来判定速度的分配方向
    if (dir == 1) //正X
    {
        // todo 经测试 速度60满足要求
        w1 = 6, w2 = 0;
        x_pn = 1, y_pn = 0;
    }
    else if (dir == 2) //负x方向
    {
        w1 = 1, w2 = 7; //视情况改变
        x_pn = 1, y_pn = 0;
        return;
    }
    else if (dir == 3) //负X方向
    {
        w1 = 1, w2 = 7;
        x_pn = -1, y_pn = 0;
    }
    else if (dir == 4) //负Y方向
    {
        w1 = 3, w2 = 5;
        x_pn = -1, y_pn = 0;
    }
    MIN_SPEED = 60;//TODO 增大碰撞时候的力度 
//开始靠近
Closing:
#define overtimeval 8000
    flag1 = flag2 = 0;
    set_speed(MIN_SPEED * x_pn, MIN_SPEED * y_pn, 0); //设置一个基础速度，此速度与方向参数有关
    //等待开关都开启
    int dlt = 0;
    do
    {
        flag1 = Get_SW(w1); //获取状态
        flag2 = Get_SW(w2);
        if (flag1 == on || flag2 == on) //当有开关触碰到时，关闭陀螺仪
            set_imu_status(0);          //关闭陀螺仪,否则设置w速度无意义
        /*下面这一句语句，只在单个开关开启时会有作用*/
        w_speed_set(Switch_Factor * (flag1 * w1_factor + flag2 * w2_factor));
        //任务调度
        osDelay(10);
        dlt += 10;              //超时保护机制 防止出现卡死的情况
        if (dlt >= overtimeval) //防止卡死 设置最长等待时间
        {
            printf("%ds时间阈值到 超时退出\n", (overtimeval / 1000));
            dlt = 0;
            goto switch_exit;
        }
    } while (flag1 == off || flag2 == off); //只有两个都接通，才退出该循环
    osDelay(300);
    if (flag1 == off || flag2 == off)
    {
        MIN_SPEED -= 5.0; //不使用除法了 尝试使用减法
        if (ABS(MIN_SPEED) < 5)
            goto switch_exit; //防止卡死在这里
        goto Closing;         //继续回到靠近的程序
    }
switch_exit:
    //    Exit_Swicth_Read(); //用完了就关闭任务
    set_speed(0, 0, 0); //速度置0 防止此时出现移动
    Set_InitYaw(0);     //修正车身角度
    set_imu_status(1);  //修正后再打开陀螺仪
    // todo：调用完函数根据实际需要进行陀螺仪角度的修正
}

/**********************************************************************
 * @Name    HWSwitch_Move
 * @brief  单独使用红外来移动到平台的一侧
 * @param   dir: [输入/出]  贴边移动的方向 1 2 为左右 5 6为侧边的 具体参考阶梯平台
 * @param 	 enable_imu: [输入/出]  是否使能陀螺仪
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void HWSwitch_Move(int dir, int enable_imu)
{
#define Speed_Factor_HW 1.2
    Set_IMUStatus(enable_imu);
    if (dir == 0)
    {
        while (Get_HW(dir) == off)
        {
            set_speed(0, -MIN_ * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
            osDelay(5);
        }
        set_speed(VERTICAL, MIN_ * Speed_Factor_HW, 0);
        while (Get_HW(dir) == on)
        {
            osDelay(10);
        }
    }
    else if (dir == 1)
    {
        while (Get_HW(dir) == off)
        {
            set_speed(0, -MIN_ * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
            osDelay(5);
        }
        set_speed(-VERTICAL, MIN_ * Speed_Factor_HW, 0);
        while (Get_HW(dir) == on)
        {
            osDelay(10);
        }
    }
    else if (dir == 2)
    {
        while (Get_HW(dir) == off)
        {
            set_speed(0, MIN_ * Speed_Factor_HW, 0); //把速度增大 避免出现无法启动的现象
            osDelay(5);
        }
        set_speed(VERTICAL, -MIN_ * Speed_Factor_HW, 0);
        while (Get_HW(dir) == on)
        {
            osDelay(10);
        }
    }
    set_speed(0, 0, 0);
    osDelay(150);
}

/**********************************************************************
 * @Name    Single_Switch
 * @declaration :检测单边开关
 * @param   switch_id: [输入/出]  开关号 1-8
 * @retval   : 无
 * @author  peach99CPP
 ***********************************************************************/
void Single_Switch(int switch_id)
{
    //    Set_IMUStatus(false); //直接抵着墙撞击，无需陀螺仪稳定角度
    short x, y; //不同方向的速度因子
    short x_vertical, y_vertical;
    int status;         //存储开关状态的变量
    if (read_task_exit) //确保开关的开启状态
        Start_Read_Switch();
    switch (switch_id) //分配速度方向和垂直板子的速度方向
    {
    case 1:
        x = -1, y = 0;
        x_vertical = 0, y_vertical = 1;
        break;
    case 2:
        x = 1, y = 0;
        x_vertical = 0, y_vertical = 1;
        break;
    case 3:
        x = 0, y = 1;
        x_vertical = -1, y_vertical = 0;
        break;
    case 4:
        x = 0, y = -1;
        x_vertical = -1, y_vertical = 0;
        break;
    case 5:
        x = 0, y = 1;
        x_vertical = 1, y_vertical = 0;
        break;
    case 6:
        x = 0, y = -1;
        x_vertical = 1, y_vertical = 0;
        break;
    case 7:
        x = -1, y = 0;
        x_vertical = 0, y_vertical = -1;
        break;
    case 8:
        x = 1, y = 0;
        x_vertical = 0, y_vertical = -1;
        break;
    default:
        x = 0, y = 0;
        x_vertical = 0, y_vertical = 0;
    }
RECLOSE:
    while (Get_Switch_Status(switch_id) != on)
    {
        set_speed(x_vertical * VERTICAL, y_vertical * VERTICAL, 0);
        osDelay(5);
    }
    osDelay(500);
    if (Get_Switch_Status(switch_id) != on)
        goto RECLOSE;
    set_speed(x * MIN_ + x_vertical * VERTICAL, y * MIN_ + y_vertical * VERTICAL, 0); //给一个速度,经测试需要在垂直方向上也给一个速度值避免车身被反弹
    do
    {
        status = Get_Switch_Status(switch_id); //获取状态
        if (status == err)
            Start_Read_Switch(); //防止此时任务退出而卡死在循环里
        osDelay(20);             //任务调度
    } while (status == on);      //直到开关断开，此时说明到达边界
    set_speed(0, 0, 0);          //停车
}

/**********************************************************************
 * @Name    Set_SwitchParam
 * @declaration : 调试所用途的函数接口
 * @param   main: [输入/出] 主要速度，沿着边沿移动的速度
 **			 vertical: [输入/出]  垂直与边沿的速度，来确保紧贴的状态
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void Set_SwitchParam(int main, int vertical)
{
    //调试速度的API
    MIN_ = main;         //沿着板子水平方向的速度
    VERTICAL = vertical; //垂直板子的速度，确保紧贴着。
}
/**********************************************************************
 * @Name    QR_Scan
 * @declaration :使用二维码进行阶梯平台的扫描
 * @param   status: [输入/出]  是否开启
 **			 color: [输入/出]  要抓的颜色，用于判断高度  代表红色 2代表蓝色
 **			 dir: [输入/出]    方向
 * @retval   :
 * @author  peach99CPP
 ***********************************************************************/
void QR_Scan(int status, int color, int dir, int enable_imu)
{
    ;
}

/**
 * @name:
 * @brief:
 * @param {*}
 * @return {*}
 */
void Ring_Move(void)
{
#define Ring_Action 42  //对应动作组的编号
#define Ring_HW_ID 1    //复用的红外的编号
#define Side_Factor 0.3 //为保持方向给一个垂直速度
    Set_IMUStatus(false);
    set_speed(MIN_ * Side_Factor, MIN_, 0);    //开始动起来
    while (Get_Side_Switch(Ring_HW_ID) == off) //复用高度的红外进行位置的判断
        osDelay(1);                            //给系统调度时间
    set_speed(0, 0, 0);                        //到达位置马上停车
    Action_Gruop(Ring_Action, 1);              //执行对应的动作组
    while (!Get_Servo_Flag())                  //等待返回信息
        osDelay(5);
    Set_InitYaw(0); //每次这里都对的很准 所以直接用这里的角度来修正车身
    Set_IMUStatus(true);
}

void Disc_Mea(void)
{
    int mode = Get_TargetColor(); // 1红2蓝
    MV_Stop();                    //关闭
    Set_MV_Mode(false);           //防止误响应 关闭
    Action_Gruop(25, 1);          //执行预备动作组
    Wait_Servo_Signal(2000);      //等待动作组
    Action_Gruop(40, 1);          //复位爪子
    Wait_Servo_Signal(2000);      //等待动作组
    MV_Start();                   //开启openmv
    printf("\n开始靠近圆盘机\n"); // log输出
    Wait_Switches(1);             //撞击挡板修正角度以及定位
    Set_IMUStatus(false);
    set_speed(0, VERTICAL, 0); //确保撞击是直的。
    if (mode == 1)
    {
        printf("\n 红色执行45号动作组\n");
        MV_SendCmd(6, Get_TargetColor()); // 1红2蓝 3黄
        Action_Gruop(45, 1);              //执行第二个动作组
    }
    else if (mode == 2)
    {
        printf("\n 蓝色执行35号动作组\n");
        MV_SendCmd(7, Get_TargetColor()); // 1红2蓝 3黄
        Action_Gruop(35, 1);
    }
    printf("\n开始扫描\n");   // log输出
    Wait_Servo_Signal(2000);  //等待动作组完成
    osDelay(10);              //缓冲时间
    osDelay(25 * 1000);       // 圆盘机等待时间
    printf("\n收起\n");       // log输出
    while (!Get_Servo_Flag()) //确认此时不是在繁忙状态
        osDelay(5);
    Action_Gruop(24, 1);     //结束的动作组
    Wait_Servo_Signal(2000); //等待动作组完成
    MV_Stop();               //结束 彻底关闭Mv 避免误响应
    Set_IMUStatus(true);     //恢复陀螺仪使能状态
}
