#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "track_bar_receive.h"
#include "uart_handle.h"
#include "chassis.h"
void track_scan(void const *argument)
{
	
    static long counter = 0;
    while (1)
    {
		 
        if (dma_count > 0)
        {
           
            Chassis_decode();
            //printf("%d,%d,%d",y_bar.line_num,x_leftbar.line_num,x_rightbar.line_num);
//			  if(Get_US()<0XC8)
//			  {
//				  Feng_Feng();
//			  }

        }
        counter = counter > 65536 ? 0 : counter;
        counter++;
        osDelay(1);
    }
}
