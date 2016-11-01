#include "imu_motion.h"



float Pitch,Roll,Yaw;
static u8 imu_flag_100ms = 0;

/*************************************************
函数名：Imu_motion_read
功能：读取IMU DMP 四元素FIFO数据(Gyro and accel data),计算pitch roll yaw
入口参数：无
返回值：无
*************************************************/
void Imu_motion_read(void)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4];

	if(imu_flag_100ms)
	{
		if(System_key_type.new_key_type == KEY_PRESS_DOWN)
		{
			dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more);   // read DMP FIFO data

			if (sensors & INV_WXYZ_QUAT )
			{
				q0=quat[0] / q30;	
				q1=quat[1] / q30;
				q2=quat[2] / q30;
				q3=quat[3] / q30;
				Pitch  =  asin(2 * q1 * q3 - 2 * q0* q2)* 57.3*10; 							   		// pitch*10
    	 			Roll   =   atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3*10; 		// roll*10
				Yaw   = 	atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3*10;		   		// yaw*10

				GUI_ShowString(10, 70, 12, "Pitch:", 0);
				GUI_ShowNum(46, 70, Pitch/10, 3, 12);
				GUI_ShowString(64, 70, 12, ".", 0);
				GUI_ShowNum(70, 70, Pitch-(Pitch/10)*10, 1, 12);
				GUI_ShowString(76, 70, 12, "`", 0);
				
				GUI_ShowString(10, 82, 12, "Roll:", 0);
				GUI_ShowNum(40, 82, Roll/10, 3, 12);
				GUI_ShowString(58, 82, 12, ".", 0);
				GUI_ShowNum(64, 82, Roll-(Roll/10)*10, 1, 12);
				GUI_ShowString(70, 82, 12, "`", 0);
				
				GUI_ShowString(10, 94, 12, "Yaw:", 0);
				GUI_ShowNum(34, 94, Yaw/10, 3, 12);
				GUI_ShowString(52, 94, 12, ".", 0);
				GUI_ShowNum(58, 94, Yaw-(Yaw/10)*10, 1, 12);
				GUI_ShowString(64, 94, 12, "`", 0);
			}
		}
		imu_flag_100ms = 0;												 //清除定时标记
	}
}


//--------------------------------------
//系统IMU采样处理定时器 100ms
//--------------------------------------
void Imu_motion_timer(void)
{
	static u8 tick_4ms = 0;

	tick_4ms++;
	
	if (tick_4ms >= 25)			// 4*25 = 100ms
	{	
		tick_4ms=0;
		imu_flag_100ms = 1;
	}
}
