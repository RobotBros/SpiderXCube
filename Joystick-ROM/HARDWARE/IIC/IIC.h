#ifndef ___IIC_H
#define ___IIC_H
#include "sys.h"

//#include "define.h"
//#include "stm32f10x.h"

//typedef u8 char;
//
//#define false    0
//#define	true     1

#define SCL  PBout(10)
#define SDA  PBout(11)
#define SDA_read  PBin(11)


#define CLI()      __set_PRIMASK(1)  
#define SEI()      __set_PRIMASK(0)

#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))

#define true 1
#define false 0 
#define   u8  u8

#define TRUE  0
#define FALSE -1
/*
#define MPU6050_READRATE			1000	//6050读取频率
#define MPU6050_READTIME			0.001	//6050读取时间间隔
#define EE_6050_ACC_X_OFFSET_ADDR	0
#define EE_6050_ACC_Y_OFFSET_ADDR	1
#define EE_6050_ACC_Z_OFFSET_ADDR	2
#define EE_6050_GYRO_X_OFFSET_ADDR	3
#define EE_6050_GYRO_Y_OFFSET_ADDR	4
#define EE_6050_GYRO_Z_OFFSET_ADDR	5
*/
//0表示写
#define	I2C_Direction_Transmitter   0
//１表示读
#define	I2C_Direction_Receiver      1	 
/*====================================================================================================*/
/*====================================================================================================*/
u8 i2cWriteBuffer(u8 addr_, u8 reg_, u8 len_, u8 *data);
u8 i2cWrite(u8 addr_, u8 reg_, u8 data);
u8 i2cRead(u8 addr_, u8 reg_, u8 len, u8* buf);
void i2c_Init(void);
u16 i2cGetErrorCounter(void);
static void i2cUnstick(void);

int i2cwrite(u8 addr, u8 reg, u8 len, u8 * data);
int i2cread(u8 addr, u8 reg, u8 len, u8 *buf);
/*====================================================================================================*/
/*====================================================================================================*/
#endif
