#ifndef __COMM_PC_H
#define __COMM_PC_H
#include "sys.h" 

u8 Check_sum(u8 *CheckAdrr, u8 CheckCnt);					//校验和计算,所有数据求和，最低四位求反
void comm_pc_cammand(void);												//PC命令处理
void comm_pc_cammand_reset(void);									//数据处理完成后BUFF清零，标志清理，帧计数清零
void comm_to_pc(void);												//Send slave data to PC
void Systerm_get_slave_data(void);							

#endif
