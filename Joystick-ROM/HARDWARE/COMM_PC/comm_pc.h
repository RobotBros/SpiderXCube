#ifndef __COMM_PC_H
#define __COMM_PC_H
#include "sys.h" 

u8 Check_sum(u8 *CheckAdrr, u8 CheckCnt);					//У��ͼ���,����������ͣ������λ��
void comm_pc_cammand(void);												//PC�����
void comm_pc_cammand_reset(void);									//���ݴ�����ɺ�BUFF���㣬��־����֡��������
void comm_to_pc(void);												//Send slave data to PC
void Systerm_get_slave_data(void);							

#endif
