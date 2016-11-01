/**
*����˹���麯����
*zhurong90s@2013��10��14��13:27:42
*/
#include "stm32f10x.h"
struct BOARD Table_board[Vertical_boxs] [Horizontal_boxs];
struct SHAPE Boxs[MAX_BOX] =
{
	{0x88c0,CYAN,	1},
	{0xe800,CYAN,	2},
	{0xc440,CYAN,	3},
	{0x2e00,CYAN,	0},
	
	{0x44c0,MAGENTA,5},
	{0x8e00,MAGENTA,6},
	{0xc880,MAGENTA,7},
	{0xe200,MAGENTA,4},
	
	{0x8c40,YELLOW,	9},
	{0x6c00,YELLOW,	8},
	
	{0x4c80,BLUE2,	11},
	{0xc600,BLUE2,	10},
	
	{0x4e00,WHITE,	13},
	{0x8c80,WHITE,	14},
	{0xe400,WHITE,	15},
	{0x4c40,WHITE,	12},
	
	{0x8888,RED,	17},
	{0xf000,RED,	16},
	
	{0xcc00,BLUE, 18}
};

//----------------------------------
//	��������systick_init
//	���ܣ���ʼ��systick��ʱ����
//----------------------------------
void Systick_Init(void)
{
	//��systick�ж�
    	NVIC_SetPriority(SysTick_IRQn, 0x00);
	
	//����systickʱ��
	SysTick->LOAD = 0x0afc80;

	SysTick->CTRL = ( + SysTick_CTRL_TICKINT + SysTick_CTRL_CLKSOURCE );
}

//---------------------------------------------------------
//	��������frame_init
//	���ܣ���ʼ����ʾ���ʹ�������ʾ����ʼsystickʱ�ӣ���ʾpoint����
//	 
//---------------------------------------------------------
void Frame_Init (void)
{	
	//���������
	u16 cur_x,cur_y;
	//u8 up[] = "Up",down[] = "Down",lift[] = "Lift",ringt[] = "Right";
	
	Point_color = GREEN;
	Back_color = BLACK;
	Lcd_Clear(Back_color,0,0);
	
	for(cur_x = SYS_STARX;cur_x < SYS_ENDX;cur_x += BSIZE)
	{
		for(cur_y = SYS_STARY;cur_y < SYS_ENDY;cur_y += BSIZE)
		{
			GUI_DrawLine(cur_x,		   cur_y,		   cur_x,	         cur_y+BSIZE);
			GUI_DrawLine(cur_x+BSIZE,cur_y,		   cur_x+BSIZE,cur_y+BSIZE);

			GUI_DrawLine(cur_x,		   cur_y,		   cur_x+BSIZE, cur_y		);
			GUI_DrawLine(cur_x,		   cur_y+BSIZE, cur_x+BSIZE, cur_y+BSIZE);
		}
	}
	
	//GUI_DrawLine(2,104,113,104);
	
	//���ư���
	//��һ��
	//ssd1289_square_full(290,5,315,45,BUTTON_UP);
	//ssd1289_putchs(295,10,Black,CoppyB,lift);

	//�ڶ���
	//ssd1289_square_full(290,68,315,108,BUTTON_UP);
	//ssd1289_putchs(295,68,Black,CoppyB,ringt);

	//������
	//ssd1289_square_full(290,131,315,171,BUTTON_UP);
	//ssd1289_putchs(295,136,Black,CoppyB,down);

	//���ĸ�
	//ssd1289_square_full(290,194,315,234,BUTTON_UP);
	//ssd1289_putchs(295,204,Black,CoppyB,up);

	//��ʼ��board���� ��
	for(cur_x = 0;cur_x < Vertical_boxs;cur_x ++)
	{
		for(cur_y = 0;cur_y < Horizontal_boxs;cur_y ++)
		{
			Table_board[cur_x][cur_y].var = 0;
			Table_board[cur_x][cur_y].color = BLACK;
		}
				
	}
	//�򿪵δ�ʱ��
	SysTick->CTRL |= SysTick_CTRL_ENABLE;
	//��ʼ����������
	show_point(0);		
}

//---------------------------------------------
// 	��������show_box
//	���ܣ���ָ��λ����ʾbox��
//	������(x,y)Ϊboxλ�ã�nuΪbox�����кš�
//---------------------------------------------
void Show_Box(u16 x ,u16 y,u8 nu)
{
	u8 i;
	u16 yt,xt;

	Point_color = Boxs[nu].clor;
	if((x<LCD_X_MAX_PIXEL)&&(y<LCD_Y_MAX_PIXEL))
	{
		
		for(i=0;i<16;i++)
		{
			if(i<4)
			{
				if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
				{
					xt = x;
					yt = y+i*BSIZE;
					GUI_DrawRectangle(xt,yt,xt+BSIZE,yt+BSIZE);
				}
			}else if(i<8)
			{
				if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
				{
					xt = x + BSIZE;
					yt = y+(i-4)*BSIZE;
					GUI_DrawRectangle(xt,yt,xt+BSIZE,yt+BSIZE);
				}
			}else if(i<12)
			{
				if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
				{
					xt = x + BSIZE*2;
					yt = y+(i-8)*BSIZE;
					GUI_DrawRectangle(xt,yt,xt+BSIZE,yt+BSIZE);
				}
			}else if(i<16)
			{
				if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
				{
					xt = x + BSIZE*3;
					yt = y+(i-12)*BSIZE;
					GUI_DrawRectangle(xt,yt,xt+BSIZE,yt+BSIZE);
				}
			}
		}
	}	
}

//--------------------------------------------
// 	��������eraser_box
//	���ܣ�����ָ��λ�õ�box��
//	������(x,y)Ϊbox��λ�á�nuΪbox�ı�š�
//--------------------------------------------
void Eraser_Box(u16 x ,u16 y,u8 nu)
{
	u8 i;
	u16 yt,xt;

	Point_color = BLACK;
	if((x<LCD_X_MAX_PIXEL)&&(y<LCD_Y_MAX_PIXEL))
	{
		
		for(i=0;i<16;i++)
		{
			if(i<4)
			{
				if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
				{
					xt = x;
					yt = y+i*BSIZE;
					GUI_DrawRectangle(xt,yt,xt+BSIZE,yt+BSIZE);	
				}
			}else if(i<8)
			{
				if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
				{
					xt = x + BSIZE;
					yt = y+(i-4)*BSIZE;
					GUI_DrawRectangle(xt,yt,xt+BSIZE,yt+BSIZE);
				}
			}else if(i<12)
			{
				if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
				{
					xt = x + BSIZE*2;
					yt = y+(i-8)*BSIZE;
					GUI_DrawRectangle(xt,yt,xt+BSIZE,yt+BSIZE);
				}
			}else if(i<16)
			{
				if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
				{
					xt = x + BSIZE*3;
					yt = y+(i-12)*BSIZE;
					GUI_DrawRectangle(xt,yt,xt+BSIZE,yt+BSIZE);
				}
			}
		}
	}	
}

//----------------------------------------------
//	��������mov_able
//	���ܣ��ж�box�Ƿ����ƶ���
//	������(x,y)�ƶ����λ�ã�nuΪbox�����кš�
//	      ����ֵΪable��unable��	
//----------------------------------------------
u8 Mov_Able(u16 x ,u16 y,u8 nu)
{
	u8 i;
	u16 yt = y,xt = x;
	
	for(i=0;i<16;i++)
	{
		if(i<4)
		{
			if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
			{
				xt = x;
				yt = y+i*BSIZE;
				if((xt<SYS_STARX)||(xt>=SYS_ENDX)||(yt<SYS_STARY)||(yt>=SYS_ENDY))
					return unable;
				if(Table_board[xt/BSIZE][yt/BSIZE].var == 1)
					return unable;
						
			}
		}else if(i<8)
		{
			if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
			{
				xt = x + BSIZE;
				yt = y+(i-4)*BSIZE;
				if((xt<SYS_STARX)||(xt>=SYS_ENDX)||(yt<SYS_STARY)||(yt>=SYS_ENDY))
					return unable;
				if(Table_board[xt/BSIZE][yt/BSIZE].var == 1)
					return unable;	
			}
		}else if(i<12)
		{
			if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
			{
				xt = x + BSIZE*2;
				yt = y+(i-8)*BSIZE;
				if((xt<SYS_STARX)||(xt>=SYS_ENDX)||(yt<SYS_STARY)||(yt>=SYS_ENDY))
					return unable;
				if(Table_board[xt/BSIZE][yt/BSIZE].var == 1)
					return unable;	
			}
		}else if(i<16)
		{
			if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
			{
				xt = x + BSIZE*3;
				yt = y+(i-12)*BSIZE;
				if((xt<SYS_STARX)||(xt>=SYS_ENDX)||(yt<SYS_STARY)||(yt>=SYS_ENDY))
					return unable;
				if(Table_board[xt/BSIZE][yt/BSIZE].var == 1)
					return unable;	
			}
		}
		
	}

	return able;
}

//-------------------------------------------------------------
//	��������updata_board
//	���ܣ� ��һ��box��ӵ�board�ϡ�����һ�������������С�
//		   ˢ��board��ʾ������������������
//	��������������ֵΪ���̵��������ĺ�������x��y��ΪҪ����board��box��
//		  nuΪҪ����box�����кš�
//-------------------------------------------------------------
u8 Updata_Board(u16 x ,u16 y,u8 nu)
{
	u8 i,j,row=0;
	u16 yt = y,xt = x;
	
	//��box��ӵ�board��
	for(i=0;i<16;i++)
	{
		if(i<4)
		{
			if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
			{
				xt = x;
				yt = y+i*BSIZE;
				Table_board[xt/BSIZE][yt/BSIZE].var = 1;
				Table_board[xt/BSIZE][yt/BSIZE].color = Boxs[nu].clor; 					
			}
		}else if(i<8)
		{
			if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
			{
				xt = x + BSIZE;
				yt = y+(i-4)*BSIZE;
				Table_board[xt/BSIZE][yt/BSIZE].var = 1;
				Table_board[xt/BSIZE][yt/BSIZE].color = Boxs[nu].clor;	
			}
		}else if(i<12)
		{
			if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
			{
				xt = x + BSIZE*2;
				yt = y+(i-8)*BSIZE;
				Table_board[xt/BSIZE][yt/BSIZE].var = 1;
				Table_board[xt/BSIZE][yt/BSIZE].color = Boxs[nu].clor;	
			}
		}else if(i<16)
		{
			if((Boxs[nu].box>>(15 - i)&0x0001) == 0x0001)
			{
				xt = x + BSIZE*3;
				yt = y+(i-12)*BSIZE;
				Table_board[xt/BSIZE][yt/BSIZE].var = 1;
				Table_board[xt/BSIZE][yt/BSIZE].color = Boxs[nu].clor;	
			}
		}
	}
	//����Ƿ��ֿ�������
	//��δ���Ч���е��
	//���Կ���ȫ���һ�飬Ҫ����������
	//��ͳһ��������ʱ���ٳ���һ��
	i = 14;
	do{
		 i--;
		 j=0;
		 for(j=0;j<9;j++)
		 {
		 	if(Table_board[i][j].var != 1) break;
		 }
		 //�ж��Ƿ�����
		 if(j == 9)
		 {
		 	row++;
			xt=i;
		 	while(xt > 0)
			{
				for(yt=0;yt<9;yt++)
				{
					Table_board[xt][yt].var = Table_board[xt - 1][yt].var;
					Table_board[xt][yt].color = Table_board[xt - 1][yt].color;	
				}
				xt--;
			}
			
			for(yt=0;yt<9;yt++)
			{
				Table_board[0][yt].var = 0;
				Table_board[0][yt].color = BLACK;	
			}
		 	i=14;				
		 }

	}while(i != 0);


	for(i=0;i<14;i++)
	{
		for(j=0;j<9;j++)
		{
			xt=i*BSIZE + 5;
			yt=j*BSIZE + 5;
			Point_color = Table_board[i][j].color;
			GUI_DrawRectangle(xt,yt,xt+BSIZE,yt+BSIZE);
		}
	}
	
	return row;	
}

//------------------------------------------
//	��������show_point
//	���ܣ���ʾ������
//	������pointΪҪ��ʾ�Ĳ�����
//------------------------------------------
void Show_Point(u8 point)
{
	Point_color = GREEN;
	Back_color = COPPYB;
	GUI_ShowString(0, 104, 12, "Point:", 0);
	Back_color = BLACK;
	GUI_ShowNum(49, 104, point, 3, 12);
}

