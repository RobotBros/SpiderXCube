/**
  ******************************************************************************
  * @file    SpiderXCube/FLASH_SaveData/Src/main.c
  * @author  Enix Yu, RobotBros.cn
  * @version V0.1.0
  * @date    12-Mar-2016
  * @brief   FLASH��д����
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 RobotBros.cn</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of RobotBros.cn nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "main.h"
    
/* Private variables **********************************************************/
/* Global variables ***********************************************************/

static u8  sampleData1[5]  = {0x01, 0x02, 0x03, 0x04, 0x05};
static u8  sampleData2[6]  = {0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b};
static u32 saveDataaddress = FLASH_DATA_STARTADDR;

#define PAD                 0xff

/* Private functions **********************************************************/

static Boolean Mem_Flash_Compare(u8 *memBuffer, u8 *flashBuffer, u8 len);

/* Public functions ***********************************************************/

/**
 *@brief  Main function
 *@param  None
 *@retval None
 */
int main(void)
{
    Boolean error = FALSE;
    
    /* Systick Init */
    HAL_InitTick(IRQ_SYSTICK_Priority, 0);
    
    LED_Init();
    
    /* ��ʼһ��flashд������ */
    Flash_Sync_TransWriteBegin();
    
    /* д��ǰ�������� */
    Flash_Sync_TransErase(saveDataaddress, sizeof(sampleData1) + sizeof(sampleData2));
    
    /* д��sampleData1 */
    Flash_Sync_TransWrite(saveDataaddress, sampleData1, sizeof(sampleData1), PAD);
    
    /* д��sampleData2 */
    Flash_Sync_TransWrite(saveDataaddress + sizeof(sampleData1), sampleData2, sizeof(sampleData2), PAD);
    
    /* �������� */
    Flash_Sync_TransWriteEnd();
    
    /* ���д��������Ƿ���ȷ */
    if (Mem_Flash_Compare((u8 *)saveDataaddress, sampleData1, sizeof(sampleData1)))
    {
        if (Mem_Flash_Compare((u8 *)saveDataaddress, sampleData1, sizeof(sampleData1)))
            error = FALSE;
        else
            error = TRUE;
    }
    else
    {
        error = TRUE;
    }
    
    while(1)
    {
        if (error)
            LED_flash(1, 1000);
        else
            LED_flash(1, 2000);
    }
}

/**
 *@brief  ���memBuffer��flashBuffer��ÿһ���ֽ��Ƿ����
 *@param  memBuffer:�ڴ滺����
 *@param  flashBuffer:flash������
 *@param  len:����������
 *@retval TRUE - ���; FALSE - ����
 */
static Boolean Mem_Flash_Compare(u8 *memBuffer, u8 *flashBuffer, u8 len)
{
    u8 i = 0;
    
    for (; i < len; i ++)
    {
        if (*(memBuffer + i) != *(flashBuffer + i))
            break;
    }
    
    return (i == len) ? TRUE : FALSE;
}
