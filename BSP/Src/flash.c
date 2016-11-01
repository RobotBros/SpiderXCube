/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Src/flash.c
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

#include "flash.h"
    
/* Private variables **********************************************************/
/* Global variables ***********************************************************/
/* Private functions **********************************************************/
/* Public functions ***********************************************************/

/**
 *@brief  ��ʼһ��д����
 *  Note: ��������󣬱������Flash_Sync_TransWriteEnd
 *@param  None
 *retval  None
 */
void Flash_Sync_TransWriteBegin(void)
{
    FLASH_UnlockBank1();
    FLASH_ClearFlag( FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR );
}

/**
 *@brief  �������FLASH����
 *  Note: ����ǰ���������Flash_Sync_TransWriteBegin��������
 *@param  address:   ��д������ݵ�flash��ʼ��ַ
 *@param  buffer:   ԭ���ݵĻ�������ַ
 *@param  writeLen: д������ݳ��� (��λ�ֽ�)
 *@retval None
 */
Boolean Flash_Sync_TransErase(uint32_t address, uint32_t writeLen)
{
    Boolean rc = FALSE;
    uint32_t i = 0, flash_start_addr, flash_addr_pt, number_of_pages;
    FLASH_Status flashStatus;
    
    flash_start_addr = address;
    if ( IS_VALID_DATA_ADDR(flash_start_addr) )
    {
        number_of_pages = writeLen / FLASH_PAGE_SIZE_BYTE;
        if ( number_of_pages * FLASH_PAGE_SIZE_BYTE < writeLen )
            number_of_pages ++;
        
        for (i = 0; i < number_of_pages; i ++)
        {
            flash_addr_pt = flash_start_addr + i * FLASH_PAGE_SIZE_BYTE;
            flashStatus = FLASH_ErasePage(flash_addr_pt);
            if ( flashStatus != FLASH_COMPLETE )
                break;
        }
        
        rc = ( i == number_of_pages ? TRUE : FALSE );
    }
    
    return rc;
}

/**
 *@brief  ͬ����ʽ����д��flash(��������)
 *   Note: ���ô˷�����ѭ�������̣�
 *         1. �������ȵ��� Flash_Sync_TransWriteBegin
 *         2. Ȼ����ò������� Flash_Sync_TransErase���������������漰��page
 *         3. ��������ϱ������ Flash_Sync_TransWriteEnd��������
 *@param  address:   ��д������ݵ�flashƫ����ʼ��ַ
 *@param  buffer:   ԭ���ݵĻ�������ַ
 *@param  writeLen: д������ݳ��� (��λ�ֽ�)
 *@param  pad:      ����ַ�����writeLen����2�ı��������һ���ֽڽ���pad���
 *@retval None
 */
Boolean Flash_Sync_TransWrite(uint32_t address, const uint8_t *buffer, uint32_t writeLen, uint8_t pad)
{
    return Flash_Sync_Write(address, buffer, writeLen, pad, TRUE);
}

/**
 *@brief  ����һ��д����
 *@param  None
 *retval  None
 */
void Flash_Sync_TransWriteEnd(void)
{
    FLASH_LockBank1();
}

/**
 *@brief  ͬ������д��flash
 *@param  address:   ��д������ݵ�flash��ʼ��ַ
 *@param  buffer:   ԭ���ݵĻ�������ַ
 *@param  writeLen: д������ݳ��� (��λ�ֽ�)
 *@param  pad:      ����ַ�����writeLen����2�ı��������һ���ֽڽ���pad���
 *@param  isTrans:  �Ƿ�Ϊ����ʽд��
 *@return None
 */
Boolean Flash_Sync_Write(uint32_t address, const uint8_t *buffer, uint32_t writeLen, uint8_t pad, Boolean isTrans)
{
    Boolean rc = FALSE;
    uint32_t i = 0, flash_start_addr, flash_addr_pt, write_buf_len = writeLen;
    FLASH_Status flashStatus;
    uint16_t buffer_temp;
    
    flash_start_addr = address;
    if ( IS_VALID_DATA_ADDR(flash_start_addr) )
    {
        if (!isTrans)
        {
            FLASH_UnlockBank1();
            FLASH_ClearFlag( FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR );
        }
        
        // д������ݷǰ��ֶ���, ������һλ���λ�������Ϊpad����ֵ
        if ( writeLen % 2 )
        {
            write_buf_len = writeLen + 1;
        }
        
        while( i < write_buf_len )
        {
            flash_addr_pt = flash_start_addr + i;

            if ( i != writeLen)
                *(((uint8_t *)&buffer_temp) + (i % 2)) = *(buffer + i);
            else
                // д�����ݷǰ��ֶ��룬���ѵ����һ���ֽ�
                *(((uint8_t *)&buffer_temp) + 1) = pad;
            
            if (!isTrans)
            {
                // д������ǰ���Ȳ���һҳ����
                if( i % FLASH_PAGE_SIZE_BYTE == 0 )
                {
                    flashStatus = FLASH_ErasePage(flash_addr_pt);
                    if ( flashStatus != FLASH_COMPLETE )
                        break;
                }
            }
            
            // ÿһ������Halfword��д������
            if ( i % 2 )
            {
                flashStatus = FLASH_ProgramHalfWord(flash_addr_pt - 1, buffer_temp);
                if ( flashStatus != FLASH_COMPLETE )
                    break;
            }
            
            i ++;
        }
        
        // �ж��Ƿ�ȫ��������д�����
        if ( i == write_buf_len )
            rc = TRUE;
        else
            rc = FALSE;
        
        if (!isTrans)
            FLASH_LockBank1();
    }
    else
    {
        rc = FALSE;
    }
    
    return rc;
}

/**
 *@brief  ͬ����ȡflash����
 *@param  address   ���ȡ�����ݵ�flashƫ����ʼ��ַ
 *@param  buffer   �����ȡ���ݵĻ�������ʼ��ַ
 *@param  readLen  ��ȡ�����ݳ��� (��λ�ֽ�)
 *@return None
 */
Boolean Flash_Sync_Read(uint32_t address, uint8_t *buffer, uint32_t readLen)
{
    Boolean rc = FALSE;
    uint32_t i = 0, flash_start_addr;
    
    flash_start_addr = address;
    if ( IS_VALID_DATA_ADDR(flash_start_addr) )
    {
        while( i < readLen )
        {
            *(buffer + i) = *((uint8_t *)flash_start_addr + i);
            i ++;
        }
        
        rc = TRUE;
    } 
    else 
    {
        rc = FALSE;
    }
    
    return rc;
}
