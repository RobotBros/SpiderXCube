/**
  ******************************************************************************
  * @file    SpiderXCube/BSP/Inc/flash.h
  * @author  Fishcan, RobotBros
  * @version V0.1.0
  * @date    03-Mar-2016
  * @brief   FLASH��д����
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, ROBOTBROS.  SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2016 Robotbros.Inc </center></h2>
  ******************************************************************************
  */

#ifndef __FLASH_H
#define __FLASH_H

#include "spiderx_bsp.h"

/* Global definition **********************************************************/

#define FLASH_BASE_ADDRESS                 0x08000000                           /*!<  Ƭ��FLASH��ʼ��ַ           */ 
#define FLASH_DATA_STARTADDR               FLASH_BASE_ADDRESS + 0x20000         /*!<  �û���������128K            */
#define IS_VALID_DATA_ADDR(ADDRESS)        ((ADDRESS) >= FLASH_DATA_STARTADDR) && IS_FLASH_ADDRESS((ADDRESS)) 

#if defined( STM32F10X_HD ) || defined( STM32F10X_XL ) || defined( STM32F10X_HD_VL ) || defined( STM32F10X_CL )
  #define FLASH_PAGE_SIZE_BYTE             2048                                 /*!<  ������PAGE SIZEΪ2K         */                     
#else
  #define FLASH_PAGE_SIZE_BYTE             1024                                 /*!<  ������PAGE SIZEΪ1K         */
#endif      

#define FLASH_DMA_READ_CHANNEL             DMA1_Channel1                        /*!<  ��ȡ����DMAͨ��             */
#define FLASH_DMA_WRITE_CHANNEL            DMA1_Channel4                        /*!<  д������DMAͨ��             */
#define FLASH_DMA_READ_IRQ                 DMA1_Channel1_IRQn                   /*!<  ��ȡ����DMA IRQ             */
#define FLASH_DMA_WRITE_IRQ                DMA1_Channel4_IRQn                   /*!<  д������DMA IRQ             */
#define FLASH_DMA_READ_TC                  DMA1_FLAG_TC1                        /*!<  ��ȡ����DMA����жϱ�־     */
#define FLASH_DMA_WRITE_TC                 DMA1_FLAG_TC4                        /*!<  д������DMA����жϱ�־     */

#define IS_PERIPHERAL_ADDRESS(ADDRESS)     ((ADDRESS) > PERIPH_BASE && (ADDRESS) < PERIPH_BASE + 0x1FFFFFFF)

/* Public functions ***********************************************************/

/**
 *@brief  ����ʽͬ��дFLASH
 */
void Flash_Sync_TransWriteBegin(void);
Boolean Flash_Sync_TransErase(uint32_t address, uint32_t writeLen);
Boolean Flash_Sync_TransWrite(uint32_t address, const uint8_t *buffer, uint32_t writeLen, uint8_t pad);
void Flash_Sync_TransWriteEnd(void);

/**
 *@brief  ͬ����ʽ��дFLASH
 */
Boolean Flash_Sync_Write(uint32_t address, const uint8_t *buffer, uint32_t writeLen, uint8_t pad, Boolean isTrans);
Boolean Flash_Sync_Read(uint32_t address, uint8_t *buffer, uint32_t readLen);

#endif /* __FLASH_H */
