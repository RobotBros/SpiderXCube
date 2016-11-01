/****************************************Copyright (c)****************************************************
**                    
**                 
**
**--------------File Info---------------------------------------------------------------------------------
** File name:      main.c
** Last modified  Date:      
** Last Version:  1.0
** Descriptions:    
**            
**--------------------------------------------------------------------------------------------------------
** Created by:      FiYu
** Created date:    2014-8-5
** Version:        1.0
** Descriptions:    USB HID �����շ�����ʵ�����
**          ͨ��USB HID����nRF24LU1P�����߲���:�����ŵ��ͽ������ݳ���
**          ���շ���˵�������Ϣ��ͨ��USB HID�ϴ�
**          ͨ��USB HID����Ϣ���͸�nRF24LU1P
**          
**--------------------------------------------------------------------------------------------------------
** Modified by:      EnixYu
** Modified date:    2016-09-24
** Version:          1.1
** Descriptions:    ����3·����Ƕ���չ���޸ķ��Ͱ�У��ͼ��㡣
**
** Rechecked by:      
**********************************************************************************************************/
#include "nrf24lu1p.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "hal_nrf.h"
#include "hal_usb.h"
#include "hal_usb_hid.h"
#include "usb_map.h"
#include "hal_flash.h"
#include "hal_delay.h"


/*-----------------------------------------------------------------------------
** USB��ر�������
-----------------------------------------------------------------------------*/
static xdata uint8_t usb_in_buf[EP1_2_PACKET_SIZE];
static xdata uint8_t usb_out_buf[EP1_2_PACKET_SIZE];
static bool xdata app_usb_out_data_ready = false;                  // �ж�USB send ��ɱ�־
static bool xdata system_ctrl_state_flag = false;                    // �ж��Ƿ�PC����ģʽ                  
extern code const usb_string_desc_templ_t g_usb_string_desc;
static bool xdata app_pending_usb_write = false;

/*-----------------------------------------------------------------------------
** RF��ر�������
-----------------------------------------------------------------------------*/
#define NRF_DATA_LEN  32
static uint8_t xdata rf_rx_buf[NRF_DATA_LEN];
static uint8_t xdata rf_tx_buf[NRF_DATA_LEN];
static bool xdata nrf_rx_packet_received = false;                      //NRF RX�Ƿ���ɱ�־λ
static bool xdata radio_busy = false;
static bool xdata transmitted = false;

const uint8_t TX_RX_ADDRESS[5] = {0x34,0x43,0x10,0x10,0x01};             // TX/RX��ַ 5�ֽ�


//-----------------------------------------------------------------------------
// Internal function prototypes
//-----------------------------------------------------------------------------
static void usb_send_to_pc(uint8_t * buf, uint8_t size);                  // data to PC
static void nrf_usb_out_packet();                             //USB data to NRF
static void nrf_usb_in_packet();                                //NRF data to USB
static void app_wait_while_usb_pending();
static void rf_config(void);
static void nrf_usb_out_packet();
static void nrf_usb_in_packet();
void nrf_tx_mode(void);
void nrf_rx_mode(void);
uint8_t hal_nrf_write_reg(uint8_t reg, uint8_t value);
void hal_nrf_write_multibyte_reg(uint8_t reg, const uint8_t *pbuf, uint8_t length);\
uint8_t hal_nrf_read_reg(uint8_t reg);
uint16_t hal_nrf_read_multibyte_reg(uint8_t reg, uint8_t *pbuf);
static void nrf_tx_pack_send(void);
bool NRF_Check(void);

/*-----------------------------------------------------------------------------
** USB�ص���������
-----------------------------------------------------------------------------*/
hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req* req, uint8_t** data_ptr, uint8_t* size) large reentrant;
void suspend_cb(uint8_t allow_remote_wu) large reentrant;
void resume_cb() large reentrant;
void reset_cb() large reentrant;
uint8_t ep_1_in_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;
uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;

/*******************************************************************************************************
 * ��  �� : MAIN����
 * ��  �� : none
 * ����ֵ : none
 *******************************************************************************************************/
void main()
{
    P0DIR = 0xEF;                                                   // ����P0:P04����Ϊ���
    LED = 1;                                                        // Ϩ��ָʾ��

//*************************USB HAL initialization************************//
    hal_usb_init(true, device_req_cb, reset_cb, resume_cb, suspend_cb);   
    hal_usb_endpoint_config(0x81, EP1_2_PACKET_SIZE, ep_1_in_cb);      // Configure 32 bytes IN endpoint 1 
    hal_usb_endpoint_config(0x02, EP1_2_PACKET_SIZE, ep_2_out_cb);     // Configure 32 bytes OUT endpoint 2

    rf_config();                                                    // RF initial

    while(true)                                                     // ��ʼ��δNRF RXģʽ��һֱ��NRF RXģʽ�¹���������USB ���յ��������ָ��󣬴���һ��NRF TX                              
    {  
        if(hal_usb_get_state() == CONFIGURED)                       // check USB initial
        { 
            if(app_usb_out_data_ready == true)                      // USB ���յ��������͵�������ȷ��־
            {          
                app_usb_out_data_ready = false;                     // Claer USB ���ճɹ���־
            }
        }

        if(nrf_rx_packet_received == true)                          // RF����������� RX����
        {          
            usb_send_to_pc(usb_in_buf,EP1_2_PACKET_SIZE);           // USB data TO PC      
            nrf_rx_packet_received = false;                         // clear RF RX success flag
            LED = ~LED;                                             // RF RX data to USB out buffer and USB data send to PC success, flash
        }
        nrf_tx_pack_send();
        delay_ms(100);
    }  
}

//-----------------------------------------------------------------------------
//  NRF send  data ����ң�ذ���������ҡ�����ݳ���δ������spider���Զ����δ����ݣ�ϵͳ״̬�жϣ�ѡ�������ݰ�
//-----------------------------------------------------------------------------
static void nrf_tx_pack_send(void)
{
    uint8_t cnt,sum;

    if(system_ctrl_state_flag == false)                                     // �ж��Ƿ� PC��������ģʽ�������͹̶�ҡ�����ݣ��Ƿ��Ͷ����������
    {
        rf_tx_buf[0] = 0xAB;
        rf_tx_buf[1] = 0x32;                                                // ����ҡ��X/Y��50%����
        rf_tx_buf[2] = 0x32;
        rf_tx_buf[3] = 0x32;
        rf_tx_buf[4] = 0x32;
//************** nop data ***************//
        for(cnt=5; cnt<NRF_DATA_LEN; cnt ++)                                // δ�����������㴦��     
        {
          rf_tx_buf[cnt]= 0;
        }
//***************check sum**************//        
        sum = 0;
        for(cnt=0; cnt < NRF_DATA_LEN - 1; cnt ++)                          // calculate nrf rx buffer ckeck sum
        {
          sum += rf_tx_buf[cnt];
        }
        sum = sum ^ 0xFF;
        rf_tx_buf[NRF_DATA_LEN - 1] = sum;
    }

    nrf_tx_mode();                                                          // ���ù���ģʽ��TX  
    hal_nrf_flush_tx();                                                     // RF TX FIFO clear
    delay_ms(1);                     
    CE_LOW();
    hal_nrf_write_multibyte_reg(W_TX_PAYLOAD,rf_tx_buf,RX_PAYLOAD_LEN);    //д���ݵ�TX BUF  32���ֽ�
    CE_HIGH();
}

//-----------------------------------------------------------------------------
// Handle commands from Host application ����USB���ݺ�2.4G����  ����Ƕȿ�������
//-----------------------------------------------------------------------------
static void nrf_usb_out_packet(void)
{
  uint8_t sum,cnt;

//************** ckeck ͷ�� ****************************//
  if((!(usb_out_buf[0] == 0xFA))  || (! (usb_out_buf[1] == 0xFB)))  // USB ��������ͷ�� 0xFA 0xFB
    {return;}
//************** ckeck sum ****************************//
  sum = 0;
  for(cnt=0; cnt < 25; cnt ++)                              // calculate nrf rx buffer ckeck sum
  {
    sum += usb_out_buf[cnt];
  }
  sum = sum ^ 0xFF;
  if(!(sum == usb_out_buf[25]))                             //  when the sum check wrong, return ,keep the nrf tx buffer data do not change 
  {
    return;
  }
//*************system ctrl state check*******************//
  if(usb_out_buf[2] == 0xAA)                                // �ж��Ƿ�PC��������ģʽ��������Ϊ0xAA
  {
    system_ctrl_state_flag = true;
  }          
  else
  {
    system_ctrl_state_flag = false;
  }
//*****************RF TX pack************************//    // USB out data to nrf tx buffer
  sum = 0;
  for(cnt=2; cnt < EP1_2_PACKET_SIZE + 1; cnt++ )          //����ͷ���У���룬��check sum USB out data byte2 to EP1_2_PACKET_SIZE - 1
  {
    rf_tx_buf[cnt-2] = usb_out_buf[cnt];        
    sum += rf_tx_buf[cnt-2];
  }      
  sum = sum ^ 0xFF;
  rf_tx_buf[NRF_DATA_LEN - 1] = sum;

  app_usb_out_data_ready = true;                            // USB send data is right flag
}

//-----------------------------------------------------------------------------
//����2.4���ݵ�USB  ��ص�ѹAD ����AD��
//-----------------------------------------------------------------------------
static void nrf_usb_in_packet(void)
{
  uint8_t sum,cnt;

//************** ckeck sum ***************//
  sum = 0;
  for(cnt = 0;cnt < NRF_DATA_LEN - 1;cnt++)             // calculate nrf rx buffer check sum
  {
    sum += rf_rx_buf[cnt];
  }
  sum = sum ^ 0xFF;
  if(!(sum == rf_rx_buf[NRF_DATA_LEN - 1]))             //  ckeck sum check
  {
    return;
  }    
//**************************************//

  if(*(rf_rx_buf) == 0xAC)                              // NRF RXͷ���Ƿ�Ϊ0xAC,�񣬲������˳�            
  {
    usb_in_buf[0] = 0xFA;
    usb_in_buf[1] = 0xFB;
    usb_in_buf[2] = 0xAC;

//********************* DATA Handle *******************//
    usb_in_buf[3]   = rf_rx_buf[1];                     //Bat_slave_high_val  ,slave battery AD VOL high byte value
    usb_in_buf[4]   = rf_rx_buf[2];                     //Bat_slave_low_val  ,slave battery AD VOL low byte value
    usb_in_buf[5]   = rf_rx_buf[3];                     //Bat_slave_high_cur  ,slave battery AD VOL high byte value
    usb_in_buf[6]   = rf_rx_buf[4];                     //Bat_slave_low_cur  ,slave battery AD VOL low byte value
    for(cnt=7; cnt < EP1_2_PACKET_SIZE; cnt++ )         // claer unused bytes
    {
      usb_in_buf[cnt] = 0;
    }

//*********************CHECK SUM*********************//
    sum = 0;
    for(cnt=0; cnt<EP1_2_PACKET_SIZE - 1; cnt++)        // calculate data check sum byte0 to EP1_2_PACKET_SIZE - 1
    {
      sum += usb_in_buf[cnt];
    }
    sum = sum ^ 0xFF;
    usb_in_buf[EP1_2_PACKET_SIZE - 1] = sum;            // EP1_2_PACKET_SIZE - 1 byte is check sum byte
  }
}


//-----------------------------------------------------------------------------
// RF set TX mode
//-----------------------------------------------------------------------------
void nrf_tx_mode(void)
{                               
    CE_LOW();
  
    hal_nrf_write_multibyte_reg(W_REGISTER + TX_ADDR,(uint8_t*)TX_RX_ADDRESS,5);        //дTX�ڵ��ַ 
    hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0,(uint8_t*)TX_RX_ADDRESS,5);     //����TX�ڵ��ַ,��ҪΪ��ʹ��ACK    
    hal_nrf_write_reg(EN_AA,0x01);                                                      //ʹ��ͨ��0���Զ�Ӧ��  
    hal_nrf_write_reg(EN_RXADDR,0x01);                                                  //ʹ��ͨ��0�Ľ��յ�ַ  
    hal_nrf_write_reg(SETUP_RETR,0x1a);                                                 //�����Զ��ط����ʱ��:500us + 86us;����Զ��ط�����:10��
    hal_nrf_write_reg(RF_CH,RF_CHANNEL);                                                //����RFͨ��Ϊ40
    hal_nrf_write_reg(RF_SETUP,0x0f);                                                   //����TX�������,0db����,2Mbps,���������濪��   
    hal_nrf_write_reg(CONFIG,0x0e);                                                     //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ,���������ж�
    
    CE_HIGH();
}


//-----------------------------------------------------------------------------
// RF set RX mode
//-----------------------------------------------------------------------------
void nrf_rx_mode(void)
{  
    CE_LOW();

    hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0,(uint8_t*)TX_RX_ADDRESS,5);     //дRX�ڵ��ַ  
    hal_nrf_write_reg(EN_AA,0x01);                                                      //ʹ��ͨ��0���Զ�Ӧ��  
    hal_nrf_write_reg(EN_RXADDR,0x01);                                                  //ʹ��ͨ��0�Ľ��յ�ַ     
    hal_nrf_write_reg(RF_CH,RF_CHANNEL);                                                //����RFͨ��Ƶ��      
    hal_nrf_write_reg(RX_PW_P0,RX_PAYLOAD_LEN);                                         //ѡ��ͨ��0����Ч���ݿ��     
    hal_nrf_write_reg(RF_SETUP,0x0f);                                                   //����TX�������,0db����,2Mbps,���������濪��   
    hal_nrf_write_reg(CONFIG, 0x0f);                                                    //���û�������ģʽ�Ĳ���;PWR_UP,EN_CRC,16BIT_CRC,����ģʽ 
    
    CE_HIGH();
}

//-----------------------------------------------------------------------------
// RF helper functions
//-----------------------------------------------------------------------------

// Initialize radio module
static void rf_config(void)
{
    // Enable radio SPI and clock
    RFCTL = 0x10;                           // �ڲ�SPI��ʽ����
    RFCKEN = 1;                             // RF CLK enable
   
    if(!NRF_Check())                        // check RF communicate is OK
    {
        LED = 0;
    }
    nrf_tx_mode();                          // TX mode set
  
    RF = 1;                                 // ��NRF�ж�
    EA = 1;                                 // ��NRF�ж�
}

//-----------------------------------------------------------------------------
// RF funtion check
// return 0:sucess
// return 1:not success
//-----------------------------------------------------------------------------
bool NRF_Check(void)
{
  uint8_t buf[5]={0XA5,0XA5,0XA5,0XA5,0XA5};
  uint8_t i;    
  hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0+HAL_NRF_TX,buf,5);        //д��5���ֽڵĵ�ַ.  
  hal_nrf_read_multibyte_reg(HAL_NRF_TX,buf);                                   //����д��ĵ�ַ  
  for(i=0;i<5;i++)if(buf[i]!=0XA5)break;                    
  if(i!=5)return 1;                                                             //���24L01����  
  return 0;                                                                     //��⵽24L01
}


// Interrupt handler for RF module
/*******************************************************************************************************
 * ��  �� : RF�жϷ�����
 * ��  �� : none
 * ����ֵ : none
 *******************************************************************************************************/
NRF_ISR()
{
  uint8_t irq_flags;

  irq_flags = hal_nrf_read_reg(STATUS);        // get RF IRQ flag
   
  if(irq_flags & RX_OK)                        // RX data occured
  {
    hal_nrf_read_multibyte_reg(HAL_NRF_RX_PLOAD,rf_rx_buf);    // RF RX data
    hal_nrf_flush_rx();                        // clear RX FIFO
    nrf_usb_in_packet();                       // RF RX data to USB in data
    nrf_rx_packet_received = true;             //NRF RX data handle is done flag
//****************************debug********************************//
    //LED = ~LED;
//*****************************************************************//
  }
      
  else if((irq_flags & MAX_TX) > 0)                 //TX data times was the max  occured
  {
    hal_nrf_flush_tx();                             // clear RF TX FIFO
    nrf_rx_mode();                                  //Change ro RX mode              
  }
      
  else if((irq_flags & TX_OK) > 0)                  //TX data is done(ACK feedback) occured
  {
    hal_nrf_flush_tx();                             //Clear RF TX FIFO buffer 
    nrf_rx_mode();                                  //Change ro RX mode              
  }
      
  hal_nrf_write_reg(STATUS,irq_flags);              //Clear NRF IRQ flag  
}


//-----------------------------------------------------------------------------
// USB Helper functions
//-----------------------------------------------------------------------------  

/*******************************************************************************************************
 * ��  �� : USB��������������
 * ��  �� : buf�����ͻ����׵�ַ
 *      size���������ݳ���
 * ����ֵ : none
 *******************************************************************************************************/  
static void usb_send_to_pc(uint8_t * buf, uint8_t size)
{
    app_wait_while_usb_pending();
    app_pending_usb_write = true;  
    //memcpy(usb_in_buf, buf, size);
    hal_usb_send_data(1, buf, size);
}


static void app_wait_while_usb_pending()
{  
    uint16_t timeout = 50000;                     // Will equal ~ 50-100 ms timeout 
    while(timeout--)
    {
      if(!app_pending_usb_write)
      {
          break;
      }
    }  
}

//-----------------------------------------------------------------------------
// USB Callbacks
//-----------------------------------------------------------------------------  

hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req* req, uint8_t** data_ptr, uint8_t* size) large reentrant
{
  hal_usb_dev_req_resp_t retval;

  if( hal_usb_hid_device_req_proc(req, data_ptr, size, &retval) == true ) 
  {
  // The request was processed with the result stored in the retval variable
  return retval;
  }
  else
  {
  // The request was *not* processed by the HID subsystem
  return STALL;   
  }
}

void suspend_cb(uint8_t allow_remote_wu) large reentrant
{
  USBSLP = 1; // Disable USB clock (auto clear)
  allow_remote_wu = 0;  
}

void resume_cb(void) large reentrant
{
}

void reset_cb(void) large reentrant
{
}

//-----------------------------------------------------------------------------
// USB Endpoint Callbacks
//-----------------------------------------------------------------------------  
uint8_t ep_1_in_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant    //MCU to PC
{  
    app_pending_usb_write = false;
    return 0x60;                           // NAK
    adr_ptr = adr_ptr;
    size = size;
}

uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant  //PC to MCU �жϵ���
{
    memcpy(usb_out_buf, adr_ptr, *size);                // get USB out data to buffer
//*********************debug********************************//
    //LED =~LED;
//**********************************************************//
    nrf_usb_out_packet();                               // USB ���յ����ݣ�У�飬���NRF TX����֡

    return 0xff;                                        // ACK
}

/********************************************END FILE*****************************************************/
