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
** Descriptions:    USB HID ÎÞÏßÊÕ·¢Êý¾ÝÊµÑé³ÌÐò£
**          Í¨¹ýUSB HIDÅäÖÃnRF24LU1PµÄÎÞÏß²ÎÊý:ÎÞÏßÐÅµÀºÍ½ÓÊÕÊý¾Ý³¤¶È
**          ½ÓÊÕ·¢Éä¶ËµÄÎÞÏßÐÅÏ¢£¬Í¨¹ýUSB HIDÉÏ´«
**          Í¨¹ýUSB HID½«ÐÅÏ¢·¢ËÍ¸ønRF24LU1P
**          
**--------------------------------------------------------------------------------------------------------
** Modified by:      EnixYu
** Modified date:    2016-09-24
** Version:          1.1
** Descriptions:    Ôö¼Ó3Â·¶æ»ú½Ç¶ÈÀ©Õ¹£¬ÐÞ¸Ä·¢ËÍ°üÐ£ÑéºÍ¼ÆËã¡£
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
** USBÏà¹Ø±äÁ¿¶¨Òå
-----------------------------------------------------------------------------*/
static xdata uint8_t usb_in_buf[EP1_2_PACKET_SIZE];
static xdata uint8_t usb_out_buf[EP1_2_PACKET_SIZE];
static bool xdata app_usb_out_data_ready = false;                  // ÅÐ¶ÏUSB send Íê³É±êÖ¾
static bool xdata system_ctrl_state_flag = false;                    // ÅÐ¶ÏÊÇ·ñPC¿ØÖÆÄ£Ê½                  
extern code const usb_string_desc_templ_t g_usb_string_desc;
static bool xdata app_pending_usb_write = false;

/*-----------------------------------------------------------------------------
** RFÏà¹Ø±äÁ¿¶¨Òå
-----------------------------------------------------------------------------*/
#define NRF_DATA_LEN  32
static uint8_t xdata rf_rx_buf[NRF_DATA_LEN];
static uint8_t xdata rf_tx_buf[NRF_DATA_LEN];
static bool xdata nrf_rx_packet_received = false;                      //NRF RXÊÇ·ñÍê³É±êÖ¾Î»
static bool xdata radio_busy = false;
static bool xdata transmitted = false;

const uint8_t TX_RX_ADDRESS[5] = {0x34,0x43,0x10,0x10,0x01};             // TX/RXµØÖ· 5×Ö½Ú


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
** USB»Øµ÷º¯ÊýÉùÃ÷
-----------------------------------------------------------------------------*/
hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req* req, uint8_t** data_ptr, uint8_t* size) large reentrant;
void suspend_cb(uint8_t allow_remote_wu) large reentrant;
void resume_cb() large reentrant;
void reset_cb() large reentrant;
uint8_t ep_1_in_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;
uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;

/*******************************************************************************************************
 * Ãè  Êö : MAINº¯Êý
 * Èë  ²Î : none
 * ·µ»ØÖµ : none
 *******************************************************************************************************/
void main()
{
    P0DIR = 0xEF;                                                   // ÅäÖÃP0:P04ÅäÖÃÎªÊä³ö
    LED = 1;                                                        // Ï¨ÃðÖ¸Ê¾µÆ

//*************************USB HAL initialization************************//
    hal_usb_init(true, device_req_cb, reset_cb, resume_cb, suspend_cb);   
    hal_usb_endpoint_config(0x81, EP1_2_PACKET_SIZE, ep_1_in_cb);      // Configure 32 bytes IN endpoint 1 
    hal_usb_endpoint_config(0x02, EP1_2_PACKET_SIZE, ep_2_out_cb);     // Configure 32 bytes OUT endpoint 2

    rf_config();                                                    // RF initial

    while(true)                                                     // ³õÊ¼»¯Î´NRF RXÄ£Ê½£¬Ò»Ö±ÔÚNRF RXÄ£Ê½ÏÂ¹¤×÷£¬Óöµ½USB ½ÓÊÕµ½¶æ»ú¿ØÖÆÖ¸Áîºó£¬´¥·¢Ò»´ÎNRF TX                              
    {  
        if(hal_usb_get_state() == CONFIGURED)                       // check USB initial
        { 
            if(app_usb_out_data_ready == true)                      // USB ½ÓÊÕµ½Ö÷»ú·¢ËÍµÄÊý¾ÝÕýÈ·±êÖ¾
            {          
                app_usb_out_data_ready = false;                     // Claer USB ½ÓÊÕ³É¹¦±êÖ¾
            }
        }

        if(nrf_rx_packet_received == true)                          // RF½ÓÊÕÊý¾ÝÍê³É RX»º´æ
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
//  NRF send  data ¼æÈÝÒ£¿Ø°åÖ÷¶¯·¢ËÍÒ¡¸ËÊý¾Ý³ÌÐò£¬Î´½âËø£¬spider»á×Ô¶¯ÆÁ±Î´ËÊý¾Ý£¬ÏµÍ³×´Ì¬ÅÐ¶Ï£¬Ñ¡Ôñ·¢ËÍÊý¾Ý°ü
//-----------------------------------------------------------------------------
static void nrf_tx_pack_send(void)
{
    uint8_t cnt,sum;

    if(system_ctrl_state_flag == false)                                     // ÅÐ¶ÏÊÇ·ñ PC¿ØÖÆÃüÃûÄ£Ê½£¬·ñÔò·¢ËÍ¹Ì¶¨Ò¡¸ËÊý¾Ý£¬ÊÇ·¢ËÍ¶æ»ú¿ØÖÆÊý¾Ý
    {
        rf_tx_buf[0] = 0xAB;
        rf_tx_buf[1] = 0x32;                                                // ×óÓÒÒ¡¸ËX/YÖá50%Êý¾Ý
        rf_tx_buf[2] = 0x32;
        rf_tx_buf[3] = 0x32;
        rf_tx_buf[4] = 0x32;
//************** nop data ***************//
        for(cnt=5; cnt<NRF_DATA_LEN; cnt ++)                                // Î´¶¨ÒåÊý¾ÝÇåÁã´¦Àí     
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

    nrf_tx_mode();                                                          // ÉèÖÃ¹¤×÷Ä£Ê½£ºTX  
    hal_nrf_flush_tx();                                                     // RF TX FIFO clear
    delay_ms(1);                     
    CE_LOW();
    hal_nrf_write_multibyte_reg(W_TX_PAYLOAD,rf_tx_buf,RX_PAYLOAD_LEN);    //Ð´Êý¾Ýµ½TX BUF  32¸ö×Ö½Ú
    CE_HIGH();
}

//-----------------------------------------------------------------------------
// Handle commands from Host application ½ÓÊÕUSBÊý¾Ýºó2.4G·¢ËÍ  ¶æ»ú½Ç¶È¿ØÖÆÊý¾Ý
//-----------------------------------------------------------------------------
static void nrf_usb_out_packet(void)
{
  uint8_t sum,cnt;

//************** ckeck Í·Âë ****************************//
  if((!(usb_out_buf[0] == 0xFA))  || (! (usb_out_buf[1] == 0xFB)))  // USB ·¢ËÍÊý¾ÝÍ·Âë 0xFA 0xFB
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
  if(usb_out_buf[2] == 0xAA)                                // ÅÐ¶ÏÊÇ·ñPC¿ØÖÆÃüÃûÄ£Ê½£¬ÃüÁî×ÖÎª0xAA
  {
    system_ctrl_state_flag = true;
  }          
  else
  {
    system_ctrl_state_flag = false;
  }
//*****************RF TX pack************************//    // USB out data to nrf tx buffer
  sum = 0;
  for(cnt=2; cnt < EP1_2_PACKET_SIZE + 1; cnt++ )          //ÆÁ±ÎÍ·ÂëºÍÐ£ÑéÂë£¬Çócheck sum USB out data byte2 to EP1_2_PACKET_SIZE - 1
  {
    rf_tx_buf[cnt-2] = usb_out_buf[cnt];        
    sum += rf_tx_buf[cnt-2];
  }      
  sum = sum ^ 0xFF;
  rf_tx_buf[NRF_DATA_LEN - 1] = sum;

  app_usb_out_data_ready = true;                            // USB send data is right flag
}

//-----------------------------------------------------------------------------
//·¢ËÍ2.4Êý¾Ýµ½USB  µç³ØµçÑ¹AD µçÁ÷ADµÈ
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

  if(*(rf_rx_buf) == 0xAC)                              // NRF RXÍ·ÂëÊÇ·ñÎª0xAC,·ñ£¬²»´¦ÀíÍË³ö            
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
  
    hal_nrf_write_multibyte_reg(W_REGISTER + TX_ADDR,(uint8_t*)TX_RX_ADDRESS,5);        //Ð´TX½ÚµãµØÖ· 
    hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0,(uint8_t*)TX_RX_ADDRESS,5);     //ÉèÖÃTX½ÚµãµØÖ·,Ö÷ÒªÎªÁËÊ¹ÄÜACK    
    hal_nrf_write_reg(EN_AA,0x01);                                                      //Ê¹ÄÜÍ¨µÀ0µÄ×Ô¶¯Ó¦´ð  
    hal_nrf_write_reg(EN_RXADDR,0x01);                                                  //Ê¹ÄÜÍ¨µÀ0µÄ½ÓÊÕµØÖ·  
    hal_nrf_write_reg(SETUP_RETR,0x1a);                                                 //ÉèÖÃ×Ô¶¯ÖØ·¢¼ä¸ôÊ±¼ä:500us + 86us;×î´ó×Ô¶¯ÖØ·¢´ÎÊý:10´Î
    hal_nrf_write_reg(RF_CH,RF_CHANNEL);                                                //ÉèÖÃRFÍ¨µÀÎª40
    hal_nrf_write_reg(RF_SETUP,0x0f);                                                   //ÉèÖÃTX·¢Éä²ÎÊý,0dbÔöÒæ,2Mbps,µÍÔëÉùÔöÒæ¿ªÆô   
    hal_nrf_write_reg(CONFIG,0x0e);                                                     //ÅäÖÃ»ù±¾¹¤×÷Ä£Ê½µÄ²ÎÊý;PWR_UP,EN_CRC,16BIT_CRC,½ÓÊÕÄ£Ê½,¿ªÆôËùÓÐÖÐ¶Ï
    
    CE_HIGH();
}


//-----------------------------------------------------------------------------
// RF set RX mode
//-----------------------------------------------------------------------------
void nrf_rx_mode(void)
{  
    CE_LOW();

    hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0,(uint8_t*)TX_RX_ADDRESS,5);     //Ð´RX½ÚµãµØÖ·  
    hal_nrf_write_reg(EN_AA,0x01);                                                      //Ê¹ÄÜÍ¨µÀ0µÄ×Ô¶¯Ó¦´ð  
    hal_nrf_write_reg(EN_RXADDR,0x01);                                                  //Ê¹ÄÜÍ¨µÀ0µÄ½ÓÊÕµØÖ·     
    hal_nrf_write_reg(RF_CH,RF_CHANNEL);                                                //ÉèÖÃRFÍ¨ÐÅÆµÂÊ      
    hal_nrf_write_reg(RX_PW_P0,RX_PAYLOAD_LEN);                                         //Ñ¡ÔñÍ¨µÀ0µÄÓÐÐ§Êý¾Ý¿í¶È     
    hal_nrf_write_reg(RF_SETUP,0x0f);                                                   //ÉèÖÃTX·¢Éä²ÎÊý,0dbÔöÒæ,2Mbps,µÍÔëÉùÔöÒæ¿ªÆô   
    hal_nrf_write_reg(CONFIG, 0x0f);                                                    //ÅäÖÃ»ù±¾¹¤×÷Ä£Ê½µÄ²ÎÊý;PWR_UP,EN_CRC,16BIT_CRC,½ÓÊÕÄ£Ê½ 
    
    CE_HIGH();
}

//-----------------------------------------------------------------------------
// RF helper functions
//-----------------------------------------------------------------------------

// Initialize radio module
static void rf_config(void)
{
    // Enable radio SPI and clock
    RFCTL = 0x10;                           // ÄÚ²¿SPI·½Ê½ÅäÖÃ
    RFCKEN = 1;                             // RF CLK enable
   
    if(!NRF_Check())                        // check RF communicate is OK
    {
        LED = 0;
    }
    nrf_tx_mode();                          // TX mode set
  
    RF = 1;                                 // ¿ªNRFÖÐ¶Ï
    EA = 1;                                 // ¿ªNRFÖÐ¶Ï
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
  hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0+HAL_NRF_TX,buf,5);        //Ð´Èë5¸ö×Ö½ÚµÄµØÖ·.  
  hal_nrf_read_multibyte_reg(HAL_NRF_TX,buf);                                   //¶Á³öÐ´ÈëµÄµØÖ·  
  for(i=0;i<5;i++)if(buf[i]!=0XA5)break;                    
  if(i!=5)return 1;                                                             //¼ì²â24L01´íÎó  
  return 0;                                                                     //¼ì²âµ½24L01
}


// Interrupt handler for RF module
/*******************************************************************************************************
 * Ãè  Êö : RFÖÐ¶Ï·þÎñº¯Êý
 * Èë  ²Î : none
 * ·µ»ØÖµ : none
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
 * Ãè  Êö : USBÏòÖ÷»ú·¢ËÍÊý¾Ý
 * Èë  ²Î : buf£º·¢ËÍ»º´æÊ×µØÖ·
 *      size£º·¢ËÍÊý¾Ý³¤¶È
 * ·µ»ØÖµ : none
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

uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant  //PC to MCU ÖÐ¶Ïµ÷ÓÃ
{
    memcpy(usb_out_buf, adr_ptr, *size);                // get USB out data to buffer
//*********************debug********************************//
    //LED =~LED;
//**********************************************************//
    nrf_usb_out_packet();                               // USB ½ÓÊÕµ½Êý¾Ý£¬Ð£Ñé£¬×é³ÉNRF TX»º´æÖ¡

    return 0xff;                                        // ACK
}

/********************************************END FILE*****************************************************/
