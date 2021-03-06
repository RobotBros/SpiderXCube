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
** Descriptions:    USB HID 无线收发数据实验程序�
**          通过USB HID配置nRF24LU1P的无线参数:无线信道和接收数据长度
**          接收发射端的无线信息，通过USB HID上传
**          通过USB HID将信息发送给nRF24LU1P
**          
**--------------------------------------------------------------------------------------------------------
** Modified by:      EnixYu
** Modified date:    2016-09-24
** Version:          1.1
** Descriptions:    增加3路舵机角度扩展，修改发送包校验和计算。
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
** USB相关变量定义
-----------------------------------------------------------------------------*/
static xdata uint8_t usb_in_buf[EP1_2_PACKET_SIZE];
static xdata uint8_t usb_out_buf[EP1_2_PACKET_SIZE];
static bool xdata app_usb_out_data_ready = false;                  // 判断USB send 完成标志
static bool xdata system_ctrl_state_flag = false;                    // 判断是否PC控制模式                  
extern code const usb_string_desc_templ_t g_usb_string_desc;
static bool xdata app_pending_usb_write = false;

/*-----------------------------------------------------------------------------
** RF相关变量定义
-----------------------------------------------------------------------------*/
#define NRF_DATA_LEN  32
static uint8_t xdata rf_rx_buf[NRF_DATA_LEN];
static uint8_t xdata rf_tx_buf[NRF_DATA_LEN];
static bool xdata nrf_rx_packet_received = false;                      //NRF RX是否完成标志位
static bool xdata radio_busy = false;
static bool xdata transmitted = false;

const uint8_t TX_RX_ADDRESS[5] = {0x34,0x43,0x10,0x10,0x01};             // TX/RX地址 5字节


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
** USB回调函数声明
-----------------------------------------------------------------------------*/
hal_usb_dev_req_resp_t device_req_cb(hal_usb_device_req* req, uint8_t** data_ptr, uint8_t* size) large reentrant;
void suspend_cb(uint8_t allow_remote_wu) large reentrant;
void resume_cb() large reentrant;
void reset_cb() large reentrant;
uint8_t ep_1_in_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;
uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant;

/*******************************************************************************************************
 * 描  述 : MAIN函数
 * 入  参 : none
 * 返回值 : none
 *******************************************************************************************************/
void main()
{
    P0DIR = 0xEF;                                                   // 配置P0:P04配置为输出
    LED = 1;                                                        // 熄灭指示灯

//*************************USB HAL initialization************************//
    hal_usb_init(true, device_req_cb, reset_cb, resume_cb, suspend_cb);   
    hal_usb_endpoint_config(0x81, EP1_2_PACKET_SIZE, ep_1_in_cb);      // Configure 32 bytes IN endpoint 1 
    hal_usb_endpoint_config(0x02, EP1_2_PACKET_SIZE, ep_2_out_cb);     // Configure 32 bytes OUT endpoint 2

    rf_config();                                                    // RF initial

    while(true)                                                     // 初始化未NRF RX模式，一直在NRF RX模式下工作，遇到USB 接收到舵机控制指令后，触发一次NRF TX                              
    {  
        if(hal_usb_get_state() == CONFIGURED)                       // check USB initial
        { 
            if(app_usb_out_data_ready == true)                      // USB 接收到主机发送的数据正确标志
            {          
                app_usb_out_data_ready = false;                     // Claer USB 接收成功标志
            }
        }

        if(nrf_rx_packet_received == true)                          // RF接收数据完成 RX缓存
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
//  NRF send  data 兼容遥控板主动发送摇杆数据程序，未解锁，spider会自动屏蔽此数据，系统状态判断，选择发送数据包
//-----------------------------------------------------------------------------
static void nrf_tx_pack_send(void)
{
    uint8_t cnt,sum;

    if(system_ctrl_state_flag == false)                                     // 判断是否 PC控制命名模式，否则发送固定摇杆数据，是发送舵机控制数据
    {
        rf_tx_buf[0] = 0xAB;
        rf_tx_buf[1] = 0x32;                                                // 左右摇杆X/Y轴50%数据
        rf_tx_buf[2] = 0x32;
        rf_tx_buf[3] = 0x32;
        rf_tx_buf[4] = 0x32;
//************** nop data ***************//
        for(cnt=5; cnt<NRF_DATA_LEN; cnt ++)                                // 未定义数据清零处理     
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

    nrf_tx_mode();                                                          // 设置工作模式：TX  
    hal_nrf_flush_tx();                                                     // RF TX FIFO clear
    delay_ms(1);                     
    CE_LOW();
    hal_nrf_write_multibyte_reg(W_TX_PAYLOAD,rf_tx_buf,RX_PAYLOAD_LEN);    //写数据到TX BUF  32个字节
    CE_HIGH();
}

//-----------------------------------------------------------------------------
// Handle commands from Host application 接收USB数据后2.4G发送  舵机角度控制数据
//-----------------------------------------------------------------------------
static void nrf_usb_out_packet(void)
{
  uint8_t sum,cnt;

//************** ckeck 头码 ****************************//
  if((!(usb_out_buf[0] == 0xFA))  || (! (usb_out_buf[1] == 0xFB)))  // USB 发送数据头码 0xFA 0xFB
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
  if(usb_out_buf[2] == 0xAA)                                // 判断是否PC控制命名模式，命令字为0xAA
  {
    system_ctrl_state_flag = true;
  }          
  else
  {
    system_ctrl_state_flag = false;
  }
//*****************RF TX pack************************//    // USB out data to nrf tx buffer
  sum = 0;
  for(cnt=2; cnt < EP1_2_PACKET_SIZE + 1; cnt++ )          //屏蔽头码和校验码，求check sum USB out data byte2 to EP1_2_PACKET_SIZE - 1
  {
    rf_tx_buf[cnt-2] = usb_out_buf[cnt];        
    sum += rf_tx_buf[cnt-2];
  }      
  sum = sum ^ 0xFF;
  rf_tx_buf[NRF_DATA_LEN - 1] = sum;

  app_usb_out_data_ready = true;                            // USB send data is right flag
}

//-----------------------------------------------------------------------------
//发送2.4数据到USB  电池电压AD 电流AD等
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

  if(*(rf_rx_buf) == 0xAC)                              // NRF RX头码是否为0xAC,否，不处理退出            
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
  
    hal_nrf_write_multibyte_reg(W_REGISTER + TX_ADDR,(uint8_t*)TX_RX_ADDRESS,5);        //写TX节点地址 
    hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0,(uint8_t*)TX_RX_ADDRESS,5);     //设置TX节点地址,主要为了使能ACK    
    hal_nrf_write_reg(EN_AA,0x01);                                                      //使能通道0的自动应答  
    hal_nrf_write_reg(EN_RXADDR,0x01);                                                  //使能通道0的接收地址  
    hal_nrf_write_reg(SETUP_RETR,0x1a);                                                 //设置自动重发间隔时间:500us + 86us;最大自动重发次数:10次
    hal_nrf_write_reg(RF_CH,RF_CHANNEL);                                                //设置RF通道为40
    hal_nrf_write_reg(RF_SETUP,0x0f);                                                   //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
    hal_nrf_write_reg(CONFIG,0x0e);                                                     //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式,开启所有中断
    
    CE_HIGH();
}


//-----------------------------------------------------------------------------
// RF set RX mode
//-----------------------------------------------------------------------------
void nrf_rx_mode(void)
{  
    CE_LOW();

    hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0,(uint8_t*)TX_RX_ADDRESS,5);     //写RX节点地址  
    hal_nrf_write_reg(EN_AA,0x01);                                                      //使能通道0的自动应答  
    hal_nrf_write_reg(EN_RXADDR,0x01);                                                  //使能通道0的接收地址     
    hal_nrf_write_reg(RF_CH,RF_CHANNEL);                                                //设置RF通信频率      
    hal_nrf_write_reg(RX_PW_P0,RX_PAYLOAD_LEN);                                         //选择通道0的有效数据宽度     
    hal_nrf_write_reg(RF_SETUP,0x0f);                                                   //设置TX发射参数,0db增益,2Mbps,低噪声增益开启   
    hal_nrf_write_reg(CONFIG, 0x0f);                                                    //配置基本工作模式的参数;PWR_UP,EN_CRC,16BIT_CRC,接收模式 
    
    CE_HIGH();
}

//-----------------------------------------------------------------------------
// RF helper functions
//-----------------------------------------------------------------------------

// Initialize radio module
static void rf_config(void)
{
    // Enable radio SPI and clock
    RFCTL = 0x10;                           // 内部SPI方式配置
    RFCKEN = 1;                             // RF CLK enable
   
    if(!NRF_Check())                        // check RF communicate is OK
    {
        LED = 0;
    }
    nrf_tx_mode();                          // TX mode set
  
    RF = 1;                                 // 开NRF中断
    EA = 1;                                 // 开NRF中断
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
  hal_nrf_write_multibyte_reg(W_REGISTER + RX_ADDR_P0+HAL_NRF_TX,buf,5);        //写入5个字节的地址.  
  hal_nrf_read_multibyte_reg(HAL_NRF_TX,buf);                                   //读出写入的地址  
  for(i=0;i<5;i++)if(buf[i]!=0XA5)break;                    
  if(i!=5)return 1;                                                             //检测24L01错误  
  return 0;                                                                     //检测到24L01
}


// Interrupt handler for RF module
/*******************************************************************************************************
 * 描  述 : RF中断服务函数
 * 入  参 : none
 * 返回值 : none
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
 * 描  述 : USB向主机发送数据
 * 入  参 : buf：发送缓存首地址
 *      size：发送数据长度
 * 返回值 : none
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

uint8_t ep_2_out_cb(uint8_t *adr_ptr, uint8_t* size) large reentrant  //PC to MCU 中断调用
{
    memcpy(usb_out_buf, adr_ptr, *size);                // get USB out data to buffer
//*********************debug********************************//
    //LED =~LED;
//**********************************************************//
    nrf_usb_out_packet();                               // USB 接收到数据，校验，组成NRF TX缓存帧

    return 0xff;                                        // ACK
}

/********************************************END FILE*****************************************************/
