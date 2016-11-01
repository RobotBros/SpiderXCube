/* Copyright (c) 2009 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is confidential property of Nordic 
 * Semiconductor ASA.Terms and conditions of usage are described in detail 
 * in NORDIC SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *              
 * $LastChangedRevision: 5717 $
 */

/** @file
@brief Implementation of the rf_test  module
*/

#include "hal_nrf.h"
#include "hal_nrf_hw.h"


#include "pn9.h"

#include "rf_test.h"
#include <stdbool.h>
#include <intrins.h>

#define DEFAULT_SIZE_PACKET 32
#define NB_BITS_FOR_ERROR_RATE_CALC 100000
#define NB_BYTES_FOR_ERROR_RATE_CALC 12500
#define MAX_SIZE_PACKET 32
#define ADDR_WIDTH 5


#define RF_ENABLE() (RFCKEN = 1);
#define RF_IRQ_ENABLE() (RF = 1);
#define RF_IRQ_DISABLE() (RF = 0);


typedef struct rf_test_ctx_t 
{
  uint16_t nb_computed_bytes;
  uint32_t bit_errors;
  uint8_t actual_packet_size;
  uint8_t payload[MAX_SIZE_PACKET];
  uint8_t sweep_first_channel;
  uint8_t sweep_last_channel;
  uint8_t sweep_current_channel;
  uint8_t mod_tx_addr[ADDR_WIDTH];
  uint16_t bit_error_rate;
} rf_test_ctx_t;


static volatile rf_test_ctx_t rf_test_ctx;

void rf_test_init()
{
  uint8_t index;
  
  rf_test_ctx.nb_computed_bytes = 0;
  rf_test_ctx.actual_packet_size = 0xFF;
  rf_test_ctx.bit_errors = 0;
  rf_test_ctx.bit_error_rate = 0xFFFF;
  rf_test_ctx.sweep_first_channel = 0;
  rf_test_ctx.sweep_last_channel = 80;
  rf_test_ctx.sweep_current_channel = 0;
  
  for(index = 0; index < ADDR_WIDTH; index ++)
  {
    rf_test_ctx.mod_tx_addr[index] = 0;
  }
  for(index = 0; index < MAX_SIZE_PACKET; index ++)
  {
    rf_test_ctx.payload[index] = 0;
  }

}

void rf_test_sensitivity_set_expected_data(uint8_t data_size, uint8_t *p_expected_data)
{
  uint8_t byte_index;
  
  CE_LOW();
  if (NULL == p_expected_data)
  {
    hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, rf_test_ctx.actual_packet_size);
  }
  else
  {
    rf_test_ctx.actual_packet_size = data_size;
    hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, rf_test_ctx.actual_packet_size); // pipe0 expect 32 byte payload
    for(byte_index = 0; byte_index < data_size; byte_index++)
    {
      rf_test_ctx.payload[byte_index] = p_expected_data[byte_index];
    }
  }
  CE_HIGH();
}


rf_test_ret_t rf_test_sensitivity_init(uint8_t ch, hal_nrf_output_power_t pwr, hal_nrf_datarate_t datarate, uint8_t *address)
{
  uint8_t default_payload[MAX_SIZE_PACKET] = {0xaa, 0x1b, 0x2c, 0x10, 0x81, 0x01, 0x11, 0x1a,
                                        0xfe, 0xee, 0xcd, 0x12, 0x13, 0xa9, 0x92, 0x13,
                                        0x14, 0x17, 0x7d, 0x12, 0xdf, 0xfd, 0xaa, 0xaf,
                                        0x27, 0x65, 0x41, 0x43, 0x3a, 0x2d, 0xc1, 0x55};

  rf_test_ctx.bit_errors = 0;
  rf_test_ctx.bit_error_rate = 0xFFFF;


  CE_LOW();
  hal_nrf_set_output_power(pwr);
  hal_nrf_set_datarate(datarate);
  hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);
  hal_nrf_flush_rx();
  hal_nrf_set_operation_mode(HAL_NRF_PRX); 
  hal_nrf_close_pipe(HAL_NRF_ALL);                // first close all radio pipes...
  hal_nrf_open_pipe(HAL_NRF_PIPE0, false); // Turn off all auto acknowledge functionality
  hal_nrf_set_auto_retr(HAL_NRF_PIPE0,0);
  hal_nrf_set_rf_channel(ch);
  hal_nrf_set_pll_mode(false);
  hal_nrf_set_crc_mode(HAL_NRF_CRC_OFF);
  hal_nrf_set_address(HAL_NRF_PIPE0, address);

  if (0xFF == rf_test_ctx.actual_packet_size)
  {
    //the size and expected packet has not been set
    rf_test_sensitivity_set_expected_data(DEFAULT_SIZE_PACKET, default_payload); //set the default values
  }
  else
  {
    rf_test_sensitivity_set_expected_data(0, NULL); //otherwise reset the previous values
  }
  CE_HIGH();
  return(RF_TEST_RC_OK);
}


static void rf_test_compute_error_rate_for_one_packet(uint8_t packet_size, uint8_t *in_packet)
{
  uint8_t byte_index;
  uint8_t xored;
  uint8_t nb_err_bits_array[] = {0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4};

  for(byte_index=0; byte_index<packet_size;byte_index++)
  {
    if (in_packet[byte_index] != rf_test_ctx.payload[byte_index])  //check against expected value stored in payload
    {
      xored = in_packet[byte_index] ^ rf_test_ctx.payload[byte_index];
      rf_test_ctx.bit_errors += nb_err_bits_array[(xored&0x0F)];
      rf_test_ctx.bit_errors += nb_err_bits_array[(xored&0xF0)>>4];
    }
  }
  rf_test_ctx.nb_computed_bytes += packet_size;
}

rf_test_ret_t rf_test_receive_and_compute_packet()
{
  volatile uint16_t nb_received_byte;
  uint8_t pload[MAX_SIZE_PACKET]; // recived from RX
  rf_test_ret_t ret_code = RF_TEST_RC_OK;

  nb_received_byte = hal_nrf_read_rx_payload(pload);  // read received data
  nb_received_byte &= 0x00FF;  	  //the received number of bytes is on the LSB
  if(rf_test_ctx.actual_packet_size != nb_received_byte)
  {
    ret_code = RF_TEST_RC_ERR_NB_RX_BYTES;
  }
  rf_test_compute_error_rate_for_one_packet(nb_received_byte, pload);
  if (rf_test_ctx.nb_computed_bytes >= NB_BYTES_FOR_ERROR_RATE_CALC)
  {
    ret_code = RF_TEST_RC_FINISHED;
  }
  return(ret_code);

}


uint16_t rf_test_compute_error_rate()
{
  uint32_t local_nb_computed_bits;
  uint16_t local_nb_error;
  local_nb_error = (uint16_t)rf_test_ctx.bit_errors;
  if (0xFFFF == rf_test_ctx.bit_error_rate)
  {
    local_nb_computed_bits = (uint32_t)rf_test_ctx.nb_computed_bytes;
    local_nb_computed_bits = local_nb_computed_bits << 3;             //multiply by 8 to get the number of bits.
    if (rf_test_ctx.nb_computed_bytes >= NB_BYTES_FOR_ERROR_RATE_CALC)
    {
      if (local_nb_computed_bits == rf_test_ctx.bit_errors)
      {
        rf_test_ctx.bit_error_rate = 9999;
      }
      else
      {
        //calculate bit error rate
        //multiplied by 10000 to fit in two bytes, this value must be divided by 100 to get BER in percent.
        rf_test_ctx.bit_error_rate =(uint16_t)((rf_test_ctx.bit_errors*10000)/local_nb_computed_bits);
      }
      rf_test_ctx.nb_computed_bytes = 0;
      rf_test_ctx.bit_errors  = 0;
    }
  }
  return(rf_test_ctx.bit_error_rate);
}


rf_test_ret_t rf_test_init_rx_sweep(uint8_t pwr, uint8_t datarate, uint8_t *rx_mode_address, uint8_t first_channel, uint8_t last_channel)
{
  rf_test_ctx.sweep_first_channel = first_channel;
  rf_test_ctx.sweep_last_channel = last_channel;
  rf_test_ctx.sweep_current_channel = rf_test_ctx.sweep_first_channel;

  CE_LOW();
  hal_nrf_set_output_power(pwr);
  hal_nrf_set_datarate(datarate);
  hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);
  hal_nrf_flush_rx();
  hal_nrf_set_operation_mode(HAL_NRF_PRX); 
  hal_nrf_close_pipe(HAL_NRF_ALL);                // first close all radio pipes...
  hal_nrf_open_pipe(HAL_NRF_PIPE0, false);        // Turn off all auto acknowledge functionality  
  hal_nrf_set_auto_retr(HAL_NRF_PIPE0,0);
  hal_nrf_set_pll_mode(false);
  hal_nrf_set_crc_mode(HAL_NRF_CRC_OFF);
  hal_nrf_set_address(HAL_NRF_PIPE0, rx_mode_address);
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DEFAULT_SIZE_PACKET); // pipe0 expect 32 byte payload

  CE_HIGH();
  return(RF_TEST_RC_OK);
}

rf_test_ret_t rf_test_init_tx_sweep(uint8_t pwr, uint8_t datarate, uint8_t first_channel, uint8_t last_channel)
{
  rf_test_ctx.sweep_first_channel = first_channel;
  rf_test_ctx.sweep_last_channel = last_channel;
  rf_test_ctx.sweep_current_channel = rf_test_ctx.sweep_first_channel;

  CE_LOW();
  hal_nrf_set_operation_mode(HAL_NRF_PTX);
  hal_nrf_set_pll_mode(true);
  hal_nrf_set_output_power(pwr);
  hal_nrf_set_datarate(datarate);
  hal_nrf_set_crc_mode(HAL_NRF_CRC_OFF);
  hal_nrf_enable_continious_wave(true);
  CE_HIGH();
  return(RF_TEST_RC_OK);
}


void rf_test_txrx_sweep()
{
  CE_LOW();
  hal_nrf_set_rf_channel(rf_test_ctx.sweep_current_channel);
  CE_HIGH();

  rf_test_ctx.sweep_current_channel++; 
  if (rf_test_ctx.sweep_current_channel > rf_test_ctx.sweep_last_channel)
  {
    rf_test_ctx.sweep_current_channel = rf_test_ctx.sweep_first_channel;
  }
}


void rf_test_set_modulation_payload()
{
  uint8_t i;

  pn9_init();
  
  for (i = 0; i < ADDR_WIDTH; i++)
  {
    rf_test_ctx.mod_tx_addr[i] = pn9_get_byte();
  }
  
  for (i = 0; i < DEFAULT_SIZE_PACKET; i++)
  {
    rf_test_ctx.payload[i] = pn9_get_byte();
  }
}

rf_test_ret_t rf_test_start_mod_carrier(uint8_t channel, uint8_t pwr, uint8_t datarate)
{
  if (channel > RF_TEST_MAX_CHANNEL)
  {
    return(RF_TEST_RC_ERR_PARAM);
  }
  if (pwr > HAL_NRF_0DBM){
    return(RF_TEST_RC_ERR_PARAM);
  }
  if (datarate  > HAL_NRF_250KBPS)
  {
    return(RF_TEST_RC_ERR_PARAM);
  }

  CE_LOW();

  hal_nrf_set_output_power(pwr);
  hal_nrf_set_datarate(datarate);
  hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);
  hal_nrf_set_operation_mode(HAL_NRF_PTX); 
  hal_nrf_open_pipe(HAL_NRF_PIPE0, false); // Turn off all auto acknowledge functionality  
  hal_nrf_set_auto_retr(0,0);
  hal_nrf_set_rf_channel(channel);
  hal_nrf_set_pll_mode(true);

  hal_nrf_set_address(HAL_NRF_TX, rf_test_ctx.mod_tx_addr);
  hal_nrf_write_tx_payload(rf_test_ctx.payload, DEFAULT_SIZE_PACKET);
  hal_nrf_set_crc_mode(HAL_NRF_CRC_OFF);

  CE_PULSE();
  return(RF_TEST_RC_OK);
}


void rf_test_const_carrier_isr(void)
{
  static uint8_t pk_cnt_ = 0;
  
  CE_HIGH();
  hal_nrf_reuse_tx();
  
  hal_nrf_get_clear_irq_flags();
  
  if(pk_cnt_ > 19)
  {
    CE_LOW();
    pk_cnt_ = 0;
    CE_PULSE();  // Refresh
  }
  else
  {
    pk_cnt_++;
  }
}

rf_test_ret_t rf_test_start_unmod_carrier(uint8_t channel, uint8_t pwr, uint8_t datarate)
{
  CE_LOW();
  
  hal_nrf_set_operation_mode(HAL_NRF_PTX);
  hal_nrf_set_rf_channel(channel);
  hal_nrf_set_pll_mode(true);
  hal_nrf_set_output_power(pwr);
  hal_nrf_set_datarate(datarate);
  hal_nrf_set_crc_mode(HAL_NRF_CRC_OFF);
  hal_nrf_enable_continious_wave(true);
  CE_HIGH();
  return(RF_TEST_RC_OK);
}

rf_test_ret_t rf_test_start_rx_carrier(uint8_t channel, uint8_t pwr, uint8_t datarate, uint8_t *rx_carrier_address)
{
  if (channel > RF_TEST_MAX_CHANNEL)
  {
    return(RF_TEST_RC_ERR_PARAM);
  }
  if (pwr > HAL_NRF_0DBM){
    return(RF_TEST_RC_ERR_PARAM);
  }
  if (datarate  > HAL_NRF_250KBPS)
  {
    return(RF_TEST_RC_ERR_PARAM);
  }
  if (NULL == rx_carrier_address)
  {
    return(RF_TEST_RC_ERR_PARAM);
  }
  CE_LOW();
  hal_nrf_set_output_power(pwr);
  hal_nrf_set_datarate(datarate);
  hal_nrf_set_address_width(HAL_NRF_AW_5BYTES);
  
  hal_nrf_flush_rx();
  hal_nrf_set_operation_mode(HAL_NRF_PRX); 
  hal_nrf_close_pipe(HAL_NRF_ALL);                // first close all radio pipes...
  hal_nrf_open_pipe(HAL_NRF_PIPE0, false);        // Turn off all auto acknowledge functionality  
  hal_nrf_set_auto_retr(0,0);
  hal_nrf_set_rf_channel(channel);
  hal_nrf_set_pll_mode(false);
  
  hal_nrf_set_crc_mode(HAL_NRF_CRC_OFF);
  
  hal_nrf_set_address(HAL_NRF_PIPE0, rx_carrier_address);
  hal_nrf_set_rx_payload_width(HAL_NRF_PIPE0, DEFAULT_SIZE_PACKET); // pipe0 expect 32 byte payload
  
  CE_HIGH();
  return(RF_TEST_RC_OK);
}

