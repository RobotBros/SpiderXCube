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
 * $LastChangedRevision: 230 $
 */

/** @file
 * @brief
 * Implementation of Gazell Pairing Library (gzp), Host functions.
*/

#include "gzp.h"
#include "gzll.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"
#include "hal_aes.h"
#include "hal_flash.h"
#include "memdefs.h"

//lint -esym(40, GZP_PARAMS_STORAGE_ADR) "Undeclared identifier"

//-----------------------------------------------------------------------------
// Typedefs
//-----------------------------------------------------------------------------

/**
  Definition of internal states.
*/
typedef enum
{
  GZP_ID_REQ_IDLE,
  GZP_ID_REQ_PENDING,
  GZP_ID_REQ_PENDING_AND_GRANTED,
  GZP_ID_REQ_PENDING_AND_REJECTED,
} gzp_id_req_stat_t;

//-----------------------------------------------------------------------------
// Internal (static) functions
//-----------------------------------------------------------------------------

/**
  Function for incrementing internal session counter.
*/
static void gzp_session_counter_inc();

/**
  Function for reading value of internal session counter.
*/
static void gzp_get_session_counter(uint8_t* dst);

/**
  Function for reading "Host ID" from non volatile (NV) memory.

  Returns false if "Host ID" has not been set and no ID has been
  returned to *dst.
*/
static bool gzp_set_host_id(const uint8_t* dst);

/**
  Function processing received "system address request" from Device.
*/
static void gzp_process_address_req(uint8_t* gzp_req);

/**
  Functions processing various command packets received from a Device.
*/
static void gzp_process_id_req(uint8_t* rx_payload);
static void gzp_process_id_fetch(uint8_t* rx_payload);
static void gzp_process_key_update_prepare();
static void gzp_process_key_update(uint8_t* rx_payload);
static void gzp_process_encrypted_user_data(uint8_t* rx_payload, uint8_t length);

/**
  Functions processing various command packets received from a Device.
*/
static void gzp_preload_ack(uint8_t* src, uint8_t length, uint8_t pipe);

/**
  Function for reading unique chip ID.
*/
void gzp_host_chip_id_read(uint8_t *dst, uint8_t n);

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

static gzp_id_req_stat_t gzp_id_req_stat;
static xdata bool gzp_pairing_enabled_f;
static xdata bool gzp_address_exchanged_f;
static xdata uint8_t gzp_session_counter[GZP_SESSION_TOKEN_LENGTH];
static xdata uint8_t gzp_encrypted_user_data_length;
static xdata bool gzp_encrypted_user_data[GZP_ENCRYPTED_USER_DATA_MAX_LENGTH];
extern __no_init uint8_t gzp_dyn_key[GZP_DYN_KEY_LENGTH];

//-----------------------------------------------------------------------------
// Implementation: Application programming interface (API)
//-----------------------------------------------------------------------------

void gzp_init()
{
  uint8_t system_address[GZP_SYSTEM_ADDRESS_WIDTH];

  // Read "chip id", of which 4 bytes (GZP_SYSTEM_ADDRESS_WIDTH)
  // are used as system address
  gzp_host_chip_id_read(system_address, GZP_SYSTEM_ADDRESS_WIDTH);

  // Set up radio parameters (addresses and channel subset) from system_address
  gzp_update_radio_params(system_address);

  // Only "data pipe" enabled by default
  gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << GZP_DATA_PIPE));

  gzp_pairing_enabled_f = false;
  gzp_address_exchanged_f = false;
  gzp_id_req_stat = GZP_ID_REQ_IDLE;
  gzp_encrypted_user_data_length = 0;

  // Infinite RX timeout
  gzll_set_param(GZLL_PARAM_RX_TIMEOUT, 0);
}

void gzp_pairing_enable(bool enable)
{
  gzll_states_t temp_gzll_state;

  temp_gzll_state = gzll_get_state();

  if(gzp_pairing_enabled_f != enable)
  {
    gzll_goto_idle();

    if(enable)
    {
      gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << GZP_PAIRING_PIPE));
    }
    else
    {
      gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) & ~(1 << GZP_PAIRING_PIPE));
      gzp_id_req_stat = GZP_ID_REQ_IDLE;
    }

    gzp_pairing_enabled_f = enable;

    if(temp_gzll_state == GZLL_HOST_ACTIVE)
    {
      gzll_rx_start();
    }
  }
}

void gzp_host_execute()
{
  uint8_t rx_pipe;
  uint8_t payload_length;
  uint8_t rx_payload[GZLL_MAX_FW_PAYLOAD_LENGTH];

  gzp_address_exchanged_f = false;

  rx_pipe = gzll_get_rx_data_ready_pipe_number();

  if((rx_pipe == GZP_PAIRING_PIPE) || ((rx_pipe == GZP_DATA_PIPE) && (gzp_encrypted_user_data_length == 0)))
  {
    gzll_rx_fifo_read(rx_payload, &payload_length, NULL);

    switch(rx_payload[0])
    {
      case GZP_CMD_HOST_ADDRESS_REQ:
        gzp_process_address_req(rx_payload);
        break;

      #ifndef GZP_CRYPT_DISABLE

      case GZP_CMD_HOST_ID_REQ:
        gzp_process_id_req(rx_payload);
        break;
      case GZP_CMD_HOST_ID_FETCH:
        gzp_process_id_fetch(rx_payload);
        break;
      case GZP_CMD_KEY_UPDATE_PREPARE:
        gzp_process_key_update_prepare();
        break;
      case GZP_CMD_KEY_UPDATE:
        gzp_process_key_update(rx_payload);
        break;
      case GZP_CMD_ENCRYPTED_USER_DATA:
        gzp_process_encrypted_user_data(rx_payload, payload_length);
        break;

      #endif

      case GZP_CMD_FETCH_RESP:
      default:
        break;
    }
  }

  // Restart reception if "not proximity backoff" period has elapsed
  if(gzll_get_state() == GZLL_IDLE)
  {
    gzll_set_param(GZLL_PARAM_RX_TIMEOUT, 0);
    if(gzp_pairing_enabled_f)
    {
      gzll_set_param(GZLL_PARAM_RX_PIPES, gzll_get_param(GZLL_PARAM_RX_PIPES) | (1 << GZP_PAIRING_PIPE));
    }
    gzll_rx_start();
  }

  #ifndef GZP_CRYPT_DISABLE
  gzp_session_counter_inc();
  #endif
}

static void gzp_process_address_req(uint8_t* gzp_req)
{
  uint8_t temp_rx_pipes, temp_host_mode;
  uint8_t pairing_resp[GZP_CMD_HOST_ADDRESS_RESP_PAYLOAD_LENGTH];

  gzp_address_exchanged_f = false;

  gzll_goto_idle();
  temp_rx_pipes = gzll_get_param(GZLL_PARAM_RX_PIPES);
  temp_host_mode =  gzll_get_param(GZLL_PARAM_HOST_MODE);

  // If requesting Device within close proximity
  if(gzll_rx_power_high())
  {
    gzll_set_param(GZLL_PARAM_RX_PIPES, 0);
    gzll_set_param(GZLL_PARAM_HOST_MODE, 0);
    gzll_set_param(GZLL_PARAM_RX_TIMEOUT, GZP_CLOSE_PROXIMITY_BACKOFF_RX_TIMEOUT);
    gzll_rx_fifo_flush();

    // Start "proximity" back off period
    gzll_rx_start();

    while(gzll_get_state() != GZLL_IDLE)
    ;

    // Build pairing response packet
    pairing_resp[0] = GZP_CMD_HOST_ADDRESS_RESP;
    gzp_host_chip_id_read(&pairing_resp[GZP_CMD_HOST_ADDRESS_RESP_ADDRESS], GZP_SYSTEM_ADDRESS_WIDTH);
    gzll_ack_payload_write(&pairing_resp[0], GZP_CMD_HOST_ADDRESS_RESP_PAYLOAD_LENGTH, 0);
    gzll_set_param(GZLL_PARAM_RX_TIMEOUT, GZP_STEP1_RX_TIMEOUT);

    // Enable only pairing pipe when waiting for pairing request step 1
    gzll_set_param(GZLL_PARAM_RX_PIPES, (1 << GZP_PAIRING_PIPE));
    gzll_rx_start();

    while(gzll_get_state() != GZLL_IDLE)
    {
      if(gzll_rx_fifo_read(&gzp_req[0], NULL, NULL))
      {
        // Validate step 1 of pairing request
        if(gzp_req[0] == GZP_CMD_HOST_ADDRESS_FETCH)
        {
          gzp_address_exchanged_f = true;
        }
      }
    }

    gzll_tx_fifo_flush();
    gzll_rx_fifo_flush();
    gzll_set_param(GZLL_PARAM_RX_TIMEOUT, 0);
    gzll_set_param(GZLL_PARAM_RX_PIPES, temp_rx_pipes);
    gzll_set_param(GZLL_PARAM_HOST_MODE, temp_host_mode);

    // Return to normal operation
    gzll_rx_start();
  }
  else
  {
    gzll_set_param(GZLL_PARAM_RX_PIPES, temp_rx_pipes & ~(1 << GZP_PAIRING_PIPE));
    gzll_set_param(GZLL_PARAM_RX_TIMEOUT, GZP_NOT_PROXIMITY_BACKOFF_RX_TIMEOUT);
    // Start "not proximity" backoff period
    gzll_rx_start();
  }
}

static void gzp_preload_ack(uint8_t* src, uint8_t length, uint8_t pipe)
{
  gzll_goto_idle();
  gzll_tx_fifo_flush();
  gzll_ack_payload_write(src, length, pipe);
  gzll_rx_start();
}

bool gzp_address_exchanged()
{
  return gzp_address_exchanged_f;
}

#ifndef GZP_CRYPT_DISABLE

bool gzp_crypt_user_data_received()
{
  return (gzp_encrypted_user_data_length > 0);
}

bool gzp_crypt_user_data_read(uint8_t* dst, uint8_t* length)
{
  if(gzp_encrypted_user_data_length > 0)
  {
    memcpy(dst, (void*)gzp_encrypted_user_data, gzp_encrypted_user_data_length);

    if(length != NULL)
    {
      *length = gzp_encrypted_user_data_length;
    }
    gzp_encrypted_user_data_length = 0;

    return true;
  }
  else
  {
    return false;
  }
}

bool gzp_id_req_received()
{
  return (gzp_id_req_stat != GZP_ID_REQ_IDLE);
}

void gzp_id_req_reject()
{
  if(gzp_id_req_received())
  {
    gzp_id_req_stat = GZP_ID_REQ_PENDING_AND_REJECTED;
  }
}

void gzp_id_req_grant()
{
  if(gzp_id_req_received())
  {
    gzp_id_req_stat = GZP_ID_REQ_PENDING_AND_GRANTED;
  }
}

void gzp_id_req_cancel()
{
  if(gzp_id_req_received())
  {
    gzp_id_req_stat = GZP_ID_REQ_IDLE;
  }
}

static void gzp_session_counter_inc()
{
  uint8_t i;

  for(i = 0; i < GZP_SESSION_TOKEN_LENGTH; i++)
  {
    gzp_session_counter[i]++;
    if(gzp_session_counter[i] != 0)
    {
      break;
    }
  }
}

static void gzp_get_session_counter(uint8_t* dst)
{
  memcpy(dst, (void*)gzp_session_counter, GZP_SESSION_TOKEN_LENGTH);
}

static bool gzp_set_host_id(const uint8_t* src)
{
  if(hal_flash_byte_read(GZP_PARAMS_STORAGE_ADR) == 0xff)
  {
    hal_flash_byte_write(GZP_PARAMS_STORAGE_ADR, 0x00);
    hal_flash_bytes_write(GZP_PARAMS_STORAGE_ADR + 1, src, GZP_HOST_ID_LENGTH);
    return true;
  }
  else
  {
    return false;
  }
}

bool gzp_get_host_id(uint8_t* dst)
{
  if(hal_flash_byte_read(GZP_PARAMS_STORAGE_ADR) == 0)
  {
    hal_flash_bytes_read(GZP_PARAMS_STORAGE_ADR + 1, dst, GZP_HOST_ID_LENGTH);
    return true;
  }
  else
  {
    return false;
  }
}

static void gzp_process_id_req(uint8_t* rx_payload)
{
  uint8_t temp_host_id[GZP_HOST_ID_LENGTH];

  if(gzp_pairing_enabled_f)
  {
    if(!gzp_id_req_received())
    {
      gzp_crypt_set_session_token(&rx_payload[GZP_CMD_HOST_ID_REQ_SESSION_TOKEN]);
      gzp_id_req_stat = GZP_ID_REQ_PENDING;
    }

    // If host ID not generated yet
    if(!gzp_get_host_id(temp_host_id))
    {
      // Generate new host ID from "session counter" and received "session token"
      gzp_get_session_counter(temp_host_id);
      if(GZP_HOST_ID_LENGTH > GZP_SESSION_TOKEN_LENGTH)
      {
        gzp_xor_cipher(temp_host_id, temp_host_id, &rx_payload[GZP_CMD_HOST_ID_REQ_SESSION_TOKEN], GZP_SESSION_TOKEN_LENGTH);
      }
      else
      {
        gzp_xor_cipher(temp_host_id, temp_host_id, &rx_payload[GZP_CMD_HOST_ID_REQ_SESSION_TOKEN], GZP_HOST_ID_LENGTH);
      }

      gzp_set_host_id(temp_host_id);
    }
  }
}

static void gzp_process_id_fetch(uint8_t* rx_payload)
{
  uint8_t tx_payload[GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH];

  if(gzp_id_req_received())
  {
    gzp_crypt_select_key(GZP_ID_EXCHANGE);
    gzp_crypt(&rx_payload[1], &rx_payload[1], GZP_CMD_HOST_ID_FETCH_PAYLOAD_LENGTH - 1);
    if(gzp_validate_id(&rx_payload[GZP_CMD_HOST_ID_FETCH_VALIDATION_ID]))
    {
      switch(gzp_id_req_stat)
      {
        case GZP_ID_REQ_PENDING_AND_GRANTED:
          tx_payload[GZP_CMD_HOST_ID_FETCH_RESP_STATUS] = GZP_ID_RESP_GRANTED;
          gzp_get_host_id(&tx_payload[GZP_CMD_HOST_ID_FETCH_RESP_HOST_ID]);
          gzp_id_req_stat = GZP_ID_REQ_IDLE;
          break;
        case GZP_ID_REQ_PENDING_AND_REJECTED:
          tx_payload[GZP_CMD_HOST_ID_FETCH_RESP_STATUS] = GZP_ID_RESP_REJECTED;
          gzp_id_req_stat = GZP_ID_REQ_IDLE;
          break;
        case GZP_ID_REQ_PENDING:
        default:
          tx_payload[GZP_CMD_HOST_ID_FETCH_RESP_STATUS] = GZP_ID_RESP_PENDING;
          break;
      }

      tx_payload[0] = GZP_CMD_HOST_ID_FETCH_RESP;
      gzp_add_validation_id(&tx_payload[GZP_CMD_HOST_ID_FETCH_RESP_VALIDATION_ID]);
      gzp_crypt(&tx_payload[1], &tx_payload[1], GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH - 1);
      gzp_preload_ack(tx_payload, GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH, GZP_DATA_PIPE);
    }
  }
}

static void gzp_process_key_update_prepare()
{
  uint8_t tx_payload[GZP_CMD_KEY_UPDATE_PREPARE_RESP_PAYLOAD_LENGTH];

  tx_payload[0] = GZP_CMD_KEY_UPDATE_PREPARE_RESP;

  gzp_get_session_counter(&tx_payload[GZP_CMD_KEY_UPDATE_PREPARE_RESP_SESSION_TOKEN]);

  // Update session token if no ID request is pending
  if(!gzp_id_req_received())
  {
    gzp_crypt_set_session_token(&tx_payload[GZP_CMD_KEY_UPDATE_PREPARE_RESP_SESSION_TOKEN]);
  }

  gzp_preload_ack(tx_payload, GZP_CMD_KEY_UPDATE_PREPARE_RESP_PAYLOAD_LENGTH, GZP_DATA_PIPE);
}

static void gzp_process_key_update(uint8_t* rx_payload)
{
  gzp_crypt_select_key(GZP_KEY_EXCHANGE);
  gzp_crypt(&rx_payload[1], &rx_payload[1], GZP_CMD_KEY_UPDATE_PAYLOAD_LENGTH - 1);
  if(gzp_validate_id(&rx_payload[GZP_CMD_KEY_UPDATE_VALIDATION_ID]))
  {
    gzp_crypt_set_dyn_key(&rx_payload[GZP_CMD_KEY_UPDATE_NEW_KEY]);
  }
}

static void gzp_process_encrypted_user_data(uint8_t* rx_payload, uint8_t length)
{
  uint8_t tx_payload[GZP_CMD_ENCRYPTED_USER_DATA_RESP_PAYLOAD_LENGTH];

  if(gzp_id_req_received())
  {
    gzp_crypt_select_key(GZP_ID_EXCHANGE);
  }
  else
  {
    gzp_crypt_select_key(GZP_DATA_EXCHANGE);
  }

  gzp_crypt(&rx_payload[1], &rx_payload[1], length - 1);
  if(gzp_validate_id(&rx_payload[GZP_CMD_ENCRYPTED_USER_DATA_VALIDATION_ID]))
  {
    gzp_encrypted_user_data_length = length - GZP_USER_DATA_PACKET_OVERHEAD;
    memcpy((void*)gzp_encrypted_user_data, &rx_payload[GZP_CMD_ENCRYPTED_USER_DATA_PAYLOAD], gzp_encrypted_user_data_length);
  }

  // Build response packet
  tx_payload[0] = GZP_CMD_ENCRYPTED_USER_DATA_RESP;
  gzp_add_validation_id(&tx_payload[GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID]);
  gzp_crypt(&tx_payload[GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID], &tx_payload[GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID], GZP_VALIDATION_ID_LENGTH);
  gzp_get_session_counter(&tx_payload[GZP_CMD_ENCRYPTED_USER_DATA_RESP_SESSION_TOKEN]);

  // Update "session token" only if no ID request is pending
  if(!gzp_id_req_received())
  {
    gzp_crypt_set_session_token(&tx_payload[GZP_CMD_ENCRYPTED_USER_DATA_RESP_SESSION_TOKEN]);
  }

  gzp_preload_ack(tx_payload, GZP_CMD_ENCRYPTED_USER_DATA_RESP_PAYLOAD_LENGTH, GZP_DATA_PIPE);
}

#endif
