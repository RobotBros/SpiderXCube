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
 * $LastChangedRevision: 226 $
 */

/** @file
 * @brief
 * Gazell pairing library header file
 */
 
#ifndef __GZP_H
#define __GZP_H

#include "gzll.h"
#include "gzp_params.h"
#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------------------
// Misc. defines
//-----------------------------------------------------------------------------
#define GZP_PAIRING_PIPE 0
#define GZP_DATA_PIPE 1
#define GZP_TX_RX_TRANS_DELAY 10
#define GZP_SYSTEM_ADDRESS_WIDTH 4     // Must equal Gazell address width - 1
#define GZP_VALIDATION_ID {0x32, 0x53, 0x66}
#define GZP_VALIDATION_ID_LENGTH 3
#define GZP_HOST_ID_LENGTH 5
#define GZP_SESSION_TOKEN_LENGTH GZP_HOST_ID_LENGTH
#define GZP_DYN_KEY_LENGTH (16 - GZP_VALIDATION_ID_LENGTH)

//-----------------------------------------------------------------------------
// Device -> Host packet definitions
//-----------------------------------------------------------------------------

// "Host address request" packet
#define GZP_CMD_HOST_ADDRESS_REQ_PAYLOAD_LENGTH 1

// "Host address fetch" packet
#define GZP_CMD_HOST_ADDRESS_FETCH_PAYLOAD_LENGTH 1

// "Host ID request" packet
#define GZP_CMD_HOST_ID_REQ_SESSION_TOKEN 1
#define GZP_CMD_HOST_ID_REQ_PAYLOAD_LENGTH (GZP_CMD_HOST_ID_REQ_SESSION_TOKEN + GZP_SESSION_TOKEN_LENGTH) 

#if (GZP_CMD_HOST_ID_REQ_PAYLOAD_LENGTH > 17)
#error GZP_SESSION_TOKEN_LENGTH too long. 
#endif

// "Host ID fetch" packet
#define GZP_CMD_HOST_ID_FETCH_VALIDATION_ID 1
#define GZP_CMD_HOST_ID_FETCH_PAYLOAD_LENGTH (GZP_CMD_HOST_ID_FETCH_VALIDATION_ID + GZP_VALIDATION_ID_LENGTH)

#if (GZP_CMD_HOST_ID_FETCH_PAYLOAD_LENGTH > 17)
#error GZP_VALIDATION_ID_LENGTH set too long. 
#endif
 
// "Key update prepare" packet
#define GZP_CMD_KEY_UPDATE_PREPARE_PAYLOAD_LENGTH 1

// "Key update" packet
#define GZP_CMD_KEY_UPDATE_VALIDATION_ID 1
#define GZP_CMD_KEY_UPDATE_NEW_KEY (GZP_CMD_KEY_UPDATE_VALIDATION_ID + GZP_VALIDATION_ID_LENGTH)
#define GZP_CMD_KEY_UPDATE_PAYLOAD_LENGTH (GZP_CMD_KEY_UPDATE_NEW_KEY + GZP_DYN_KEY_LENGTH)

#if (GZP_CMD_KEY_UPDATE_PAYLOAD_LENGTH > 17)
#error Sum (GZP_VALIDATION_ID_LENGTH + GZP_DYN_KEY_LENGTH) too high. 
#endif

// "Encrypted user data" packet
#define GZP_CMD_ENCRYPTED_USER_DATA_VALIDATION_ID 1
#define GZP_CMD_ENCRYPTED_USER_DATA_PAYLOAD ((GZP_CMD_ENCRYPTED_USER_DATA_VALIDATION_ID + GZP_VALIDATION_ID_LENGTH))
#define GZP_USER_DATA_PACKET_OVERHEAD ( GZP_CMD_ENCRYPTED_USER_DATA_VALIDATION_ID + GZP_VALIDATION_ID_LENGTH)  

#define GZP_ENCRYPTED_USER_DATA_MAX_LENGTH (17 - GZP_USER_DATA_PACKET_OVERHEAD)

#if(GZLL_MAX_FW_PAYLOAD_LENGTH < 17)
  #error GZLL_MAX_FW_PAYLOAD_LENGTH must be greater or equal to 17.
#endif

// General "fetch response" packet
#define GZPAR_CMD_FETCH_RESP_PAYLOAD_LENGTH 1

//-----------------------------------------------------------------------------
// Host -> Device packet definitions
//-----------------------------------------------------------------------------

// "Host address fetch" response packet
#define GZP_CMD_HOST_ADDRESS_RESP_ADDRESS 1
#define GZP_CMD_HOST_ADDRESS_RESP_PAYLOAD_LENGTH (GZP_CMD_HOST_ADDRESS_RESP_ADDRESS + GZP_SYSTEM_ADDRESS_WIDTH) 

#if(GZLL_MAX_ACK_PAYLOAD_LENGTH < GZP_CMD_HOST_ADDRESS_RESP_PAYLOAD_LENGTH)
  #error GZLL_MAX_ACK_PAYLOAD_LENGTH must be greater or equal to GZP_CMD_HOST_ADDRESS_RESP_PAYLOAD_LENGTH.
#endif

// "Host ID fetch" response packet
#define GZP_CMD_HOST_ID_FETCH_RESP_VALIDATION_ID 1
#define GZP_CMD_HOST_ID_FETCH_RESP_STATUS (GZP_CMD_HOST_ID_FETCH_RESP_VALIDATION_ID + GZP_VALIDATION_ID_LENGTH)
#define GZP_CMD_HOST_ID_FETCH_RESP_HOST_ID (GZP_CMD_HOST_ID_FETCH_RESP_STATUS + 1)
#define GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH (GZP_CMD_HOST_ID_FETCH_RESP_HOST_ID + GZP_HOST_ID_LENGTH)

#if(GZLL_MAX_ACK_PAYLOAD_LENGTH < GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH)
  #error GZLL_MAX_ACK_PAYLOAD_LENGTH must be greater or equal to GZP_CMD_HOST_ID_FETCH_RESP_PAYLOAD_LENGTH.
#endif

// "Key update prepare" response packet
#define GZP_CMD_KEY_UPDATE_PREPARE_RESP_SESSION_TOKEN 1
#define GZP_CMD_KEY_UPDATE_PREPARE_RESP_PAYLOAD_LENGTH (GZP_CMD_KEY_UPDATE_PREPARE_RESP_SESSION_TOKEN + GZP_SESSION_TOKEN_LENGTH)

#if(GZLL_MAX_ACK_PAYLOAD_LENGTH < GZP_CMD_KEY_UPDATE_PREPARE_RESP_PAYLOAD_LENGTH)
  #error GZLL_MAX_ACK_PAYLOAD_LENGTH must be greater or equal to GZP_CMD_KEY_UPDATE_PREPARE_RESP_PAYLOAD_LENGTH.
#endif
 
// "Encrypted user data" response packet
#define GZP_CMD_ENCRYPTED_USER_DATA_RESP_SESSION_TOKEN 1
#define GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID (GZP_CMD_ENCRYPTED_USER_DATA_RESP_SESSION_TOKEN + GZP_SESSION_TOKEN_LENGTH)
#define GZP_CMD_ENCRYPTED_USER_DATA_RESP_PAYLOAD_LENGTH (GZP_CMD_ENCRYPTED_USER_DATA_RESP_VALIDATION_ID + GZP_VALIDATION_ID_LENGTH)

#if(GZLL_MAX_ACK_PAYLOAD_LENGTH < GZP_CMD_ENCRYPTED_USER_DATA_RESP_PAYLOAD_LENGTH)
  #error GZLL_MAX_ACK_PAYLOAD_LENGTH must be greater or equal to GZP_CMD_ENCRYPTED_USER_DATA_RESP_PAYLOAD_LENGTH.
#endif

#if(GZP_VALIDATION_ID_LENGTH > GZP_HOST_ID_LENGTH)
  #error GZP_HOST_ID_LENGTH should be greater or equal to GZP_VALIDATION_ID_LENGTH.
#endif

#if(GZP_SESSION_TOKEN_LENGTH != GZP_HOST_ID_LENGTH)
  #error GZP_SESSION_TOKEN_LENGTH must equal GZP_HOST_ID_LENGTH.
#endif

#ifdef GZLL_CRYPT_ENABLE
  #error Gazell encryption can not be enabled when using the Gazell pairing library. \
  GZLL_CRYPT_ENABLE must be undefined.
#endif

//-----------------------------------------------------------------------------
// Typedefs
//-----------------------------------------------------------------------------
typedef enum
{
  GZP_ID_EXCHANGE,    // "Secret key" only
  GZP_KEY_EXCHANGE,   // "Secret key" and "Host ID"
  GZP_DATA_EXCHANGE   // "Dynamic key" and "Host ID"
} gzp_key_select_t;

typedef enum
{
  GZP_CMD_HOST_ADDRESS_REQ = 0,
  GZP_CMD_HOST_ADDRESS_FETCH,
  GZP_CMD_HOST_ID_REQ,
  GZP_CMD_HOST_ID_FETCH,
  GZP_CMD_KEY_UPDATE_PREPARE,
  GZP_CMD_KEY_UPDATE,      
  GZP_CMD_ENCRYPTED_USER_DATA,
  GZP_CMD_FETCH_RESP,
  GZP_CMD_HOST_ADDRESS_RESP,
  GZP_CMD_HOST_ID_FETCH_RESP,
  GZP_CMD_KEY_UPDATE_PREPARE_RESP,
  GZP_CMD_ENCRYPTED_USER_DATA_RESP,
} gzp_cmd_t;

typedef enum
{
  GZP_ID_RESP_PENDING,
  GZP_ID_RESP_GRANTED,
  GZP_ID_RESP_REJECTED,
  GZP_ID_RESP_FAILED
} gzp_id_req_res_t;

//-----------------------------------------------------------------------------
// Misc. function prototypes
//-----------------------------------------------------------------------------

/**
  Function for writing global variable gzp_session_token.
*/
void gzp_crypt_set_session_token(const uint8_t *token);

/**
  Function for writing global variable gzp_dyn_key.
*/
void gzp_crypt_set_dyn_key(const uint8_t* src_key);

/**
  Function for reading global variable gzp_session_token.
*/
void gzp_crypt_get_session_token(uint8_t *dst_token);

/**
  Function for reading global variable gzp_dyn_key.
*/
void gzp_crypt_get_dyn_key(uint8_t *dst_key);

/**
  Function for wriring variable holding "Host ID".
  Returns @b false if host ID could not be written.
*/
static bool gzp_set_host_id(const uint8_t* src);

/**
  Function reading current "Host ID".
  On Host, returns @b false if host ID not yet generated.
*/
bool gzp_get_host_id(uint8_t *dst);

/**
  Function selecting what key-set that should be used when encrypting data
  using gzp_crypt().
*/
void gzp_crypt_select_key(gzp_key_select_t key_select);

/**
  Function for encypting / decrypting data. 
  
  The current "session token" will be used as initialization vector (IV).
  The AES key to be used is selected by gzp_crypt_select_key().
*/
void gzp_crypt(uint8_t* dst, const uint8_t* src, uint8_t length);

/**
  Function for comparing *src_id with a pre-defined validation ID.
  
  Returns true if *src_id equals the pre-defined ID. 
*/
bool gzp_validate_id(const uint8_t *src_id);

/**
  Function for adding the pre-defined validation ID to dst_id.
  GZP_VALIDATION_ID_LENGTH bytes will be added.
*/
void gzp_add_validation_id(uint8_t *dst_id);

/**
  Function generating and returning n random numbers to dst.
*/
void gzp_random_numbers_generate(uint8_t *dst, uint8_t n);

/**
  Function updating channel subset and all pipe addresses from a single 
  5 byte address. 
*/
void gzp_update_radio_params(const uint8_t *system_address);

/**
  Function generating channel subset from a single 
  5 byte address. 
*/
void gzp_generate_channels(uint8_t *ch_dst, const uint8_t *system_address, uint8_t channel_tab_size);

/**
  Function byte-wise xor'ing pad with src and returns result in dst. 
*/
void gzp_xor_cipher(uint8_t* dst, const uint8_t* src, const uint8_t* pad, uint8_t length);

/** @defgroup nordic_gzp_api Application Programming Interface (API) for the Gazell pairing library
    @ingroup nordic_gzp
@{
*/

/**
@name Common Device and Host functions 
*/

/**
  Initialization function. This function initializes the Gazell Pairing Library.
  
  This function must be called before any of the other Gazell Pairing Library functions are 
  used and must be called @b after gzll_init() is called. 
*/
void gzp_init(void);

/**
  Function for cancelling an ongoing (pending) "Host ID request".

  After calling this function the "Host ID request" status will go to 
  "ID request Idle".
*/
void gzp_id_req_cancel(void);

/**
@name Device functions
*/

/**
  Function for sending a "system address" request to a Host.

  When calling this function the Device will attempt acquiring the "system address" from 
  any Host within close proximity.
  
  If a host is located within close proximity and pairing is enabled in the Host,
  a "system address" will be sent in return to the Device.

  The new "system address" will apply immediately in the Device, and the new "system address"
  will be stored in non volatile (NV) memory.

  Note. Using OTP devices limits the number of times a new "system address" can
  be stored in NV memory.
  
  @return 

  @retval true if new "system address" was received from a Host.
  @retval false if no "system address" was received from a Host.
*/
bool gzp_address_req_send(void);

/**
  Function for sending a "Host ID request" to a Host.

  The "Host ID" is needed to be able to send encrypted data using 
  gzp_crypt_data_send().

  The request will be sent using the "system address" previously received using
  gzp_address_req_send().

  It is not required that the Host is within close proximity in order to acquire the 
  "Host ID".

  The new "Host ID" will apply immediately for the Device, and the new "Host ID"
  will be stored in non volatile (NV) memory.

  Note. Using OTP devices limits the number of times a new "Host ID" can
  be stored in NV memory.

  @return 
  
  @retval GZP_ID_RESP_PENDING if a "Host ID request" has been sent to the Host, but the Host application has
  not yet decided whether to Grant or Reject the "ID request".
  @retval GZP_ID_RESP_GRANTED if the "Host ID" has been received from the Host. The received "Host ID" will be stored 
  in non volatile memory. 
  @retval GZP_ID_RESP_REJECTED if the Host application has rejected the "Host ID request".
  @retval GZP_ID_RESP_FAILED if failing to send a request or receive a response from the Host. 
*/
gzp_id_req_res_t gzp_id_req_send(void);

/**
  Function for sending encrypted user data to the Host.

  Before any data can be sent the Device must acquire both the Host's 
  "system address" by using gzp_address_req_send() and the "Host ID" by using
  gzp_id_req_send().

  @param *src is a pointer to the data packet to be sent.
  @param length is the length of the data packet to be sent.


  @return
  @retval true if the data was successfully transmitted and decrypted by the Host.
  @retval false if data transmission failed or Host failed to decryption data correctly.
*/
bool gzp_crypt_data_send(const uint8_t *src, uint8_t length);

/**
@name Host functions
*/

/**
  Function for enabling/disabling pairing in a host. When pairing is enabled the host will
  be monitoring for "system address" and "Host ID" requests from Devices.

  A "system address request" received from a Device will always be granted. 
  When a "host ID request" has been received, the Host application have to grant, 
  reject or cancel this by using one of the following functions:

  - gzp_id_req_grant()
  - gzp_id_req_reject()
  - gzp_id_req_cancel() 
  
  @param enable
  @arg true enables pairing.
  @arg false disables pairing.
*/
void gzp_pairing_enable(bool enable);

/**
  Function for executing Gazell Pairing Library host operation.
  
  This function must be called regularly by the Host application.
*/
void gzp_host_execute(void);

/**
  Function returning @b true if a "system address" was delivered to
  a requesting Device during the previous call to gzp_host_execute(); 
*/
bool gzp_address_exchanged(void);

/**
  Function for checking if a "Host ID request" has been received from a Device.
  
  If a request has been received, the Pairing library will enter "ID request pending"
  state.
  
  The application is responsible for responding to this request by calling 
  one of the following functions:
  
  - gzp_id_req_grant()
  - gzp_id_req_reject()
  - gzp_id_req_cancel() 
  
  @return
  @retval true if a "Host ID request" has been received (internal state is "ID request pending") 
  @retval false if no "Host ID request" has been received (internal state is "ID request idle") 
*/
bool gzp_id_req_received(void);

/**
  Function for rejecting the previously received "Host ID request". This function should be called
  only when a "Host ID request" has been received (internal state is "ID request pending"). 
  
  The internal state of the Pairing library will remain "ID request pending" until the a "reject" message
  has been successfully transmitted to the requesting Device. After this the internal state will 
  change to "ID request idle".  
*/
void gzp_id_req_reject(void);

/**
  Function for granting the previously received "Host ID request". This function should be called
  only when a "Host ID request" has been received (internal state is "ID request pending"). 
  
  The internal state of the Pairing library will remain "ID request pending" until the "Host ID" has  
  been successfully transmitted to the requesting Device. After this the internal state will 
  change to "ID request idle".  
*/
void gzp_id_req_grant(void);

/**
  Function returning @b true if encrypted user data has been received.
*/
bool gzp_crypt_user_data_received(void);

/**
  Function for reading encrypted user data.

  Note that the read user data will be automatically decrypted. Only data
  that was decrypted correctly will be presented.

  @param dst* is a pointer to where the received data will be written.
  @param length* is a pointer for returning the number of bytes received. Only 1 byte will
  be writtem to length*.

  @return 
  @retval true if data has been received and is written to dst*
  @retval false if no data has been received.
*/
bool gzp_crypt_user_data_read(uint8_t* dst, uint8_t* length);

/**
@} @} 
*/

#endif
