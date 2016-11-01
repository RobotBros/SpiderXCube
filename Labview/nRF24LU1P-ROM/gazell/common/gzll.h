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
 * $LastChangedRevision: 232 $
 */

/** @file
 * @brief Gazell Link Layer header file
 */

#ifndef GZLL_H__
#define GZLL_H__

#include <stdint.h>
#include <stdbool.h>
#include "gzll_macros.h"
#include "gzll_params.h"
#include "hal_nrf.h"
#include "assertions.h"

/**
Typical payload length.
Used for calculating host mode 1 burst timing.
*/
#define GZLL_TYP_TX_PAYLOAD_LENGTH 15

/**
Typical transmit time including auto retry delay and 130 us radio startup.
Used for calculating host mode 1 burst behavior.
*/
#define GZLL_TYP_TX_PERIOD (130+((GZLL_CONST_BYTES_PR_PACKET+GZLL_TYP_TX_PAYLOAD_LENGTH) * GZLL_US_PR_BYTE) + GZLL_AUTO_RETR_DELAY)

/**
Dynamic protocol parameters.
*/
typedef enum
{
  GZLL_PARAM_DEVICE_MODE,
  GZLL_PARAM_TX_TIMEOUT,
  GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_ON,
  GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_OFF,
  GZLL_PARAM_HOST_MODE,
  GZLL_PARAM_RX_PIPES,
  GZLL_PARAM_RX_TIMEOUT,
  GZLL_PARAM_HOST_MODE_1_CYCLE_PERIOD,
  GZLL_PARAM_RX_PERIOD,
  GZLL_PARAM_RX_PERIOD_MODIFIER,
  GZLL_PARAM_RX_CHANNEL_HOLD_PERIODS,
  GZLL_PARAM_CRYPT_PIPES,
  GZLL_PARAM_OUTPUT_POWER,
  GZLL_PARAM_POWER_DOWN_IDLE_ENABLE,
  GZLL_PARAM_MAX_SYNC_PERIOD,
  GZLL_PARAM_COLLISION_CHANNEL_SWITCH_LIMIT,
  GZLL_DYN_PARAM_SIZE
} gzll_dyn_params_t;

/**
Maximum values for dynamic protocol parameters.
*/
#define GZLL_PARAMS_MAX {4, 0xffff, 0xffff, 0xffff, 1, 0x3f, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0x3f, 0x03, 1, 0xffff, 0xffff}

/**
Protocol states.
*/
typedef enum
{
  GZLL_IDLE,
  GZLL_DEVICE_ACTIVE,
  GZLL_HOST_ACTIVE
}gzll_states_t;

/**
RX (host) modes.
*/
typedef enum
{
  GZLL_HOST_MODE_0,
  GZLL_HOST_MODE_1,
}gzll_rx_modes_t;

/**
TX (device) modes.
*/
typedef enum
{
  GZLL_DEVICE_MODE_0,
  GZLL_DEVICE_MODE_1,
  GZLL_DEVICE_MODE_2,
  GZLL_DEVICE_MODE_3,
  GZLL_DEVICE_MODE_4,
}gzll_device_modes_t;


#define GZLL_DR_1MBPS 0
#define GZLL_DR_2MBPS 1
#define GZLL_DR_250KBPS 2

/*
CRC length.

Possible values:
@arg HAL_NRF_CRC_8BIT
@arg HAL_NRF_CRC_16BIT
*/
#define GZLL_CRC HAL_NRF_CRC_16BIT

/*
Address width to be used by Gazell.
*/
#define GZLL_ADDRESS_WIDTH HAL_NRF_AW_5BYTES

/*
Number of bytes pEr packet exclusive the payload.
That is: 1 preamble + 2 CRC + 5 address + ~1 byte packet control
*/
#define GZLL_CONST_BYTES_PR_PACKET (1 + 2 + 5 + 1)


/** @defgroup nordic_protocol_gzll_api Application Programming Interface (API) for the Gazell Link Layer
    @ingroup nordic_gzll
@{
*/

/** @name General functions
These functions are useful on both the Host and Device side.
@{
*/

/**
Initialization function for the Gazell Link Layer.

This function must be called before any other Gazell functions.
*/
void gzll_init(void);

/**
Function for setting the address of a pipe.

When in receive mode, the radio can monitor up to six pipes simultaneously.
Each pipe has its own address.

For pipes 0 and 1 all 5 address bytes have to be set.
For pipes 2 to 5 the least significant address byte has to be set.
The remaining address bytes for pipes 2 to 5 will be the same as for pipe 1.

@note The least significant address byte must be unique for all 6 pipes.

It is only allowed to modify the address in GZLL_IDLE state.

@param pipe specifies the pipe number (0-5).

@param address is a pointer to the pipe address.

@sa gzll_goto_idle(), gzll_get_state()
*/
void gzll_set_address(hal_nrf_address_t pipe, const uint8_t *address);

/**
Function for setting the channels to be used by the Gazell protocol.

In order for several units running the Gazell protocol to be able to communicate,
they have to use the same (or overlapping) set of channels.

It is recommended that the selected channels are distributed over a wide
frequency range. The maximum channel range for the nRF24L01 radio is 0 to 123.
However, in order to ensure compliance with world wide frequency regulations
it is recommended to use channels 2 to 80 only.

@param channels is a pointer to the channel array.

@param size is the number of channels in the array.
*/
void gzll_set_channels(uint8_t *channels, uint8_t size);

/**
Function for setting a dynamic protocol parameter.

@param param is the parameter to set. The possible parameters are:
@arg GZLL_PARAM_DEVICE_MODE,
@arg GZLL_PARAM_TX_TIMEOUT,
@arg GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_SWITCH_SYNC_ON,
@arg GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_SWITCH_SYNC_OFF,
@arg GZLL_PARAM_HOST_MODE,
@arg GZLL_PARAM_RX_PIPES,
@arg GZLL_PARAM_CRYPT_PIPES,
@arg GZLL_PARAM_RX_TIMEOUT,
@arg GZLL_PARAM_HOST_MODE_1_BURST_PERIOD,
@arg GZLL_PARAM_RX_PERIOD,
@arg GZLL_PARAM_RX_PERIOD_MOD_CONST,
@arg GZLL_PARAM_RX_CHANNEL_HOLD_PERIODS,
@arg GZLL_PARAM_OUTPUT_POWER,
@arg GZLL_PARAM_POWER_DOWN_IDLE_ENABLE,
@arg GZLL_PARAM_MAX_SYNC_PERIOD,
@arg GZLL_PARAM_COLLISION_CHANNEL_SWITCH_LIMIT

@param value is the new value for the parameter.
*/
void gzll_set_param(gzll_dyn_params_t param, uint16_t value);

/**
Function returning the maximum allowed value for a dynamic parameter.
The minimum value for a parameter is zero.

@param param is the parameter for which to get the maximum value.
Possible parameters are:
@arg GZLL_PARAM_DEVICE_MODE,
@arg GZLL_PARAM_TX_TIMEOUT,
@arg GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_SWITCH_SYNC_ON,
@arg GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_SWITCH_SYNC_OFF,
@arg GZLL_PARAM_HOST_MODE,
@arg GZLL_PARAM_RX_PIPES,
@arg GZLL_PARAM_CRYPT_PIPES,
@arg GZLL_PARAM_RX_TIMEOUT,
@arg GZLL_PARAM_HOST_MODE_1_BURST_PERIOD,
@arg GZLL_PARAM_RX_PERIOD,
@arg GZLL_PARAM_RX_PERIOD_MOD_CONST,
@arg GZLL_PARAM_RX_CHANNEL_HOLD_PERIODS,
@arg GZLL_PARAM_OUTPUT_POWER,
@arg GZLL_PARAM_POWER_DOWN_IDLE_ENABLE,
@arg GZLL_PARAM_MAX_SYNC_PERIOD,
@arg GZLL_PARAM_COLLISION_CHANNEL_SWITCH_LIMIT

@return
Maximum value for the parameter.
*/
uint16_t gzll_get_param_max(gzll_dyn_params_t param);

/**
Function returning the current value of a dynamic parameter.

@param param is the parameter for which to get the current value.
The possible parameters are:
@arg GZLL_PARAM_DEVICE_MODE,
@arg GZLL_PARAM_TX_TIMEOUT,
@arg GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_SWITCH_SYNC_ON,
@arg GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_SWITCH_SYNC_OFF,
@arg GZLL_PARAM_HOST_MODE,
@arg GZLL_PARAM_RX_PIPES,
@arg GZLL_PARAM_CRYPT_PIPES,
@arg GZLL_PARAM_RX_TIMEOUT,
@arg GZLL_PARAM_HOST_MODE_1_BURST_PERIOD,
@arg GZLL_PARAM_RX_PERIOD,
@arg GZLL_PARAM_RX_PERIOD_MOD_CONST,
@arg GZLL_PARAM_RX_CHANNEL_HOLD_PERIODS,
@arg GZLL_PARAM_OUTPUT_POWER,
@arg GZLL_PARAM_POWER_DOWN_IDLE_ENABLE,
@arg GZLL_PARAM_MAX_SYNC_PERIOD,
@arg GZLL_PARAM_COLLISION_CHANNEL_SWITCH_LIMIT

@return
Value for the parameter.
*/
uint16_t gzll_get_param(gzll_dyn_params_t param);

/**
Function for achieving the current protocol state.

@return
Returns the current Gazell protocol state.

@retval GZLL_IDLE
@retval GZLL_DEVICE_ACTIVE
@retval GZLL_HOST_ACTIVE

@sa gzll_goto_idle()
*/
gzll_states_t gzll_get_state(void);

/**
Function telling if the radio is currently active (receiving or transmitting).

This function is used for assisting power management in the application.

For nRF24LE1, the application cannot enter any other power-saving mode deeper
than @b standby when the radio is active.

@return
Returns true if the radio is active and false if the radio is not active.
*/
bool gzll_radio_active();

/**
Function for forcing the protocol to GZLL_IDLE state.

The Gazell protocol has three states: GZLL_IDLE, GZLL_DEVICE_ACTIVE and GZLL_HOST_ACTIVE.

@sa gzll_get_state()
*/
void gzll_goto_idle(void);

/**
Function for checking for data in the RX FIFO.

The function checks if data is available on a given pipe.

If a packet is available, the packet can be read from the FIFO by using
gzll_rx_fifo_read().

@param pipe is the pipe number.
@arg 0-5 will check if a packet is available on this pipe.
@arg >5 will check if a packet is available on any pipe.

@retval true if data is available
@retval false if data is not available

@sa gzll_rx_fifo_read(), gzll_get_rx_data_ready_pipe_number()
*/
bool gzll_rx_data_ready(uint8_t pipe);

/**
Function for checking for data in the RX FIFO.

The function returns the pipe number for the available data.

If a packet is available, the packet can be read from the FIFO by using
gzll_rx_fifo_read().

@returns The function returns the pipe number for the available data.
If no data is available the function returns 0xff.

@sa gzll_rx_fifo_read(), gzll_rx_data_ready()
*/
uint8_t gzll_get_rx_data_ready_pipe_number(void);

/**
Function for reading a packet from the RX FIFO.

Reading the RX FIFO during GZLL_DEVICE_ACTIVE state can cause loss of packets.

@param *dst is the destination for the data.
To avoid memory corruption the destination array should at least have space
for #GZLL_MAX_PAYLOAD_LENGTH bytes.

@param *length returns the number of bytes read.
By passing a @b NULL pointer this parameter will be ignored.

@param *pipe returns the pipe from which data was read.
By passing a @b NULL pointer this parameter will be ignored.

@return
Status of RX FIFO read operation.

@retval true if data was read from the RX FIFO.
@retval false if RX FIFO was empty.

@sa gzll_get_rx_data_ready_pipe_number(), gzll_rx_data_ready()
*/
bool gzll_rx_fifo_read(uint8_t *dst, uint8_t *length, uint8_t *pipe);

/**
Function returning true if the receive signal level for the previous received packet
was above -64 dBm. This function is useful for estimating the relative proximity
of the transmitting device.

@retval true if receive signal level >= -64 dBm.
@retval false if receive signal level < -64 dBm.

@sa gzll_set_address(), gzll_rx_start(), gzll_rx_fifo_read(), gzll_rx_data_ready()

*/
bool gzll_rx_power_high(void);

/**
Function returning the number of channels set up for Gazell.

@sa gzll_set_channels()
*/
uint8_t gzll_get_channel_tab_size();

/**
Function returning the channel table set for Gazell.

@sa gzll_set_channels(), gzll_get_channel_tab_size()
*/
void gzll_get_channels(uint8_t *channels);

/**
Function for getting the address of a pipe.

@param pipe is the pipe number(0-5).

@param *address returns the address of the pipe.
For pipes 0 and 1, five address bytes are returned.
For pipes 2 to 5 one address byte is returned.

@sa gzll_set_address()
*/
void gzll_get_address(uint8_t pipe, uint8_t* address);

/**
Function for flushing the transmit (TX) FIFO.

This function will only have effect in the GZLL_IDLE state.

*/
void gzll_tx_fifo_flush(void);

/**
Function for flushing the receive (RX) FIFO.

The function can be used in any protocol state.

*/
void gzll_rx_fifo_flush(void);

/**
Function for setting the 16 byte (128 bit) AES key
to be used for encrypting/decrypting data.

This key must be distributed in a secure manner to the transmitter and the receiver.

Afterwards, AES 128 bit "Counter mode" is used for the communication.
In Counter mode a given plaintext payload will be encrypted differently each time.

Each pipe can use different AES keys.

@param key128 is a pointer to the 16 byte AES key to be used.

@param pipe is the pipe number for which to set the AES key. The pipe number must be < than
the constant #GZLL_MAX_CRYPT_PIPES.

@sa gzll_get_crypt_key()
*/
void gzll_set_crypt_key(uint8_t pipe, uint8_t* key128);

/**
Function for getting the 16 byte (128 bit) AES key set for a given pipe.

@param key128 is a pointer to where the 16 byte AES shall be copied.

@param pipe is the pipe number for which to get the AES key. The pipe number must be < than
the constant #GZLL_MAX_CRYPT_PIPES.

@sa gzll_set_crypt_key()
*/
void gzll_get_crypt_key(uint8_t pipe, uint8_t* key128);

/**
Function to be called by the radio interrupt service routine.

*/
void gzll_radio_isr_function(void);

/**
Function to be called by the protocol timer interrupt service routine.

*/
void gzll_timer_isr_function(void);

/** @} */

/** @name Device functions
These functions are only useful for a Device application.
@{
*/

/**
Function used during device mode 2 for minimizing the delay from
gzll_tx_data() is called and until the transmission is started.

Device mode 2 is the synchronous frequency agility mode. This means
that the Gazell link layer for every new transmission will use the
previous successful transmission channel, and await starting the transmission
until the host is actually monitoring this channel.

This wait time will be minimized if the gzll_tx_data() function is called
when gzll_dev_mode2_rx_channel_match() returns true.
*/
bool gzll_dev_mode2_rx_channel_match(void);

/**
Function for uploading data to TX FIFO and start data transmission
(Device operation).

The protocol does not have to be in GZLL_IDLE state to issue this function.

If the protocol is in GZLL_DEVICE_ACTIVE state, the data will be uploaded to the
TX FIFO and sent immediately after the ongoing transmission has completed.
This can be used to stream data.

For this to happen the following must be fulfilled:
- The TX FIFO must not be full. (The TX FIFO can hold up to 3 payloads).
- The destination pipe for the data must be the same as for the ongoing
transmission.

If the protocol is in GZLL_HOST_ACTIVE state (Host operation) when issuing
gzll_tx_data(), reception will be terminated and the protocol will switch
to GZLL_DEVICE_ACTIVE state (Device operation).

@param *src is the payload to be transmitted.

@param length is the number of bytes in the payload.

@param pipe is the pipe to be used.
The TX address for the pipe is set using gzll_set_address().

@retval true if data was successfully uploaded to the TX FIFO.
This does NOT indicate that the packet was successfully transmitted,
which should instead be checked with gzll_tx_success()

@retval false if data was not uploaded. In this case any the protocol
state remains the same as before the function was called.

@sa gzll_tx_success(), gzll_get_state(), gzll_goto_idle(void),
gzll_set_address()
*/
bool gzll_tx_data(const uint8_t *src, uint8_t length, uint8_t pipe);

/**
Function for checking the result of the latest transmit operation.

The return value from this function is only reliable when the protocol
is in GZLL_IDLE state

@retval true if the latest packet was successfully transmitted.
@retval false if the latest packet was not successfully transmitted.

@sa gzll_tx_data(), gzll_get_state(), gzll_goto_idle()
*/
bool gzll_tx_success(void);

/**
Function returning the number of transmission attempts for the latest packet.

This function is useful for checking the current radio conditions and
whether the protocol is in sync or not.

The returned value from this function is only reliable
when the protocol is in GZLL_IDLE state.

@return
The number of transmission attempts for the latest packet.

@sa gzll_get_tx_channel_switches()
*/
uint16_t gzll_get_tx_attempts(void);

/**
Function returning the number of channel switches during transmission of the
latest packet. A large number of channel switches will be an indication of
heavy radio interference.

The returned value from this function is only reliable when the protocol is
in GZLL_IDLE state.

@return
The number of channel switches needed during the latest transmission.

@sa gzll_get_state(), gzll_goto_idle()
*/
uint16_t gzll_get_tx_channel_switches(void);

/** @} */

/** @name Host functions
These functions are only useful for a Host application.
@{
*/

/**
Function for starting reception.

This function will make the protocol enter GZLL_HOST_ACTIVE state and start
listening for incoming data.

If this function is called during the GZLL_DEVICE_ACTIVE state, the transmission
will be terminated and the protocol will enter GZLL_HOST_ACTIVE state.

@sa gzll_set_address(), gzll_rx_data_ready(), gzll_rx_fifo_read().
*/
void gzll_rx_start(void);

/**
Convenience function.
*/
#define gzll_rx_stop() gzll_goto_idle()

/**
Function for uploading the payload to be piggybacked onto the next acknowledgement
packet (ACK) for the selected pipe.

It is recommended that the Gazell Link Layer is in IDLE state when uploading an acknowledgement packet.
Alternatively, the application can upload packets during HOST_ACTIVE state as long as the
TX FIFO for the given pipe is not empty.

Uploading acknowledgement packet during HOST_ACTIVE state when the TX FIFO is empty can
cause loss of packets.

@param *src  is a pointer to the payload.

@param length is the number of bytes of the payload.
The maximum number of bytes in an ACK payload is #GZLL_MAX_ACK_PAYLOAD_LENGTH.

@param pipe is the receive pipe.
The payload will be sent as part of the next ACK packet for this pipe.

@retval true if uploading of the ACK payload to TX FIFO was successful.
@retval false if uploading of the ACK payload failed due to full TX FIFO or length
above #GZLL_MAX_ACK_PAYLOAD_LENGTH.

@sa gzll_set_address(), gzll_rx_start()
*/
bool gzll_ack_payload_write(const uint8_t *src, uint8_t length, uint8_t pipe);

/** @} */

/**
@name Hardware dependent functions
These functions must be customized for the MCU being used.
@{
*/

/**
Function used by the Gazell protocol for setting the protocol timer
period.

The implementation of this function must be customized for the actual MCU
used.
*/
void gzll_set_timer_period(uint16_t period);

/** @} @} @} @} */

//-----------------------------------------------------------------------------
// Compile time defines
//-----------------------------------------------------------------------------

#define GZLL_CRYPT_PAYLOAD_OVERHEAD 5
#define GZLL_MAX_INTERNAL_PAYLOAD_LENGTH 32
#define GZLL_MAX_CRYPT_PAYLOAD_LENGTH (GZLL_MAX_INTERNAL_PAYLOAD_LENGTH - GZLL_CRYPT_PAYLOAD_OVERHEAD)

#if(GZLL_MAX_FW_PAYLOAD_LENGTH > GZLL_MAX_ACK_PAYLOAD_LENGTH)
  #define GZLL_MAX_PAYLOAD_LENGTH GZLL_MAX_FW_PAYLOAD_LENGTH
#else
  #define GZLL_MAX_PAYLOAD_LENGTH GZLL_MAX_ACK_PAYLOAD_LENGTH
#endif

#if(GZLL_MAX_FW_PAYLOAD_LENGTH > GZLL_MAX_INTERNAL_PAYLOAD_LENGTH)
{
  #error "GZLL_MAX_FW_PAYLOAD_LENGTH is limited to 32"
}
#endif

#if(GZLL_MAX_ACK_PAYLOAD_LENGTH > GZLL_MAX_INTERNAL_PAYLOAD_LENGTH)
{
  #error "GZLL_MAX_ACK_PAYLOAD_LENGTH is limited to 32"
}
#endif

#define GZLL_INTERNAL_FW_PAYLOAD_LENGTH GZLL_MAX_FW_PAYLOAD_LENGTH
#define GZLL_INTERNAL_ACK_PAYLOAD_LENGTH GZLL_MAX_ACK_PAYLOAD_LENGTH


#if(GZLL_MAX_CRYPT_PIPES == 0)
  #define GZLL_MAX_CRYPT_PIPES_VAL 0x00
#elif (GZLL_MAX_CRYPT_PIPES == 1)
  #define GZLL_MAX_CRYPT_PIPES_VAL 0x01
#elif (GZLL_MAX_CRYPT_PIPES == 2)
  #define GZLL_MAX_CRYPT_PIPES_VAL 0x03
#elif (GZLL_MAX_CRYPT_PIPES == 3)
  #define GZLL_MAX_CRYPT_PIPES_VAL 0x07
#elif (GZLL_MAX_CRYPT_PIPES == 4)
  #define GZLL_MAX_CRYPT_PIPES_VAL 0x0f
#elif (GZLL_MAX_CRYPT_PIPES == 5)
  #define GZLL_MAX_CRYPT_PIPES_VAL 0x1f
#elif (GZLL_MAX_CRYPT_PIPES == 6)
  #define GZLL_MAX_CRYPT_PIPES_VAL 0x3f
#else
  #error GZLL_MAX_CRYPT_PIPES can not exceed 6.
#endif

#if(GZLL_DEFAULT_PARAM_CRYPT_PIPES > GZLL_MAX_CRYPT_PIPES_VAL)
  #error GZLL_MAX_CRYPT_PIPES not in accordance with GZLL_DEFAULT_PARAM_CRYPT_PIPES.
#endif

#if(GZLL_INTERNAL_FW_PAYLOAD_LENGTH > GZLL_INTERNAL_ACK_PAYLOAD_LENGTH)
  #define GZLL_INTERNAL_PAYLOAD_LENGTH GZLL_INTERNAL_FW_PAYLOAD_LENGTH
#else
  #define GZLL_INTERNAL_PAYLOAD_LENGTH GZLL_INTERNAL_ACK_PAYLOAD_LENGTH
#endif

#if(GZLL_DATARATE == GZLL_DR_2MBPS)
  #define GZLL_US_PR_BYTE 4
  #define GZLL_HAL_DATARATE HAL_NRF_2MBPS
  #define GZLL_HOST_CE_LOW_IDLE_DELAY 15 // Host cative -> Idle delay. 9 * 50us = 500 us.
  #if(GZLL_INTERNAL_ACK_PAYLOAD_LENGTH > 15)
    #define GZLL_AUTO_RETR_DELAY 500
  #else
    #define GZLL_AUTO_RETR_DELAY 250
  #endif
#endif

#if(GZLL_DATARATE == GZLL_DR_1MBPS)
  #define GZLL_US_PR_BYTE 8
  #define GZLL_HAL_DATARATE HAL_NRF_1MBPS
  #define GZLL_HOST_CE_LOW_IDLE_DELAY 14 // Host cative -> Idle delay. 13 * 50us = 700 us.
  #if(GZLL_INTERNAL_ACK_PAYLOAD_LENGTH > 5)
    #define GZLL_AUTO_RETR_DELAY 500
  #else
    #define GZLL_AUTO_RETR_DELAY 250
  #endif
#endif

#if(GZLL_DATARATE == GZLL_DR_250KBPS)
  #define GZLL_US_PR_BYTE 32
  #define GZLL_HAL_DATARATE HAL_NRF_250KBPS
  #define GZLL_HOST_CE_LOW_IDLE_DELAY 37 // Host cative -> Idle delay. 36 * 50us = 1850 us.

  #if(GZLL_INTERNAL_ACK_PAYLOAD_LENGTH == 0)
    #define GZLL_AUTO_RETR_DELAY 500
  #elif(GZLL_INTERNAL_ACK_PAYLOAD_LENGTH < 8)
    #define GZLL_AUTO_RETR_DELAY 750
  #elif(GZLL_INTERNAL_ACK_PAYLOAD_LENGTH < 16)
    #define GZLL_AUTO_RETR_DELAY 1000
  #elif(GZLL_INTERNAL_ACK_PAYLOAD_LENGTH < 24)
    #define GZLL_AUTO_RETR_DELAY 1250
  #else
    #define GZLL_AUTO_RETR_DELAY 1500
  #endif
#endif

#ifndef GZLL_CRYPT_ENABLE
  #if(GZLL_MAX_CRYPT_PIPES > 0)
  #warning GZLL_MAX_CRYPT_PIPES > 0 but GZLL_CRYPT_ENABLE is not defined.
  #endif
#endif

#endif // GZLL_H__
/** @} */
/** @} */

