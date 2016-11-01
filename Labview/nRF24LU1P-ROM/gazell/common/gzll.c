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
 * $LastChangedRevision: 2368 $
 */

/** @file
 * @brief Gazell Link Layer implementation
 */

#include "gzll.h"
#include <string.h>

#ifdef __C51__
#include <intrins.h>
#elif __ICC8051__
#include <intrinsics.h>
#endif

#include "hal_delay.h"

#define GZLL_PULSE_FIX

/*-----------------------------------------------------------------------------
  Misc. Internal function prototypes
-----------------------------------------------------------------------------*/

/**
  Linear feedback shift register. The sequence will repeat itself for every 255
  value.

  @param seed is used for resetting the register. If 0 is passed the current
  random sequence will be unmodified.

  @param max_limit specifies the limit for the return data byte. Max value will
  be (max_limit - 1).

  @return
  Returns a pseudo random value between 0 and (max_limit - 1).
*/
static uint8_t gzll_lfsr_get(uint8_t seed, uint8_t max_limit);

/**
  Delay function. Gives a delay of ~(n * 50us) with 16 MHz system clock.
*/
void gzll_delay_50us(uint8_t n);

/**
  Definition of the possible ways of selecting initial channel for a
  transmission.
*/
typedef enum
{
  GZLL_CHANNEL_PREVIOUS_SUCCESS,
  GZLL_CHANNEL_RANDOM,
  GZLL_CHANNEL_ESTIMATED,
  GZLL_CHANNEL_NEXT_INDEX
} gzll_new_tx_ch_t;

/**
  Function for starting a new transmission.

  @param channel_select selects how the initial channel should be selected.
  Possible arguments:
  @arg GZLL_CHANNEL_PREVIOUS_SUCCESS
  @arg GZLL_CHANNEL_RANDOM
  @arg GZLL_CHANNEL_ESTIMATED
*/
static void gzll_start_new_tx(gzll_new_tx_ch_t channel_select);

/**
  Function for reloading the global variable gzll_tries_pr_channel_counter
  holding the number of transmit attempts to be used for the next transmission.
*/
static void gzll_reload_tries_pr_channel_counter(void);

/**
  Function for configuring the number of radio auto retransmit attempts.
  This setting is derived from the global variable
  gzll_tries_pr_channel_counter.
*/
static void gzll_set_radio_auto_retries(void);

/**
  Function for setting the Gazell Link Layer to idle state
*/
static void gzll_set_system_idle(void);

/**
  Function for disabling the Gazell Link Layer interrupts (radio and timer)
  and enabling the radio clock.
*/
static void gzll_interupts_disable_rfck_enable(void);

/**
  Function for enabling the Gazell Link Layer interrupts (radio and timer)
  and disabling the radio clock.
*/
static void gzll_interupts_enable_rfck_disable(void);

/*-----------------------------------------------------------------------------
  Channel Manager (CHM) function prototypes
-----------------------------------------------------------------------------*/

/**
  Function for resetting the channel rotation counters. Used by a device for
  synchronizing the a host.
*/
static void gzll_chm_reset_rx_channel_index(void);

/**
  Function returning the current radio channel index. In host mode the radio
  should always be configured using this frequency. In device mode, this
  function can be used for synchronizing transmit frequency to the receiver (host)
  frequency.
*/
static uint8_t gzll_chm_get_current_rx_channel(void);

/**
  Function returning the next radio channel index the receiver will be monitoring.
  In transmit mode, this function can be used for synchronizing transmit frequency
  to receive frequency.
*/
static uint8_t gzll_chm_get_next_rx_channel(void);

/**
  When calling this function the receive channel rotation will be stopped and the current
  channel will be held for a number of receive periods given by the parameter
  GZLL_PARAM_RX_CHANNEL_HOLD_PERIODS.
*/
static void gzll_chm_hold_rx_channel(void);

/**
  Function executing the channel manager.
  Must be called regularly by the Gazell timer.
*/
static void gzll_chm_execute(void);

/**
  Function for a host to be informed when the receiver should be
  switched on (CE high). The function returns the number of Gazell timer
  periods remaining until CE should be high.

  Thus, CE sholud be set high when this function returns 0.
*/
static uint16_t gzll_chm_get_rx_ce_offset(void);

/**
  Function for a device (PTX) to be informed when the next transmission
  should be started (CE high) in case synchronization is to be used. The
  function returns the number of Gazell timer periods remaining until
  CE high.

  Thus, CE should be set high when this function returns 0.
*/
static uint16_t gzll_chm_get_tx_ce_offset(void);

/*-----------------------------------------------------------------------------
  Radio power manager (PM) function prototypes
-----------------------------------------------------------------------------*/

/**
  Function for powering up/down radio.

  @param on is a boolean telling if the radio should be switched on/off.
*/
static void gzll_set_radio_power_on(bool on);

/*-----------------------------------------------------------------------------
  Function prototypes for assembling / disassembling encrypted packages
-----------------------------------------------------------------------------*/

/**
  Function for assembling an encrypted packet.

  *dst is the address to where the encrypted packet should be written. This packet
  will contain the encrypted data + the 5 byte plain text counter used for
  the encryption.

  *src is the the data to be encrypted.

  *length_src is the length of the data to be encrypted. The length of the
  output packet will equal length_src + GZLL_CRYPT_PAYLOAD_OVERHEAD.

  *aes_key is a pointer to the 16 byte (128 bit) AES key to be used.
*/
static void gzll_crypt_payload_assemble(uint8_t *dst, uint8_t *src, uint8_t length_src, uint8_t *aes_key);

/**
  Function for disassembling (decrypting) an encrypted packet.

  The function extracts the 5 byte plain text counter value from *src and uses
  this counter to decrypt the payload part of *src.

  *dst is the address to where the decrypted packet should be written.

  *src is the address for the package to be encrypted.

  *length_src is the length of the data to be encrypted. The length of the
  output packet will equal length_src - GZLL_CRYPT_PAYLOAD_OVERHEAD.

  *aes_key is a pointer to the 16 byte (128 bit) AES key to be used.
*/
static void gzll_crypt_payload_disassemble(uint8_t *dst, uint8_t *src, uint8_t length_src, uint8_t *aes_key);

/*-----------------------------------------------------------------------------
  Global variables
-----------------------------------------------------------------------------*/

/*
  Dynamic parameters.
*/
static uint16_t xdata gzll_dyn_params[GZLL_DYN_PARAM_SIZE];

/*
  Channel subset array.
*/
static uint8_t xdata gzll_channel_tab[GZLL_MAX_CHANNEL_TAB_SIZE] = GZLL_DEFAULT_CHANNEL_TAB;

/*
  Pipe 0 address shadow register.
*/
static uint8_t xdata gzll_p0_adr[GZLL_ADDRESS_WIDTH] = GZLL_DEFAULT_ADDRESS_PIPE0;

#ifdef __C51__
static uint16_t bdata gzll_bit_storage;
#define GZLL_BIT(_var, _bitnum) sbit _var = gzll_bit_storage ^ _bitnum
#else
#define GZLL_BIT(_var, _bitnum) static xdata bool volatile _var
#endif

/*
  Status variables.
*/
GZLL_BIT(gzll_tx_success_f, 0);       // Result of previous transmission
GZLL_BIT(gzll_tx_setup_modified, 1);  // TX setup modified since previous transmission
GZLL_BIT(gzll_rx_setup_modified, 2);  // RX setup modified since previous receive session
GZLL_BIT(gzll_sync_on, 3);            // Sync on
GZLL_BIT(gzll_rx_dr, 4);              // Received data ready
GZLL_BIT(gzll_rx_power_high_f, 5);    // Receive signal strength high
GZLL_BIT(gzll_radio_active_f, 6);     // For assisting power management in application
GZLL_BIT(gzll_power_on, 7);

static xdata uint8_t volatile gzll_current_tx_pipe;            // Current TX pipe setup
static xdata uint8_t volatile gzll_current_tx_payload_length;  // Current TX payload length
static xdata uint8_t volatile gzll_channel_tab_size;           // Channel subset size
static xdata uint8_t volatile gzll_channel_tab_index;          // Channel subset index
static xdata gzll_states_t volatile gzll_state_var;            // State variable

/*
  Variables for signaling to timer and radio ISR.
*/
GZLL_BIT(gzll_pending_tx_start, 8);       // Transmission should be started in next timer ISR
GZLL_BIT(gzll_pending_goto_idle, 9);      // Goto idle when current radio events completed
GZLL_BIT(gzll_timer_period_modified, 10);  // Timer period temporarily modified
GZLL_BIT(gzll_claim_rfck_en, 11);
GZLL_BIT(b_rfce, 12);

/*
  FIFO holding receive pipes for ACK payloads
  residing in radio HW RX FIFO.
*/
static xdata uint8_t gzll_ack_rx_pipe_fifo[3];        // FIFO holding pipe for received ACK payload
static xdata uint8_t gzll_ack_rx_pipe_fifo_cnt;       // FIFO index

/*
  Counters.
*/
static xdata uint8_t gzll_tries_pr_channel_counter;   // Counter used for counting transmit attempts before channel switch
static xdata uint16_t gzll_sync_period;               // Counter for counting duration since previous successfull TX
static xdata uint16_t gzll_timeout_counter;            // Counter used for TX/RX timeout

/*
  Transmission statistics.
*/
static xdata uint16_t gzll_channel_switch_counter;    // Channel switches for previous transmission
static xdata uint16_t gzll_try_counter;               // Transmit attempts for previous transmission

/*
  Macros / functions for disabling/enabling Gazell interrupts and
  the radio enable (RFEN) and radio clk enable (RFCKEN).

  The logical relation between RFCKEN and RFEN expresses as:

  RFCKEN = GZLL_RFCK_ENABLE() || GZLL_RFCE_HIGH()

*/

#define GZLL_INTERRUPTS_DISABLE() do{ \
  RFEN = 0;                           \
  TIMEREN = 0;                        \
}while(0)

#define GZLL_INTERRUPTS_ENABLE() do{  \
  RFEN = 1;                           \
  TIMEREN = 1;                        \
}while(0)

#ifdef GZLL_PULSE_FIX
#define GZLL_RFCK_ENABLE() do{ \
  gzll_claim_rfck_en = 1;      \
  RFCKEN = 1;                  \
} while(0)

#define GZLL_RFCK_DISABLE() do{ \
  gzll_claim_rfck_en = 0;       \
  RFCKEN = b_rfce; \
} while(0)

#define GZLL_RFCE_PULSE() do{ \
  RFCKEN = 1;                \
  RFCE = 1;                  \
  b_rfce = true; \
  delay_us(10); \
  RFCE = 0; \
} while(false)

#define GZLL_RFCE_HIGH() do{ \
  RFCKEN = 1;                \
  RFCE = 1;                  \
  b_rfce = true; \
} while(false)

#define GZLL_RFCE_LOW() do{ \
  RFCE = 0;                       \
  RFCKEN = gzll_claim_rfck_en;    \
  b_rfce = false; \
} while(0)

#else

#define GZLL_RFCK_ENABLE() do{ \
  gzll_claim_rfck_en = 1;      \
  RFCKEN = 1;                  \
} while(0)

#define GZLL_RFCK_DISABLE() do{ \
  gzll_claim_rfck_en = 0;       \
  RFCKEN = RFCE; \
} while(0)


#define GZLL_RFCE_HIGH() do{ \
  RFCKEN = 1;                \
  RFCE = 1;                  \
} while(false)

#define GZLL_RFCE_PULSE() GZLL_RFCE_HIGH()

#define GZLL_RFCE_LOW() do{ \
  RFCE = 0;                       \
  RFCKEN = gzll_claim_rfck_en;    \
} while(0)

#endif

static void gzll_interupts_disable_rfck_enable()
{
  uint8_t t_ea;
  t_ea = EA;
  EA = 0;
  GZLL_INTERRUPTS_DISABLE();
  GZLL_RFCK_ENABLE();
  EA = t_ea;
}

static void gzll_interupts_enable_rfck_disable()
{
  GZLL_INTERRUPTS_ENABLE();
  GZLL_RFCK_DISABLE();
}

/*-----------------------------------------------------------------------------
  Implementation: Application interface (user functions)
-----------------------------------------------------------------------------*/

void gzll_init(void)
{
  uint8_t temp_adr[GZLL_ADDRESS_WIDTH] = GZLL_DEFAULT_ADDRESS_PIPE1;

  gzll_interupts_disable_rfck_enable();
  GZLL_RFCE_LOW();

  hal_nrf_enable_ack_payload(true);
  hal_nrf_enable_dynamic_payload(true);
  hal_nrf_setup_dynamic_payload(0xff);

  /*
  Initialize status variables.
  */
  gzll_channel_tab_index = 0;
  gzll_channel_tab_size = GZLL_DEFAULT_CHANNEL_TAB_SIZE;

  gzll_pending_goto_idle = false;
  gzll_timer_period_modified = false;

  gzll_current_tx_pipe = 0;
  gzll_pending_tx_start = false;
  gzll_tx_setup_modified = true;
  gzll_rx_setup_modified = true;
  gzll_radio_active_f = false;
  gzll_tx_success_f = true;

  gzll_sync_period = 0;
  gzll_sync_on = false;

  gzll_rx_dr = false;
  gzll_rx_power_high_f = false;
  gzll_ack_rx_pipe_fifo_cnt = 0;

  /*
  Set up default addresses.
  */
  hal_nrf_set_address(HAL_NRF_PIPE0, gzll_p0_adr);
  hal_nrf_set_address(HAL_NRF_PIPE1, temp_adr);

  temp_adr[0] = GZLL_DEFAULT_ADDRESS_PIPE2;
  hal_nrf_set_address(HAL_NRF_PIPE2, temp_adr);

  temp_adr[0] = GZLL_DEFAULT_ADDRESS_PIPE3;
  hal_nrf_set_address(HAL_NRF_PIPE3, temp_adr);

  temp_adr[0] = GZLL_DEFAULT_ADDRESS_PIPE4;
  hal_nrf_set_address(HAL_NRF_PIPE4, temp_adr);

  temp_adr[0] = GZLL_DEFAULT_ADDRESS_PIPE5;
  hal_nrf_set_address(HAL_NRF_PIPE5, temp_adr);

  /*
  Set up default channel.
  */
  hal_nrf_set_rf_channel(gzll_channel_tab[gzll_channel_tab_index]);

  /*
  Initialize dynamic parameters using default values.
  */
  gzll_dyn_params[GZLL_PARAM_DEVICE_MODE] = GZLL_DEFAULT_PARAM_DEVICE_MODE;
  gzll_dyn_params[GZLL_PARAM_TX_TIMEOUT] = GZLL_DEFAULT_PARAM_TX_TIMEOUT;
  gzll_dyn_params[GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_ON] = GZLL_DEFAULT_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_ON;
  gzll_dyn_params[GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_OFF] = GZLL_DEFAULT_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_OFF;
  gzll_dyn_params[GZLL_PARAM_HOST_MODE] = GZLL_DEFAULT_PARAM_HOST_MODE;
  gzll_dyn_params[GZLL_PARAM_RX_PIPES] = GZLL_DEFAULT_PARAM_RX_PIPES;
  gzll_dyn_params[GZLL_PARAM_CRYPT_PIPES] = GZLL_DEFAULT_PARAM_CRYPT_PIPES;
  gzll_dyn_params[GZLL_PARAM_RX_TIMEOUT] = GZLL_DEFAULT_PARAM_RX_TIMEOUT;
  gzll_dyn_params[GZLL_PARAM_HOST_MODE_1_CYCLE_PERIOD] = GZLL_DEFAULT_PARAM_HOST_MODE_1_CYCLE_PERIOD;
  gzll_dyn_params[GZLL_PARAM_RX_PERIOD] = GZLL_DEFAULT_PARAM_RX_PERIOD;
  gzll_dyn_params[GZLL_PARAM_RX_PERIOD_MODIFIER] = GZLL_DEFAULT_PARAM_RX_PERIOD_MODIFIER;
  gzll_dyn_params[GZLL_PARAM_RX_CHANNEL_HOLD_PERIODS] = GZLL_DEFAULT_PARAM_RX_CHANNEL_HOLD_PERIODS;
  gzll_dyn_params[GZLL_PARAM_OUTPUT_POWER] = GZLL_DEFAULT_PARAM_OUTPUT_POWER;
  gzll_dyn_params[GZLL_PARAM_POWER_DOWN_IDLE_ENABLE] = GZLL_DEFAULT_PARAM_POWER_DOWN_IDLE_ENABLE;
  gzll_dyn_params[GZLL_PARAM_MAX_SYNC_PERIOD] = GZLL_DEFAULT_PARAM_MAX_SYNC_PERIOD;
  gzll_dyn_params[GZLL_PARAM_COLLISION_CHANNEL_SWITCH_LIMIT] = GZLL_DEFAULT_PARAM_COLLISION_CHANNEL_SWITCH_LIMIT;

  /*
  Set up default output power.
  */
  hal_nrf_set_output_power((hal_nrf_output_power_t) gzll_dyn_params[GZLL_PARAM_OUTPUT_POWER]);

  /*
  Static radio setup.
  */
  hal_nrf_set_datarate(GZLL_HAL_DATARATE);
  hal_nrf_set_crc_mode(GZLL_CRC);
  hal_nrf_set_address_width(GZLL_ADDRESS_WIDTH);

  /*
  Clear radio IRQ flags.
  */
  //lint -esym(534, hal_nrf_get_clear_irq_flags) "return value ignored"
  hal_nrf_get_clear_irq_flags();

  hal_nrf_flush_rx();
  hal_nrf_flush_tx();

  gzll_set_timer_period(GZLL_DEFAULT_PARAM_RX_PERIOD);
  gzll_set_system_idle();
  gzll_interupts_enable_rfck_disable();
}

void gzll_set_param(gzll_dyn_params_t param, uint16_t val)
{
  ASSERT((gzll_state_var == GZLL_IDLE));
  ASSERT((param < GZLL_DYN_PARAM_SIZE));
  ASSERT(!(param == GZLL_PARAM_DEVICE_MODE && val > GZLL_DEVICE_MODE_4));
  ASSERT(!(param == GZLL_PARAM_HOST_MODE && val > GZLL_HOST_MODE_1));
  ASSERT(!(param == GZLL_PARAM_RX_PIPES && val > 0x3f));
  ASSERT(!(param == GZLL_PARAM_CRYPT_PIPES && val > GZLL_MAX_CRYPT_PIPES_VAL));
  ASSERT(!(param == GZLL_PARAM_OUTPUT_POWER && val > 3));

  gzll_interupts_disable_rfck_enable();

  if(param < GZLL_DYN_PARAM_SIZE)
  {
    gzll_dyn_params[param] = val;

    switch(param)
    {
      case GZLL_PARAM_DEVICE_MODE:
        if((val == GZLL_DEVICE_MODE_0 || val == GZLL_DEVICE_MODE_1))
        {
          gzll_sync_on = false;
        }
        break;
      case GZLL_PARAM_POWER_DOWN_IDLE_ENABLE:
        if(val == 1)
        {
          gzll_set_radio_power_on(false);
        }
        break;
      case GZLL_PARAM_RX_PERIOD:
        gzll_timer_period_modified = 1;
        break;
      case GZLL_PARAM_OUTPUT_POWER:
        hal_nrf_set_output_power((hal_nrf_output_power_t)gzll_dyn_params[GZLL_PARAM_OUTPUT_POWER]);
		break;
      case GZLL_PARAM_RX_PIPES:
        gzll_rx_setup_modified = true;
        break;
    }
  }

  gzll_interupts_enable_rfck_disable();
}

uint16_t gzll_get_param_max(gzll_dyn_params_t param)
{
  uint16_t param_max[GZLL_DYN_PARAM_SIZE] = GZLL_PARAMS_MAX;

  return param_max[param];
}

uint16_t gzll_get_param(gzll_dyn_params_t param)
{
  ASSERT((param < GZLL_DYN_PARAM_SIZE));

  gzll_interupts_disable_rfck_enable();

  if(param < GZLL_DYN_PARAM_SIZE)
  {
    gzll_interupts_enable_rfck_disable();
    return gzll_dyn_params[param];
  }
  else
  {
    gzll_interupts_enable_rfck_disable();
    return 0;
  }
}

uint8_t gzll_get_channel_tab_size()
{
  return gzll_channel_tab_size;
}

void gzll_get_channels(uint8_t *channels)
{
  memcpy(channels, gzll_channel_tab, gzll_channel_tab_size);
}

void gzll_set_channels(uint8_t *channels, uint8_t channel_tab_size)
{
  gzll_interupts_disable_rfck_enable();

  ASSERT((gzll_state_var == GZLL_IDLE));
  ASSERT((channel_tab_size <= GZLL_MAX_CHANNEL_TAB_SIZE));

  gzll_channel_tab_index = 0;
  gzll_channel_tab_size = channel_tab_size;
  memcpy(gzll_channel_tab, channels, gzll_channel_tab_size);

  hal_nrf_set_rf_channel(gzll_channel_tab[gzll_channel_tab_index]);

  gzll_interupts_enable_rfck_disable();
}

void gzll_set_address(hal_nrf_address_t pipe, const uint8_t *address)
{
  ASSERT((gzll_state_var == GZLL_IDLE));
  ASSERT((pipe <= 5));

  gzll_interupts_disable_rfck_enable();

  gzll_tx_setup_modified = true;
  gzll_rx_setup_modified = true;

  if(pipe == HAL_NRF_PIPE0)
  {
    memcpy(gzll_p0_adr, (uint8_t*)address, GZLL_ADDRESS_WIDTH);
  }

  hal_nrf_set_address(pipe, address);

  gzll_interupts_enable_rfck_disable();
}

void gzll_get_address(uint8_t pipe, uint8_t* address)
{
  ASSERT((pipe <= 5));
  ASSERT(address != NULL);

  gzll_interupts_disable_rfck_enable();

  hal_nrf_get_address(pipe, address); //lint !e534 "return value ignored"

  gzll_interupts_enable_rfck_disable();
}

#ifndef GZLL_DEVICE_ONLY


void gzll_rx_start()
{
  uint8_t i;

  gzll_goto_idle();

  if(gzll_rx_setup_modified)
  {
    gzll_interupts_disable_rfck_enable();

    gzll_rx_setup_modified = false;
    gzll_tx_setup_modified = true;

    /*
    Restore pipe 0 address (this may have been altered during transmission)
    */
    hal_nrf_set_address(HAL_NRF_PIPE0, gzll_p0_adr);

    /*
    Enable the receive pipes selected by gzll_set_param()
    */
    hal_nrf_close_pipe(HAL_NRF_ALL);
    for(i = 0; i < 6; i++)
    {
      if(gzll_dyn_params[GZLL_PARAM_RX_PIPES] & (1 << i))
      {
        hal_nrf_open_pipe((hal_nrf_address_t)i, EN_AA);
      }
    }
    hal_nrf_set_operation_mode(HAL_NRF_PRX);
  }

  gzll_set_radio_power_on(true);
  gzll_timeout_counter = 0;
  gzll_state_var = GZLL_HOST_ACTIVE;

  GZLL_RFCE_HIGH();
  gzll_interupts_enable_rfck_disable();
}

bool gzll_ack_payload_write(const uint8_t *src, uint8_t length, uint8_t pipe)
{
  ASSERT(length <= GZLL_MAX_ACK_PAYLOAD_LENGTH && length > 0);
  ASSERT(pipe <= 5);

  gzll_interupts_disable_rfck_enable();

  if(length == 0 || (length > GZLL_MAX_ACK_PAYLOAD_LENGTH) || hal_nrf_tx_fifo_full())
  {
    gzll_interupts_enable_rfck_disable();
    return false;                             // ACK payload not written
  }
  hal_nrf_write_ack_payload(pipe, src, length);
  gzll_interupts_enable_rfck_disable();
  return true;                                // ACK payload successfully written
}

#endif

#define GZLL_UPLOAD_PAYLOAD_TO_RADIO() hal_nrf_write_tx_payload(src, length)

#ifndef GZLL_HOST_ONLY

bool gzll_tx_data(const uint8_t *src, uint8_t length, uint8_t pipe)
{
  uint8_t temp_address[GZLL_ADDRESS_WIDTH];
  uint16_t temp;

  ASSERT(length <= GZLL_MAX_FW_PAYLOAD_LENGTH && length > 0);
  ASSERT(pipe <= 5);

  /*
  Length check to prevent memory corruption. (Note, assertion
  will capture this as well).
  */
  if(length == 0 || length > GZLL_MAX_FW_PAYLOAD_LENGTH)
  {
    return false;
  }

  gzll_current_tx_payload_length = length;

  if(gzll_state_var == GZLL_HOST_ACTIVE)
  {
    gzll_goto_idle();
  }

  gzll_interupts_disable_rfck_enable();

  /*
  If the specified pipe is different from the previous TX pipe,
  the TX setup must be updated
  */
  if(pipe != gzll_current_tx_pipe)
  {
    gzll_current_tx_pipe = pipe;
    gzll_tx_setup_modified = true;
  }

  /*
  Here, state can be GZLL_IDLE or GZLL_DEVICE_ACTIVE
  */
  if(gzll_state_var == GZLL_IDLE)
  {
    if(gzll_tx_setup_modified)       // TX setup has to be restored?
    {
      gzll_tx_setup_modified = false;
      gzll_rx_setup_modified = true;

      hal_nrf_set_operation_mode(HAL_NRF_PTX);
      hal_nrf_open_pipe(HAL_NRF_PIPE0, EN_AA);

       //Read out the full RX address for pipe number "pipe"
      if(pipe == HAL_NRF_PIPE0)
      {
        hal_nrf_set_address(HAL_NRF_TX, gzll_p0_adr);
        hal_nrf_set_address(HAL_NRF_PIPE0, gzll_p0_adr);
      }
      else
      {
        //lint -esym(550,bytes_in_buffer) "variable not accessed"
        //lint -esym(438,bytes_in_buffer) "last assigned value not used"
        uint8_t bytes_in_buffer;
        bytes_in_buffer = hal_nrf_get_address(HAL_NRF_PIPE1, temp_address);
        if(pipe != HAL_NRF_PIPE1)
        {
          switch(pipe)
          {
            default:
            case HAL_NRF_PIPE2:
              bytes_in_buffer = hal_nrf_get_address(HAL_NRF_PIPE2, temp_address);
              break;
            case HAL_NRF_PIPE3:
              bytes_in_buffer = hal_nrf_get_address(HAL_NRF_PIPE3, temp_address);
              break;
            case HAL_NRF_PIPE4:
              bytes_in_buffer = hal_nrf_get_address(HAL_NRF_PIPE4, temp_address);
              break;
            case HAL_NRF_PIPE5:
              bytes_in_buffer = hal_nrf_get_address(HAL_NRF_PIPE5, temp_address);
              break;
          }
        }

        //Here, temp_address will contain the full TX address
        hal_nrf_set_address(HAL_NRF_PIPE0, temp_address);
        hal_nrf_set_address(HAL_NRF_TX, temp_address);

        /*
        Change seed for random generator. Will prevent different devices
        transmitting to the same host from using the same channel hopping
        sequence.
        */
        //lint -esym(534, gzll_lfsr_get) "return value ignored"
        gzll_lfsr_get(pipe, 1);
      }
    }

    // Prepare for new transmission
    gzll_timeout_counter = 0;
    gzll_channel_switch_counter = 0;
    gzll_try_counter = 0;
    hal_nrf_flush_tx();

    GZLL_UPLOAD_PAYLOAD_TO_RADIO();

    gzll_tx_success_f = false;                // Transmission by default "failure"

    temp = gzll_dyn_params[GZLL_PARAM_DEVICE_MODE];

    gzll_set_radio_power_on(true);
    if(gzll_sync_on)
    {
      switch(temp)
      {
        case GZLL_DEVICE_MODE_2:
        default:
          gzll_start_new_tx(GZLL_CHANNEL_PREVIOUS_SUCCESS);
          break;
        case GZLL_DEVICE_MODE_3:
          gzll_start_new_tx(GZLL_CHANNEL_RANDOM);
          break;
        case GZLL_DEVICE_MODE_4:
          gzll_start_new_tx(GZLL_CHANNEL_ESTIMATED);
          break;
      }
    }
    else
    {
      switch(temp)
      {
        case GZLL_DEVICE_MODE_0:
        case GZLL_DEVICE_MODE_2:
          gzll_start_new_tx(GZLL_CHANNEL_PREVIOUS_SUCCESS);
          break;
        default:
          gzll_start_new_tx(GZLL_CHANNEL_RANDOM);
          break;
      }
    }

    gzll_state_var = GZLL_DEVICE_ACTIVE;
    gzll_interupts_enable_rfck_disable();
    return true;                              // Payload successfully written to TX FIFO
  }
  else                                        // Else TRANSMIT state
  {
    /*
    Check if criteria for starting new transmission when already transmitting
    is fulfilled
    */
    if(!gzll_tx_setup_modified &&
       !hal_nrf_tx_fifo_full()
    )
    {
      GZLL_UPLOAD_PAYLOAD_TO_RADIO();
      gzll_interupts_enable_rfck_disable();
      return true;                            // Payload successfully written to TX FIFO
    }
    else
    {
      gzll_interupts_enable_rfck_disable();
      return false;                           // Payload not written to TX FIFO
    }
  }
}

bool gzll_dev_mode2_rx_channel_match()
{
  if(gzll_sync_on)
  {
    if(gzll_channel_tab_index == gzll_chm_get_next_rx_channel())
    {
      if((gzll_dyn_params[GZLL_PARAM_HOST_MODE] == GZLL_HOST_MODE_0) ||
         gzll_chm_get_tx_ce_offset() == 1)
      {
        return true;
      }
    }
    return false;
  }
  else
  {
    return true;
  }
}

bool gzll_tx_success(void)
{
  ASSERT(gzll_state_var != GZLL_DEVICE_ACTIVE);

  return gzll_tx_success_f;
}

uint16_t gzll_get_tx_attempts(void)
{
  ASSERT(gzll_state_var != GZLL_DEVICE_ACTIVE);

  return gzll_try_counter;
}

uint16_t gzll_get_tx_channel_switches(void)
{
  ASSERT(gzll_state_var != GZLL_DEVICE_ACTIVE)
  return  gzll_channel_switch_counter;
}

#endif

void gzll_tx_fifo_flush(void)
{
  gzll_interupts_disable_rfck_enable();

  hal_nrf_flush_tx();

  gzll_interupts_enable_rfck_disable();
}


gzll_states_t gzll_get_state(void)
{
  return gzll_state_var;
}

bool gzll_radio_active()
{
  return gzll_radio_active_f;
}

uint8_t gzll_get_rx_data_ready_pipe_number()
{
  uint8_t dr_rx_pipe;

  gzll_interupts_disable_rfck_enable();

  if(gzll_rx_dr)
  {
    if(gzll_ack_rx_pipe_fifo_cnt > 0)
    {
      dr_rx_pipe = gzll_ack_rx_pipe_fifo[gzll_ack_rx_pipe_fifo_cnt - 1];
    }
    else
    {
      dr_rx_pipe = hal_nrf_get_rx_data_source();
    }
  }
  else
  {
    dr_rx_pipe = 0xff;
  }

  gzll_interupts_enable_rfck_disable();
  return dr_rx_pipe;
}

bool gzll_rx_data_ready(uint8_t pipe)
{
  uint8_t available_rx_data_pipe;

  available_rx_data_pipe = gzll_get_rx_data_ready_pipe_number();

  return (available_rx_data_pipe <= 5 && (pipe == 0xff || pipe == available_rx_data_pipe));
}

bool gzll_rx_fifo_read(uint8_t *dst, uint8_t *length, uint8_t *pipe)
{
  uint8_t temp_pipe;
  uint8_t temp_length;
  uint16_t pipe_and_length;

  ASSERT(dst != NULL);

  gzll_interupts_disable_rfck_enable();

  if(gzll_rx_dr)
  {
    temp_length = hal_nrf_read_rx_payload_width();
    if(temp_length <= 32) //TODO: Remove or comment hardcoded value
    {
      pipe_and_length = hal_nrf_read_rx_payload(dst);
      if(gzll_ack_rx_pipe_fifo_cnt > 0)
      {
        gzll_ack_rx_pipe_fifo_cnt--;
        temp_pipe = gzll_ack_rx_pipe_fifo[gzll_ack_rx_pipe_fifo_cnt];
      }
      else
      {
        temp_pipe = (pipe_and_length >> 8);
      }

      /*
      Handles if two or more payloads were received while only one interrupt
      request serviced.
      */

      if(hal_nrf_rx_fifo_empty())
      {
        gzll_rx_dr = false;
      }

      if(pipe != NULL)
      {
        *pipe = temp_pipe;
      }

      if(length != NULL)
      {
        *length = temp_length;
      }

      gzll_interupts_enable_rfck_disable();
      return true;
    }
    else
    {
      gzll_rx_fifo_flush();
    }
  }

  gzll_interupts_enable_rfck_disable();
  return false;
}

bool gzll_rx_power_high()
{
  return gzll_rx_power_high_f;
}

void gzll_rx_fifo_flush(void)
{
  gzll_interupts_disable_rfck_enable();

  hal_nrf_flush_rx();
  gzll_ack_rx_pipe_fifo_cnt = 0;
  gzll_rx_dr = false;

  gzll_interupts_enable_rfck_disable();
}

void gzll_goto_idle()
{
  if(gzll_state_var == GZLL_DEVICE_ACTIVE)
  {
    gzll_pending_goto_idle = true;

    while(gzll_state_var != GZLL_IDLE)
    ;
  }
  else
  {
    if(gzll_state_var == GZLL_HOST_ACTIVE)
    {
      gzll_interupts_disable_rfck_enable();
      gzll_set_system_idle();
      gzll_interupts_enable_rfck_disable();
    }
  }
}

/*-----------------------------------------------------------------------------
  Implementation: Misc. internal functions
-----------------------------------------------------------------------------*/

static uint8_t gzll_lfsr_get(uint8_t seed, uint8_t max_limit)
{
  static xdata uint8_t pseudoreg = 0xff; // Can never be zero
  uint8_t shiftbit;

  if(seed > 0)
  {
    pseudoreg = seed;
  }

  shiftbit = (pseudoreg << 7) & 0x80;
  shiftbit ^= (pseudoreg << 6) & 0x80;
  shiftbit ^= (pseudoreg << 5) & 0x80;
  shiftbit ^= (pseudoreg & 0x80);

  pseudoreg = (shiftbit | (pseudoreg >> 1));

  return pseudoreg % max_limit;
}

void gzll_delay_50us(uint8_t n)
{
  uint16_t c;

  while(n-- > 0)
  {
      c = 45;
      while(c-- > 0)
      ;
  }
}

static void gzll_start_new_tx(gzll_new_tx_ch_t channel_select)
{
  uint8_t temp;

  gzll_reload_tries_pr_channel_counter();
  gzll_set_radio_auto_retries();

  // If new random channel should be picked
  switch(channel_select)
  {
    case GZLL_CHANNEL_PREVIOUS_SUCCESS:
    default:
      temp = gzll_channel_tab_index;
      break;
    case GZLL_CHANNEL_RANDOM:
      temp = gzll_lfsr_get(0, gzll_channel_tab_size);
      break;
    case GZLL_CHANNEL_ESTIMATED:
      temp = gzll_chm_get_next_rx_channel();
      break;
    case GZLL_CHANNEL_NEXT_INDEX:
      temp = gzll_channel_tab_index + 1;
      temp = temp % gzll_channel_tab_size;
      break;
  }
  // Update RF channel if new is different from current

  if(temp != gzll_channel_tab_index)
  {
    gzll_channel_tab_index = temp;
    hal_nrf_set_rf_channel(gzll_channel_tab[gzll_channel_tab_index]);
    gzll_channel_switch_counter++;
  }

  if(gzll_sync_on)
  {
    // Signal to timer ISR for starting a new transmission
    gzll_pending_tx_start = true;
  }
  else
  {
    gzll_radio_active_f = true;
    GZLL_RFCE_PULSE();
  }

}

static void gzll_reload_tries_pr_channel_counter()
{
  if(gzll_sync_on)
  {
    gzll_tries_pr_channel_counter = gzll_dyn_params[GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_ON];
  }
  else
  {
    gzll_tries_pr_channel_counter = gzll_dyn_params[GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_OFF];
  }
}

static void gzll_set_radio_auto_retries()
{
  if(gzll_tries_pr_channel_counter > 15)
  {
    hal_nrf_set_auto_retr(15, GZLL_AUTO_RETR_DELAY);
  }
  else
  {
    hal_nrf_set_auto_retr((uint8_t)(gzll_tries_pr_channel_counter - 1), GZLL_AUTO_RETR_DELAY);
  }
}

static void gzll_set_system_idle(void)
{
  GZLL_RFCE_LOW();

  if(gzll_state_var == GZLL_HOST_ACTIVE)
  {
    // Add delay to ensure that any ongoing ACK transmission is completed.
    gzll_delay_50us(GZLL_HOST_CE_LOW_IDLE_DELAY);
  }

  if(gzll_dyn_params[GZLL_PARAM_POWER_DOWN_IDLE_ENABLE] == 1)
  {
    gzll_set_radio_power_on(false);
  }
  else
  {
    gzll_set_radio_power_on(true);
  }

  gzll_radio_active_f = false;
  gzll_pending_goto_idle = false;
  gzll_state_var = GZLL_IDLE;
}

/*-----------------------------------------------------------------------------
  Implementation: Interrupt Service Routines (ISR)
-----------------------------------------------------------------------------*/
void gzll_radio_isr_function(void)
{
  #ifndef GZLL_HOST_ONLY
  //lint -esym(644,tries) "Variable may not have been initialized"
  uint8_t tries;
  uint16_t timer_mod_period, temp;
  #endif
  uint8_t status;

  GZLL_INTERRUPTS_DISABLE();
  GZLL_RFCK_ENABLE();

  status = hal_nrf_clear_irq_flags_get_status();

  //If "received data ready" interrupt from radio
  if(status & ((1<<RX_DR)))
  {
    gzll_rx_dr = true;
    gzll_rx_power_high_f = hal_nrf_get_carrier_detect();
    gzll_chm_hold_rx_channel();

    #ifndef GZLL_HOST_ONLY
    /*
    If ACK payload has been received . Here, the actual RX pipe is always 0, so
    rx_pipe_fifo[] needs to store the current tx pipe.
    */
    if(gzll_state_var == GZLL_DEVICE_ACTIVE)
    {
      gzll_ack_rx_pipe_fifo[gzll_ack_rx_pipe_fifo_cnt] = gzll_current_tx_pipe;
      if(gzll_ack_rx_pipe_fifo_cnt < 2)
      {
        gzll_ack_rx_pipe_fifo_cnt++;
      }
    }
    #endif
  }

  //Read radio retransmit attempt counter and update affected variables.
  #ifndef GZLL_HOST_ONLY
  if((status & (1<<MAX_RT)) || (status & ((1<<TX_DS))))
  {
    tries = hal_nrf_get_transmit_attempts() + 1;
    gzll_tries_pr_channel_counter -= tries;
    gzll_try_counter += tries;
  }
  #endif

  //If "data sent" interrupt from radio
  if(status & (1<<TX_DS))
  {
    #ifndef GZLL_HOST_ONLY
    if(gzll_state_var == GZLL_DEVICE_ACTIVE)
    {
      gzll_timer_period_modified = 1;

      timer_mod_period = gzll_dyn_params[GZLL_PARAM_RX_PERIOD] - (gzll_dyn_params[GZLL_PARAM_RX_PERIOD_MODIFIER] + ((uint16_t)((GZLL_CONST_BYTES_PR_PACKET * 2) + gzll_current_tx_payload_length) * GZLL_US_PR_BYTE));
      if(status & ((1<<RX_DR)))
      {
        timer_mod_period -= (GZLL_US_PR_BYTE * GZLL_INTERNAL_ACK_PAYLOAD_LENGTH);
      }

      gzll_set_timer_period(timer_mod_period);

      gzll_chm_reset_rx_channel_index();
      gzll_chm_hold_rx_channel();

      temp = gzll_dyn_params[GZLL_PARAM_DEVICE_MODE];

      gzll_sync_period = gzll_dyn_params[GZLL_PARAM_MAX_SYNC_PERIOD];

      if(temp == GZLL_DEVICE_MODE_2 || temp == GZLL_DEVICE_MODE_3 || temp == GZLL_DEVICE_MODE_4)
      {
        gzll_sync_on = true;
      }

      /*
      Goto IDLE state if TX FIFO empty.
      */
      if(hal_nrf_tx_fifo_empty())
      {
        gzll_tx_success_f = true;
        gzll_set_system_idle();
      }
      else
      {
        gzll_reload_tries_pr_channel_counter();
        gzll_timeout_counter = 0;
        gzll_try_counter = 0;
        GZLL_RFCE_PULSE();
      }
    }
    #endif
  }

  /*
  If "max retransmit" interrupt from radio
  */
  #ifndef GZLL_HOST_ONLY
  if(status & (1<<MAX_RT))
  {
    GZLL_RFCE_LOW();

    gzll_timeout_counter += tries;
    temp = gzll_dyn_params[GZLL_PARAM_TX_TIMEOUT];

    // If TX has timed out, or user has called gzll_goto_idle()
    if((temp != 0 && gzll_timeout_counter >= temp) ||
      gzll_pending_goto_idle)
    {
      gzll_set_system_idle();
    }
    else
    {
      // If tries per channel has elapsed
      if(gzll_tries_pr_channel_counter == 0)
      {
        // If possible unintended sync to another device
        if(gzll_channel_switch_counter > gzll_dyn_params[GZLL_PARAM_COLLISION_CHANNEL_SWITCH_LIMIT])
        {
          gzll_sync_period = 0;
          gzll_sync_on = false;
          gzll_start_new_tx(GZLL_CHANNEL_RANDOM);
        }
        else
        {
          if(gzll_sync_on)
          {
            // If < 1 timer period until radio active -> state shall not go to !radio_active
            if(gzll_chm_get_tx_ce_offset() > 1)
            {
              gzll_radio_active_f = false;
            }
            gzll_start_new_tx(GZLL_CHANNEL_ESTIMATED);
          }
          else
          {
            gzll_start_new_tx(GZLL_CHANNEL_NEXT_INDEX);
          }
        }
      }
      else
      {
        gzll_set_radio_auto_retries();      // Continue retransmits on same channel
        GZLL_RFCE_PULSE();
      }
    }
  }
  #endif

  GZLL_RFCK_DISABLE();
  GZLL_INTERRUPTS_ENABLE();
}

void gzll_timer_isr_function(void)
{
  uint16_t temp;

  GZLL_INTERRUPTS_DISABLE();
  gzll_chm_execute();                         // Execute radio channel manager

  // If timer period temporaly modified - restore correct setting.
  #ifndef GZLL_HOST_ONLY
  if(gzll_timer_period_modified == 1)
  {
    gzll_set_timer_period(gzll_dyn_params[GZLL_PARAM_RX_PERIOD]);
    gzll_timer_period_modified = 0;
  }
  #endif

  // If receive state
  #ifndef GZLL_DEVICE_ONLY
  if(gzll_state_var == GZLL_HOST_ACTIVE)
  {
    temp = gzll_chm_get_current_rx_channel();      // Get channel radio should be monitoring
    GZLL_RFCK_ENABLE();

    // If new channel should be monitored
    if(temp != gzll_channel_tab_index)
    {
      GZLL_RFCE_LOW();
      hal_nrf_set_rf_channel(gzll_channel_tab[temp]); // Change channel
      GZLL_RFCE_HIGH();                       // Set CE high here to minimize RX off time
      gzll_channel_tab_index = temp;
    }

    temp = gzll_chm_get_rx_ce_offset();       // Get number of periods until CE should be set high

    // Radio CE handling
    if(gzll_chm_get_rx_ce_offset() == 0)
    {
      gzll_set_radio_power_on(true);
      gzll_radio_active_f = true;
      GZLL_RFCE_HIGH();
    }
    else
    {
      GZLL_RFCE_LOW();
      gzll_radio_active_f = false;
      gzll_set_radio_power_on(false);
    }

    gzll_timeout_counter++;

    temp = gzll_dyn_params[GZLL_PARAM_RX_TIMEOUT];
    if(gzll_dyn_params[GZLL_PARAM_RX_TIMEOUT] > 0 && (gzll_timeout_counter >= temp))
    {
      gzll_set_system_idle();
    }
  }
  else
  #endif
  {
    // If transmit state
    #ifndef GZLL_HOST_ONLY
    if(gzll_state_var == GZLL_DEVICE_ACTIVE)
    {
      // If pending TX payload
      if(gzll_pending_tx_start)
      {
        temp = gzll_chm_get_tx_ce_offset();

        if( !gzll_sync_on ||
              (temp == 0 && (gzll_channel_tab_index == gzll_chm_get_current_rx_channel()))
        )
        {
          GZLL_RFCE_PULSE();
          gzll_radio_active_f = true;
          gzll_pending_tx_start = 0;
        }
      }
    }
    #endif
  }

  #ifndef GZLL_HOST_ONLY
  if(gzll_sync_period > 0)
  {
    gzll_sync_period--;
  }
  else
  {
    gzll_sync_on = false;
  }
  #endif

  GZLL_RFCK_DISABLE();
  GZLL_INTERRUPTS_ENABLE();
}

/*-----------------------------------------------------------------------------
  Implementation: Radio channel manager (CHM)
-----------------------------------------------------------------------------*/

static xdata uint8_t gzll_chm_rx_channel_index = 0;
static xdata uint8_t gzll_chm_hold_rx_channel_index = 0;
static xdata uint16_t gzll_chm_rx_channel_hold = 0;
static xdata uint16_t gzll_chm_rx_mode1_sequence = 0;

static void gzll_chm_execute()
{
  if(gzll_chm_rx_channel_hold > 0)
  {
    gzll_chm_rx_channel_hold--;
  };

  // Increment channel synchronization counters
  gzll_chm_rx_mode1_sequence = (gzll_chm_rx_mode1_sequence + 1) % gzll_dyn_params[GZLL_PARAM_HOST_MODE_1_CYCLE_PERIOD];

  if( (gzll_dyn_params[GZLL_PARAM_HOST_MODE] == GZLL_HOST_MODE_0) ||
      (gzll_chm_rx_mode1_sequence == 0))
  {
    gzll_chm_rx_channel_index = (gzll_chm_rx_channel_index + 1) % gzll_channel_tab_size;
  }
}

static void gzll_chm_reset_rx_channel_index()
{
  gzll_chm_rx_channel_index = gzll_channel_tab_index;
  gzll_chm_rx_mode1_sequence = 0;
}

static uint8_t gzll_chm_get_next_rx_channel()
{
  if(gzll_chm_rx_channel_hold > 1)
  {
    return gzll_chm_hold_rx_channel_index;
  }
  else
  {
    return (gzll_chm_rx_channel_index + 1) % gzll_channel_tab_size;
  }
}

static uint8_t gzll_chm_get_current_rx_channel()
{
  if(gzll_chm_rx_channel_hold > 0)
  {
    return gzll_chm_hold_rx_channel_index;
  }
  else
  {
    return gzll_chm_rx_channel_index;
  }
}

static void gzll_chm_hold_rx_channel()
{
  gzll_chm_rx_channel_hold = gzll_dyn_params[GZLL_PARAM_RX_CHANNEL_HOLD_PERIODS];
  gzll_chm_hold_rx_channel_index = gzll_channel_tab_index;
}

static uint16_t gzll_chm_get_tx_ce_offset()
{
  if((gzll_dyn_params[GZLL_PARAM_HOST_MODE] == GZLL_HOST_MODE_0) || (gzll_chm_rx_channel_hold > 0))
  {
    return 0;       // CE always high in mode 0
  }
  // Low power RX sequence
  else
  {
    if(gzll_chm_rx_mode1_sequence == 0)
    {
      return 0;
    }
    else
    {
      return (gzll_dyn_params[GZLL_PARAM_HOST_MODE_1_CYCLE_PERIOD] - gzll_chm_rx_mode1_sequence);
    }
  }
}

static uint16_t gzll_chm_get_rx_ce_offset()
{
  uint16_t total_tx_time_per_channel;
  uint16_t rx_periods_per_channel;

  if(gzll_dyn_params[GZLL_PARAM_HOST_MODE] == GZLL_HOST_MODE_0 || (gzll_chm_rx_channel_hold > 0))
  {
    return 0;       // CE always high in mode 0, or during "channel hold"
  }
  else
  {
    if(gzll_chm_rx_mode1_sequence == 0)
    {
      return 0;
    }
    else
    {
      total_tx_time_per_channel = gzll_dyn_params[GZLL_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_OFF] * GZLL_TYP_TX_PERIOD;
      rx_periods_per_channel = total_tx_time_per_channel / gzll_dyn_params[GZLL_PARAM_RX_PERIOD];

      if(gzll_chm_rx_mode1_sequence <= (gzll_channel_tab_size * rx_periods_per_channel))
      {
        return ((rx_periods_per_channel - (gzll_chm_rx_mode1_sequence % rx_periods_per_channel)) - 1);
      }
      else
      {
        return (gzll_dyn_params[GZLL_PARAM_HOST_MODE_1_CYCLE_PERIOD] - gzll_chm_rx_mode1_sequence);
      }
    }
  }
}

static void gzll_set_radio_power_on(bool on)
{
  uint8_t n;

  if(on)
  {
    if(!gzll_power_on)
    {
      hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
      n = 3;
      while(n--) {}
      gzll_power_on = true;
    }
  }
  else
  {
    hal_nrf_set_power_mode(HAL_NRF_PWR_DOWN);
    gzll_power_on = false;
  }
}

/**
@}
*/
