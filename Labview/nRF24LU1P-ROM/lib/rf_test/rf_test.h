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
 * $LastChangedRevision: 5769 $
 */

/** @file 
@brief Interface for the rf_test module
*/

/** @defgroup rf_test RF tests
@ingroup lib
@brief Library for running RF front-end tests

@details Before using this library the radio should be powered up, ideally in 
the Standby-I state, and the RF clock must be enabled.
The first function you must call is rf_test_init(). You are
then ready to run the tests.
Example of code to use for initialisation on the nrf24LE1:
@code
  RFCE = 0;
  RFCKEN = 1;
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
  wait_ms(3);
  rf_test_init();
@endcode 
Example of code to use for initialisation on the nrf24LU1+:
@code
  RFCE = 0;
  RFCTL |= BIT_4;
  RFCKEN = 1;
  hal_nrf_set_power_mode(HAL_NRF_PWR_UP);
  wait_ms(3);
  rf_test_init();
@endcode 


This library can be used for the following tests :
<h3>TX carrier wave output</h3>
 To generate a constant TX carrier, either unmodulated or modulate with PN9,
the following functions are available:
    - rf_test_set_modulation_payload(): Creates a radio packet with the PN9 
      sequence. The packet is 32 bytes
    - rf_test_start_mod_carrier(): Transmits the PN9 radio packet continuously
    - rf_test_start_unmod_carrier(): Outputs a constant carrier

<h3>RX constant carrier (LO leakage)</h3>
To test the LO leakage you have:
     - rf_test_start_rx_carrier(): Sets the radio in RX mode

<h3>TX/RX channel sweep</h3>
To do a channel sweep on the channel range use these funstions:
    - rf_test_init_rx_sweep(): Initialize for RX sweep
    - rf_test_init_tx_sweep() : Initialize for TX sweep
    - rf_test_txrx_sweep(): Call this function each time you want to
      switch channel. An appropriate wait function must be implemented.

<h3>RX sentitivity</h3>
The functions for RX sensitivity sets the expected payload and caluculates the
bit error rate:
 - rf_test_sensitivity_init(): Init function
 - rf_test_sensitivity_set_expected_data(): Function to set the expected data
 - rf_test_receive_and_compute_packet(): Function to receive data and updating
   the number of errouneous bits. 
 - rf_test_compute_error_rate(): Function to calculate the overall bit error rate. 
  Call it regularly until it returns a value different from 0xFFFF, or call it once 
  rf_test_receive_and_compute_packet returned RF_TEST_RC_FINISHED.

Example of use without interrupt:
@code
  uint8_t expected_payload[32] = {0xff, 0x00, 0x2c, 0x20, 0x88, 0xaa, 0xee, 0xcc,
                                        0x22, 0x33, 0x44, 0x55, 0x16, 0x77, 0x88, 0x99,
                                        0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x10, 0x20,
                                        0x17, 0x18, 0x19, 0x03, 0x0a, 0x20, 0x1c, 0x55};
  rf_test_init();
  rf_test_sensitivity_init();
  rf_test_sensitivity_set_expected_data(32, expected_payload);
  do{
    wait_ms(20);
    rf_test_receive_and_compute_packet();
    bit_error_rate = rf_test_compute_error_rate();
  }while(0xFFFF == bit_error_rate);
  //test is finished, the result is in bit_error_rate and should be divided by 100 to set it in percentage
@endcode 

Example of use with interrupt (IF USING RTX-51 TINY, SET LONG_USR_INTR to 1 in file Conf_tny.A51 !):
@code
void rf_interrupt_handler()
{
  rf_test_ret_t ret_code;
  uint8_t status;

  status = hal_nrf_get_clear_irq_flags();
  switch(status){
    case (1<<HAL_NRF_TX_DS):
      break;
    case (1<<HAL_NRF_RX_DR):
      ret_code = rf_test_receive_and_compute_packet();
      if (ret_code == RF_TEST_RC_FINISHED)
      {
        isr_send_signal(TASK_RX_SENSITIVITY);         //signal the rx_sensitivity that the calculation is finished
        disable_rf();                                 //stop the RF IT
      }
      break;
    case (1<<HAL_NRF_MAX_RT):
      break;
    default:
      break;
}

void main_task()
{
  uint8_t expected_payload[32] = {0xff, 0x00, 0x2c, 0x20, 0x88, 0xaa, 0xee, 0xcc,
                                        0x22, 0x33, 0x44, 0x55, 0x16, 0x77, 0x88, 0x99,
                                        0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff, 0x10, 0x20,
                                        0x17, 0x18, 0x19, 0x03, 0x0a, 0x20, 0x1c, 0x55};
  rf_test_init();
  rf_test_sensitivity_init();
  rf_test_sensitivity_set_expected_data(32, expected_payload);
  enable_rf();

  while (true){
    os_wait1(K_SIG);  //waits for signal from RF interrupt
    //test is finished, the result is in bit_error_rate and should be divided by 100 to set it in percentage
    ber = rf_test_compute_error_rate();
    //eventually terminates this task
  }
}
@endcode 

 @{
*/



#include "hal_nrf.h"


#include <stdbool.h>
#include <intrins.h>

#define RF_TEST_MAX_CHANNEL 81   //!< Upper radio channel

/// Function return codes
typedef enum {
  RF_TEST_RC_OK,                 //!<No error
  RF_TEST_RC_FINISHED,            //!<Current process is finished
  RF_TEST_RC_ERR_PARAM,          //!<Erroneous parameter
  RF_TEST_RC_ERR_BUFFER_FULL,    //!< The internal buffer is full 
  RF_TEST_RC_ERR_NB_RX_BYTES
  //!< The number of received bits in the last packet is different from what was expected
} rf_test_ret_t;


/**@brief Initialisation function
 *
 * Use this function to initialize the rf_test library
*/
void rf_test_init();

/**@brief Sensitivity test initialisation.
 * 
 * Use this function to initialize the Rx Sensitivity test
 *
 * @param  ch Radio Channel
 * @param  pwr radio power 
 * @param  datarate radio data rate
 * @param  address RX address (pointer to an array containing 5 bytes)
 * @return RF_TEST_RC_OK if ok, error code otherwise.
*/
rf_test_ret_t rf_test_sensitivity_init(uint8_t ch, hal_nrf_output_power_t pwr, hal_nrf_datarate_t datarate, uint8_t *address);

/**@brief Function to define expected data for rx_sensitivity test.
 * 
 * Use this function to set the expected data for RxSensitivity test and the expected packet size.
 * \par
 *        By default, the lib uses 32 as pascket size and the following data :
 * \par
 *                0xaa, 0x1b, 0x2c, 0x10, 0x81, 0x01, 0x11, 0x1a, 
 * \n
 *                0xfe, 0xee, 0xcd, 0x12, 0x13, 0xa9, 0x92, 0x13,
 * \n
 *                0x14, 0x17, 0x7d, 0x12, 0xdf, 0xfd, 0xaa, 0xaf, 
 * \n
 *                0x27, 0x65, 0x41, 0x43, 0x3a, 0x2d, 0xc1, 0x55.
 * \n
 *
 * @param  data_size actual packet size 
 * @param  p_expected_data pointer to an array containing data_size bytes  
 */

void rf_test_sensitivity_set_expected_data(uint8_t data_size, uint8_t *p_expected_data);

/**@brief Function to receive and evaluate the number of error in it
 * 
 * For the RX Sensitivity test, this function has to be called on the RF Received data interrupt.
 *    This function get the received data and compute the number of errouneous bits.
 * @return RF_TEST_RC_OK if ok, RF_TEST_RC_FINISHED if enough data has been received for the calulation, error code otherwise.
*/
rf_test_ret_t rf_test_receive_and_compute_packet();

/**@brief Function to calculate and update the overall error rate
 *
 * Use this function to actually compute the error rate. This function should be called regurarly enough to avoid overflowing the buffer.
 * The calculated error rate has to be divided by 100 to get a bit error rate in percent.
 * @return Calculated error rate if finished.
 * @retval 0xFFFF if calculation is not finished
*/
uint16_t rf_test_compute_error_rate();

/** @brief Function to initialize the RX sweep test.
 *
 * Use this function to initialize a RX sweep test. Sets the given test parameters and initialize the RF accordingly.
 *
 * @param  pwr : RF power to set.
 * @param  datarate : RF data rate to set.
 * @param  rx_mode_address : Rx adress.
 * @param  first_channel : first channel to use.
 * @param  last_channel : last channel to use (when reached, goes back to first_channel).
 * @return RF_TEST_RC_OK if ok, error code otherwise.
*/
rf_test_ret_t rf_test_init_rx_sweep(uint8_t pwr, uint8_t datarate, uint8_t *rx_mode_address, uint8_t first_channel, uint8_t last_channel);

/** @brief Function to initialize the TX sweep test.
 *
 * Use this function to initialize a TX sweep test. Sets the given test parameters and initialize the RF accordingly.
 *
 * @param  pwr : RF power to set.
 * @param  datarate : RF data rate to set.
 * @param  first_channel : first channel to use.
 * @param  last_channel : last channel to use (when reached, goes back to first_channel).
 * @return RF_TEST_RC_OK if ok, error code otherwise.
*/
rf_test_ret_t rf_test_init_tx_sweep(uint8_t pwr, uint8_t datarate, uint8_t first_channel, uint8_t last_channel);

/** @brief Sweep function for RX and TX
 
Use this function to actualy do a Tx or Rx sweep. should be called at the desired sweep rate (time per channel).
Usage:
@code
while (testing)
{
  rf_test_txrx_sweep();
  wait(channel_hold();
}
@endcode
*/
void rf_test_txrx_sweep();

/** @brief Function to set a modulation payload
 *
 * Use this function to set a PN9 modulation payload. call this function for tx modulated carrier test.
 *
 */
void rf_test_set_modulation_payload();

/** @brief Function to start a modulated carrier
 *
 * Use this function to start a modulated carrier.
 *
 * @param  channel : RF channel to use.
 * @param  pwr : Rf power.
 * @param  datarate : Rf data rate.
 * @return RF_TEST_RC_OK if ok, error code otherwise.
*/
rf_test_ret_t rf_test_start_mod_carrier(uint8_t channel, uint8_t pwr, uint8_t datarate);

/** @brief RF interrupt function for TX modulated carrier
 *
 * Call this function from RF interrupt for TX modulated carrier.
 * It will assure the packet is retransmitted.
*/
void rf_test_const_carrier_isr(void);

/** @brief Function to start an unmodulated carrier
 *
 * Use this function to start TX unmodulated carrier. 
 *
 * @param  channel : RF channel to use.
 * @param  pwr : Rf power.
 * @param  datarate : Rf data rate.
 * @return RF_TEST_RC_OK if ok, error code otherwise.
*/
rf_test_ret_t rf_test_start_unmod_carrier(uint8_t channel, uint8_t pwr, uint8_t datarate);

/** @brief Function to start RX test
 *
 * Use this function to start RX constant carrier.
 *
 * @param  channel : RF channel to use.
 * @param  pwr : Rf power.
 * @param  datarate : Rf data rate.
 * @param  rx_carrier_address : address to use.
 * @return RF_TEST_RC_OK if ok, error code otherwise.
*/
rf_test_ret_t rf_test_start_rx_carrier(uint8_t channel, uint8_t pwr, uint8_t datarate, uint8_t *rx_carrier_address);

 /** @} */
