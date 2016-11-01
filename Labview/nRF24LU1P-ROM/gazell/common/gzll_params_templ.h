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
 * $LastChangedRevision: 133 $
 */

/** @file
 * @brief Gazell Link Layer default parameters template file.
 */

#ifndef GZLL_PARAMETERS_H__
#define GZLL_PARAMETERS_H__

#define GZLL_DEFAULT_CHANNEL_TAB {3, 23, 40, 61, 75}  // [RF channels]
#define GZLL_DEFAULT_CHANNEL_TAB_SIZE 5               // [bytes]

#define GZLL_DEFAULT_ADDRESS_PIPE0 {1, 3, 6, 9, 12}          
#define GZLL_DEFAULT_ADDRESS_PIPE1 {2, 6, 8, 10, 12}      
#define GZLL_DEFAULT_ADDRESS_PIPE2 3                       
#define GZLL_DEFAULT_ADDRESS_PIPE3 4                        
#define GZLL_DEFAULT_ADDRESS_PIPE4 5                        
#define GZLL_DEFAULT_ADDRESS_PIPE5 6

#define GZLL_DEFAULT_PARAM_DEVICE_MODE 2
#define GZLL_DEFAULT_PARAM_TX_TIMEOUT 0
#define GZLL_DEFAULT_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_ON 2
#define GZLL_DEFAULT_PARAM_TX_ATTEMPTS_PR_CHANNEL_WHEN_SYNC_OFF (GZLL_DEFAULT_CHANNEL_TAB_SIZE * 7)

#define GZLL_DEFAULT_PARAM_HOST_MODE 0
#define GZLL_DEFAULT_PARAM_RX_PIPES 0x3f
#define GZLL_DEFAULT_PARAM_CRYPT_PIPES 0x01
#define GZLL_DEFAULT_PARAM_RX_TIMEOUT 0                 // [# GZLL_DEFAULT_PARAM_RX_PERIODs]
#define GZLL_DEFAULT_PARAM_RX_PERIOD 1008               // [us]
#define GZLL_DEFAULT_PARAM_RX_PERIOD_MODIFIER 360       // [us]
#define GZLL_DEFAULT_PARAM_RX_CHANNEL_HOLD_PERIODS 0    // [# GZLL_DEFAULT_PARAM_RX_PERIODs]

#define GZLL_DEFAULT_PARAM_OUTPUT_POWER HAL_NRF_0DBM
#define GZLL_DEFAULT_PARAM_POWER_DOWN_IDLE_ENABLE 0
#define GZLL_DEFAULT_PARAM_MAX_SYNC_PERIOD 150          // [# GZLL_DEFAULT_PARAM_RX_PERIODs]
#define GZLL_DEFAULT_PARAM_HOST_MODE_1_CYCLE_PERIOD 100 // [# GZLL_DEFAULT_PARAM_RX_PERIODs]
#define GZLL_DEFAULT_PARAM_COLLISION_CHANNEL_SWITCH_LIMIT 50 // [# Channel switches]

#define GZLL_DEFAULT_AES_KEYS {{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16}, \
                               {2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32}}
               
// Static parameters  
#define GZLL_DATARATE GZLL_DR_2MBPS
#define GZLL_MAX_CHANNEL_TAB_SIZE 16                      // [# channels]              
#define GZLL_MAX_FW_PAYLOAD_LENGTH 27                     // [bytes]
#define GZLL_MAX_ACK_PAYLOAD_LENGTH 10                    // [bytes]
#define GZLL_MAX_CRYPT_PIPES 2                            // [# pipes]

#endif // GZLL_PARAMETERS_H__
