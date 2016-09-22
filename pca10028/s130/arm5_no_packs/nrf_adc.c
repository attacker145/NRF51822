/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** @file
 * @defgroup nrf_adc_example main.c
 * @{
 * @ingroup nrf_adc_example
 * @brief ADC Example Application main file.
 *
 * This file contains the source code for a sample application using the ADC driver.
 * 1. nrf_drv_adc_init
 */
 
#include "nrf.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_drv_adc.h"
//#include "nrf_drv_adc_Rev01.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf_log.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"


//#define ADC_BUFFER_SIZE 10                                /**< Size of buffer for ADC samples.  */
//static nrf_adc_value_t       adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
static nrf_drv_adc_channel_t m_channel_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_2); /**< Channel instance. Default configuration used. */

/**
 * @file
 * @brief ADC HAL implementation
 */

#include "nrf_adc.h"

#ifndef NRF52
//#ifdef adc_debug
/**
 * @brief Function for configuring ADC.
 *
 * This function powers on ADC and configures it. ADC is in DISABLE state after configuration,
 * so it should be enabled before using it.
 *
 * @param[in] config  Requested configuration.
 */
void nrf_adc_configure(nrf_adc_config_t * config)
{
    uint32_t config_reg = 0;

    config_reg |= ((uint32_t)config->resolution << ADC_CONFIG_RES_Pos) & ADC_CONFIG_RES_Msk;
    config_reg |= ((uint32_t)config->scaling << ADC_CONFIG_INPSEL_Pos) & ADC_CONFIG_INPSEL_Msk;
    config_reg |= ((uint32_t)config->reference << ADC_CONFIG_REFSEL_Pos) & ADC_CONFIG_REFSEL_Msk;

    if (config->reference & ADC_CONFIG_EXTREFSEL_Msk)
    {
        config_reg |= config->reference & ADC_CONFIG_EXTREFSEL_Msk;
    }

    /* select input */
    nrf_adc_input_select(NRF_ADC_CONFIG_INPUT_DISABLED);

    /* set new configuration keeping selected input */
    NRF_ADC->CONFIG = config_reg | (NRF_ADC->CONFIG & ADC_CONFIG_PSEL_Msk);
}


/**
 * @brief Blocking function for executing single ADC conversion.
 *
 * This function selects the desired input, starts a single conversion,
 * waits for it to finish, and returns the result.
 * ADC is left in STOP state, the given input is selected.
 * This function does not check if ADC is initialized and powered.
 *
 * @param[in] input Requested input to be selected.
 *
 * @return Conversion result
 */
int32_t nrf_adc_convert_single(nrf_adc_config_input_t input)
{
    int32_t val;

    nrf_adc_input_select(input);
    nrf_adc_start();

    while (!nrf_adc_conversion_finished())
    {
    }
    nrf_adc_conversion_event_clean();
    val = nrf_adc_result_get();
    nrf_adc_stop();
    return val;
}
//#endif



/**
 * @brief ADC interrupt handler.
 */

static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
//void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {
        uint32_t i;
        for (i = 0; i < p_event->data.done.size; i++)
        {
            NRF_LOG_PRINTF("Current sample value: %d\r\n", p_event->data.done.p_buffer[i]);
        }
    }
}

/**
 * @brief ADC initialization.
 */
//static void adc_config(void)
extern uint8_t	adc_configured;

void adc_config(void)
{
    ret_code_t ret_code;
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;//#define ADC_CONFIG_IRQ_PRIORITY APP_IRQ_PRIORITY_LOW

    ret_code = nrf_drv_adc_init(&config, adc_event_handler); // Returns NRF_SUCCESS 
    
		if (!ret_code) // There are 2 return codes NRF_ERROR_INVALID_STATE and NRF_SUCCESS. #define NRF_ERROR_INVALID_STATE (NRF_ERROR_BASE_NUM + 8); #define NRF_SUCCESS	(NRF_ERROR_BASE_NUM + 0)
			adc_configured = 1; // Indicates that ADC has been configured
	
		APP_ERROR_CHECK(ret_code);
			
    nrf_drv_adc_channel_enable(&m_channel_config);
}


#endif

/**
 * @brief Function for main application entry.
 */
/*
void adc_read_sample(void)
{
//    LEDS_CONFIGURE(BSP_LED_0_MASK);
//    LEDS_OFF(BSP_LED_0_MASK);

//		adc_config();
//    UNUSED_RETURN_VALUE(NRF_LOG_INIT());

//    NRF_LOG_PRINTF("ADC example\r\n");

//    while (true)
//    {
        APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));
        uint32_t i;
        for (i = 0; i < ADC_BUFFER_SIZE; i++)
        {
//            // manually trigger ADC conversion
            nrf_drv_adc_sample();
//            // enter into sleep mode
//            __SEV();
//            __WFE();
//            __WFE();

//            nrf_delay_ms(100);
//            LEDS_INVERT(BSP_LED_0_MASK);
        }
//    }
}
*/
