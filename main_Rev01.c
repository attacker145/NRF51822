/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 * Additional files:
   1. nrf_drv_spi.c 
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp.h"
#include "bsp_btn_ble.h"


//SPI LIBRARIES
#include "nrf_drv_spi.h"
#include "app_util_platform.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "boards.h"
#include "app_error.h"

//ADC libraries
#include "nrf_drv_adc_Rev01.h"

void adc_config(void);
void adc_read_sample(void);



// SPI SPI SPI SPI
#if defined(BOARD_PCA10036) || defined(BOARD_PCA10040)
#define SPI_CS_PIN   29 /**< SPI CS Pin.*/
#elif defined(BOARD_PCA10028)
//#define SPI_CS_PIN   4  /**< SPI CS Pin.*/
#define SPI_CS_PIN   19  /**< SPI CS Pin.*/			//04/27/2016
#else
#error "Example is not supported on that board."
#endif

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

#define TEST_STRING "Nordic"
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. 6 bytes*/
//static uint8_t       m_rx_buf[sizeof(TEST_STRING)+1];    /**< RX buffer. 7 bytes*/

//static uint8_t       m_tx_buf[7] = TEST_STRING; /**< TX buffer. 7 bytes*/
static uint8_t       m_rx_buf[13] = {0,0,0,0,0,0,0,0,0,0,0,0,0};    						/**< RX buffer. 13 bytes*/

uint8_t	temp_buf[18];
uint8_t	temp_buf1[27];
uint8_t temp_buf2[3];

static const uint8_t adc_conf_tc[]			= {0x40,0x03, 0x01,0x02,0x40,0x71}; 	 //ADC TC config with VBIAS{0x40,0x03,0x01,0x02,0x40,0x70}, w/o VBIAS {0x40,0x03,0x01,0x00,0x40,0x70} 
//static const uint8_t adc_conf_tc[]			= {0x40,0x03, 0x01,0x02,0x30,0x71}; 
//static const uint8_t adc_conf_tc[]			= {0x40,0x03, 0x01,0x02,0x40,0x61};

/*
	0x13 - AN2+ , AN3-
  0x00 - Bias voltage is disabled
	0x30 - Internal reference is always on, Onboard reference selected 
	0x00 - gain of the PGA = 1
*/
static const uint8_t adc_conf_rtd[]		= {0x40,0x03,0x13,0x00,0x40,0x41};      //ADC RTD config {0x40,0x03,0x13,0x00,0x40,0x40}
//static const uint8_t adc_conf_rtd[]		= {0x40,0x03,0x13,0x00,0x30,0x00};

static const uint8_t read_adc[]					= {0xff,0xff,0xff};     								//ADC conv NULL array for 3 bytes each RTD and TC values
static const uint8_t read_reg [] 				= {0x20,0x03,0x00,0x00,0x00,0x00};
static const uint8_t adc_sleep [] 			= {0x03};

static const uint8_t adc_idac_config[]	= {0x4A,0x01,0x03,0x2F};//250uA
//static const uint8_t adc_idac_config[]	= {0x4A,0x01,0x03,0x20};//250uA
//static const uint8_t ADC_WR_SETUP_EXC[]	= {0x4A,0x01,0x01,0x2F};//50uA
static const uint8_t m_length_exc 			= sizeof(adc_idac_config);




static const uint8_t m_length_setup_rtd = sizeof(adc_conf_rtd);

static const uint8_t m_length_setup_tc 	= sizeof(adc_conf_tc);        	/**< Transfer length. */
static uint8_t m_length_conv 						= sizeof(read_adc);        			/**< Transfer length. */
static const uint8_t length_read_reg 		= sizeof(read_reg);        			/**< Transfer length. */
static const uint8_t length_rx_buf 			= sizeof(m_rx_buf);
static const uint8_t m_length_sleep 		= sizeof(adc_sleep);


uint8_t Rx_bufc[3];
uint8_t Rx_buf[10];
uint8_t tensc;
uint8_t onesc;
uint8_t hundredsc;
uint8_t const * spi_tx_buff_ptr;
uint8_t connected;
uint8_t adv;

//ADC
#define ADC_BUFFER_SIZE 10
static nrf_adc_value_t       adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
/**
 * @brief Function for main application entry.
 */
void adc_read_sample(void)
{
	/*
	 * Use nrf_drv_adc_buffer_convert to perform multiple conversions on multiple channels. 
	 * This function sets up ADC but does not trigger a conversion. Conversions can be triggered 
	 * either by calling nrf_drv_adc_sample or with PPI. The task address for PPI can be retrieved 
	 * through the driver by calling nrf_drv_adc_start_task_get.
	*/
	APP_ERROR_CHECK(nrf_drv_adc_buffer_convert(adc_buffer,ADC_BUFFER_SIZE));//int16_t adc_buffer, 16 bit var array. ADC is 10 bit
	uint32_t i;
	for (i = 0; i < ADC_BUFFER_SIZE; i++)
	{
		// manually trigger ADC conversion
		nrf_drv_adc_sample();
//            // enter into sleep mode
//            __SEV();
//            __WFE();
//            __WFE();

//            nrf_delay_ms(100);
//            LEDS_INVERT(BSP_LED_0_MASK);
   }

}


/*
 * Convert uint32_t hex value to an uint8_t array.
 */
void hexdec_long( uint32_t count )
{	
    uint8_t ones;
    uint8_t tens;
    uint8_t hundreds;
    uint8_t thousands;
    uint8_t thousand10s;
    uint8_t thousand100s;
    uint8_t mill;
    uint8_t mill10s;
    uint8_t mill100s;
    uint8_t bills;
    
		bills						= 0;
		mill100s				= 0;
		mill10s					= 0;			
		mill						= 0;
		thousand100s 		= 0;
		thousand10s 		= 0;
		thousands 			= 0;
		hundreds 				= 0;
		tens  					= 0;						
		ones 						= 0;

		while ( count >= 1000000000 )
		{
			count -= 1000000000;				// subtract 1000000000, one billion		
			bills++;										// increment billions
		}
		while ( count >= 100000000 )
		{
			count -= 100000000;					// subtract 100000000, 100 million		
			mill100s++;									// increment 100th millions
		}
		while ( count >= 10000000 )
		{
			count -= 10000000;					// subtract 10000000		
			mill10s++;									// increment 10th millions
		}
		while ( count >= 1000000 )
		{
			count -= 1000000;						// subtract 1000000	
			mill++;											// increment 1 millions
		}	
		while ( count >= 100000 )
		{
			count -= 100000;						// subtract 100000		
			thousand100s++;							// increment 100th thousands
		}	
		while ( count >= 10000 )
		{
			count -= 10000;             // subtract 10000
			thousand10s++;							// increment 10th thousands
		}
		while ( count >= 1000 )
		{
			count -= 1000;							// subtract 1000	
			thousands++;								// increment thousands
		}
		while ( count >= 100 )
		{
			count -= 100;               // subtract 100	
			hundreds++;                 // increment hundreds
		}
		while ( count >= 10 )
		{
			count -= 10;								// subtract 10		
			tens++;											// increment tens
		}
			ones = count;								// remaining count equals ones
				
			Rx_buf[0]= bills + 0x30;    //Conver HEX to character
			Rx_buf[1]= mill100s + 0x30;
      Rx_buf[2]= mill10s + 0x30;
      Rx_buf[3]= mill + 0x30;
      Rx_buf[4]= thousand100s + 0x30;
      Rx_buf[5]= thousand10s + 0x30;
      Rx_buf[6]= thousands + 0x30;
      Rx_buf[7]= hundreds + 0x30;
      Rx_buf[8]= tens   + 0x30;
      Rx_buf[9]= ones + 0x30;
		return;
}
/*
 * Convert uint8_t hex value to an uint8_t array.
 */
void hexdec_char( uint8_t countc )
{	
	hundredsc = 0;
	tensc  		= 0;						
	onesc 		= 0;
	while ( countc >= 100 )
	{
		countc -= 100;	// subtract 100
		hundredsc++;		// increment hundreds
	}

	while ( countc >= 10 )
	{
		countc -= 10;		// subtract 10
		tensc++;				// increment tens
	}

	onesc = countc;		// remaining count equals ones
	Rx_bufc[0] = hundredsc + 0x30;
	Rx_bufc[1] = tensc + 0x30;
	Rx_bufc[2] = onesc + 0x30;
	return;
}
/**
 * @brief SPI user event handler.
 * @param event
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    //NRF_LOG_PRINTF(" Transfer completed.\r\n");
		uint32_t Result = 0;
		//ADC result is 3 byte long. 
		Result = (uint32_t) m_rx_buf[0];						// Save byte 0 of the ADC result in uint32_t Result
	  Result = (Result << 8);
	  Result = (Result + (uint32_t)m_rx_buf[1]);	// Save byte 1 of the ADC result in uint32_t Result
	  Result = (Result << 8);
	  Result = (Result + (uint32_t)m_rx_buf[2]);	// Save byte 2 of the ADC result in uint32_t Result
	  Result = Result * 2;
		hexdec_long( Result );											// Convert Result into character string. Fills up Rx_buf
//#define UART_SPI
#ifdef UART_SPI	
    if (m_rx_buf[0] != 0)
    {  
      
				NRF_LOG_PRINTF("SPI DATA Rx: %s\r\n", m_rx_buf);		
				NRF_LOG_PRINTF("SPI DATA Rx: %s\r\n", Rx_buf);
				NRF_LOG_PRINTF("   SPI Rx bytes: %d\r\n", m_length_conv);
			
    }
		else
		{
				//printf("SPI Received: No Data\r\n");
		}
#endif		
		//if (m_tx_buf[0] != 0)
    //{				  
				//NRF_LOG_PRINTF(" Trasmitted: %s\r\n",m_tx_buf);
		//		NRF_LOG_PRINTF("SPI DATA Tx: %s\r\n",spibuff);
		//		NRF_LOG_PRINTF("   SPI Tx bytes: %d\r\n",m_length_conv);
    //}
}

/**
 * @brief SPI user event handler.
 * @param event
 */

void spi_event_handler_init(nrf_drv_spi_evt_t const * p_event)
{
    spi_xfer_done = true;
    //NRF_LOG_PRINTF(" Transfer completed.\r\n");		
}
// SPI END


#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include the service_changed characteristic. If not enabled, the server's database cannot be changed for the lifetime of the device. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEVICE_NAME                     "Sensocheck"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(20, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                256                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                256                                         /**< UART RX buffer size. */

static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of 
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        while(app_uart_put(p_data[i]) != NRF_SUCCESS);
    }
    while(app_uart_put('\n') != NRF_SUCCESS);
}


/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/**@snippet [Handling the data received over BLE] */


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
						//adv = 1;
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
						//adv = 0;
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						connected = 1;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
						connected = 0;
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a SoftDevice event to all modules with a SoftDevice 
 *        event handler.
 *
 * @details This function is called from the SoftDevice event interrupt handler after a 
 *          SoftDevice event has been received.
 *
 * @param[in] p_ble_evt  SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
    ble_advertising_on_ble_evt(p_ble_evt);
    bsp_btn_ble_on_ble_evt(p_ble_evt);
    
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
        
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            err_code = ble_advertising_restart_without_whitelist();
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to 
 *          a string. The string will be be sent over BLE when the last character received was a 
 *          'new line' i.e '\n' (hex 0x0D) or if the string has reached a length of 
 *          @ref NUS_MAX_DATA_LENGTH.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN]; // BLE_NUS_MAX_DATA_LEN = 20
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
            {
                /*
								 * Function for sending a string to the peer.
								 * This function sends the input string as an RX characteristic notification to the peer.
							   * uint32_t ble_nus_string_send	(	ble_nus_t * p_nus, uint8_t * p_string, uint16_t length )
								 * p_nus	Pointer to the Nordic UART Service structure.
							   * p_string	String to be sent.
							   * length	Length of the string.
								 */
							
								err_code = ble_nus_string_send(&m_nus, data_array, index);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
                
                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    const app_uart_comm_params_t comm_params =
    {
        RX_PIN_NUMBER,
        TX_PIN_NUMBER,
        RTS_PIN_NUMBER,
        CTS_PIN_NUMBER,
        //APP_UART_FLOW_CONTROL_ENABLED,
			  APP_UART_FLOW_CONTROL_DISABLED,
        false,
        //UART_BAUDRATE_BAUDRATE_Baud115200
				UART_BAUDRATE_BAUDRATE_Baud9600
    };

    APP_UART_FIFO_INIT( &comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOW,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    // Build advertising data struct to pass into @ref ble_advertising_init.
    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = false;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = m_adv_uuids;

    ble_adv_modes_config_t options = {0};
    options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER), 
                                 bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}


void DisplayConfigTC (void){
			uint8_t cntr = 0;		
			uint8_t cntr_int = 0;
			uint8_t cntr_ext = 0;
			//uint8_t str1[] = "Number Bytes sent";	
			uint8_t str2[] = "TC applied config:";
			uint8_t str3[] = "TC config verified";
			uint8_t str4[] = "*******************";																		
			// Configuration data TC
			cntr = 0;
			for (cntr_ext = 0; cntr_ext < 6; cntr_ext++){
				hexdec_char( (uint8_t) adc_conf_tc[cntr_ext] );
				for (cntr_int = 0; cntr_int < 3; cntr_int++){
						temp_buf[cntr] = Rx_bufc[cntr_int];
						cntr++;
				}						
			}
			//Read Internal Registers		*********************************************************************		
			nrf_delay_ms(1);					
			//Read Internal Registers ******** Read Internal Registers ********Read Internal Registers ********
			spi_tx_buff_ptr = read_reg;		// Initialize pointer to read register buffer	
			// Reset rx buffer and transfer done flag
			memset(m_rx_buf, 0, length_rx_buf);
			//memset(m_rx_buf, 0, 13);
			spi_xfer_done = false;	
			// Read internal registers into m_rx_buf
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (uint8_t const *)spi_tx_buff_ptr, length_read_reg, m_rx_buf, length_read_reg));
			//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, read_reg, length_read_reg, m_rx_buf, length_read_reg));
			while (!spi_xfer_done)
			{
				__WFE();
			}				
			//nrf_drv_spi_uninit(&spi);							
			//Read Internal Registers: Convert binary value to decimal ********
			cntr = 0;
			for (cntr_ext = 0; cntr_ext < 2; cntr_ext++){
				hexdec_char( (uint8_t) read_reg[cntr_ext] );
				for (cntr_int = 0; cntr_int < 3; cntr_int++){
						temp_buf1[cntr] = Rx_bufc[cntr_int];
						cntr++;
				}						
			}					
			for (cntr_ext = 0; cntr_ext < 4; cntr_ext++){
				hexdec_char( (uint8_t) m_rx_buf[(cntr_ext + 2)] );
				for (cntr_int = 0; cntr_int < 3; cntr_int++){
						temp_buf1[cntr] = Rx_bufc[cntr_int];
						cntr++;
				}						
			}									
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, str4, sizeof(str4));
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, str2, sizeof(str2));
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, temp_buf, 18);
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, str3, sizeof(str3));
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, temp_buf1, 18);	
			nrf_delay_ms(200);
			//ble_nus_string_send(&m_nus, str1, sizeof(str1));
			//nrf_delay_ms(200);
			//ble_nus_string_send(&m_nus, temp_buf2, 3);
			//nrf_delay_ms(200);
			//ble_nus_string_send(&m_nus, str4, sizeof(str4));	
			//nrf_delay_ms(200);					
}

void DisplayConfigRTD (void){
			uint8_t cntr = 0;		
			uint8_t cntr_int = 0;
			uint8_t cntr_ext = 0;
			//uint8_t str1[] = "Number Bytes sent";	
			uint8_t str2[] = "RTD applied config:";
			uint8_t str3[] = "RTD config verified";
			uint8_t str4[] = "*******************";						
			//if ((cntr%10) == 0 || cntr == 0){												
			// Configuration data TC
			cntr = 0;
			for (cntr_ext = 0; cntr_ext < 6; cntr_ext++){
				hexdec_char( (uint8_t) adc_conf_rtd[cntr_ext] );
				for (cntr_int = 0; cntr_int < 3; cntr_int++){
						temp_buf[cntr] = Rx_bufc[cntr_int];
						cntr++;
				}						
			}
			//Read Internal Registers		*********************************************************************		
			nrf_delay_ms(1);					
			//Read Internal Registers ******** Read Internal Registers ********Read Internal Registers ********
			spi_tx_buff_ptr = read_reg;		// Initialize pointer to read register buffer	
			// Reset rx buffer and transfer done flag
			memset(m_rx_buf, 0, length_rx_buf);
			//memset(m_rx_buf, 0, 13);
			spi_xfer_done = false;	
			// Read internal registers into m_rx_buf
			APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (uint8_t const *)spi_tx_buff_ptr, length_read_reg, m_rx_buf, length_read_reg));
			//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, read_reg, length_read_reg, m_rx_buf, length_read_reg));
			while (!spi_xfer_done)
			{
				__WFE();
			}				
			//nrf_drv_spi_uninit(&spi);							
			//Read Internal Registers: Convert binary value to decimal ********
			cntr = 0;
			for (cntr_ext = 0; cntr_ext < 2; cntr_ext++){
				hexdec_char( (uint8_t) read_reg[cntr_ext] );
				for (cntr_int = 0; cntr_int < 3; cntr_int++){
						temp_buf1[cntr] = Rx_bufc[cntr_int];
						cntr++;
				}						
			}					
			for (cntr_ext = 0; cntr_ext < 4; cntr_ext++){
				hexdec_char( (uint8_t) m_rx_buf[(cntr_ext + 2)] );
				for (cntr_int = 0; cntr_int < 3; cntr_int++){
						temp_buf1[cntr] = Rx_bufc[cntr_int];
						cntr++;
				}						
			}									
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, str4, sizeof(str4));
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, str2, sizeof(str2));
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, temp_buf, 18);
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, str3, sizeof(str3));
			nrf_delay_ms(200);
			ble_nus_string_send(&m_nus, temp_buf1, 18);	
			nrf_delay_ms(200);
			//ble_nus_string_send(&m_nus, str1, sizeof(str1));
			//nrf_delay_ms(200);
			//ble_nus_string_send(&m_nus, temp_buf2, 3);
			//nrf_delay_ms(200);
			//ble_nus_string_send(&m_nus, str4, sizeof(str4));	
			//nrf_delay_ms(200);					
				//}
}
/**@brief Application main function.
 */
//unsigned char *uart;

int main(void)
{
		uint32_t err_code;
		uint8_t cntr = 0;
		uint32_t difference = 0;
		//uint8_t cntr_int = 0;
		uint8_t cntr_ext = 0;
	  connected = 1;
		//uint8_t str1[] = "Number Bytes sent";	
		//uint8_t str2[] = "TC Config Data:";
		//uint8_t str3[] = "TC Regist Data:";
		uint8_t str4[] = "*******************";
		uint8_t str5[] = "TC Data:";
		uint8_t str6[] = "RTD Data:";
		uint8_t str7[] = "Result:";
		uint8_t diff[]				= {0x00,0x00,0x00};		// Difference array
		uint8_t read_tc[] 	 	= {0x00,0x00,0x00};
		uint8_t read_rtd[]  	= {0x00,0x00,0x00};
		uint16_t NmbrOfChar = 10;
		uint8_t *Rx_bufPtr = &Rx_buf[0];
//  uint8_t str5[] = "!DRDY, RDATAC sent";		
    bool 		erase_bonds;	
		
		nrf_gpio_cfg_output(21);				// START
		nrf_gpio_pin_set(21);						// START set high
		nrf_gpio_cfg_output(22);				// ADC RESET
		nrf_gpio_pin_set(22);						// ADC RESET set hight
		nrf_gpio_cfg_input(23, NRF_GPIO_PIN_NOPULL );
		//RESET pin low, START pin low, CS high
		nrf_gpio_pin_clear(22);					// RESET pin low
		nrf_gpio_pin_clear(21);					// START pin low
		nrf_delay_ms(40);
		//RESET pin high, START pin high		
		nrf_gpio_pin_set(21);			  		// START pin high
		nrf_delay_ms(2);
		nrf_gpio_pin_set(22);						// RESET pin high
		
		
    // Initialize.
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
    uart_init();
    
    buttons_leds_init(&erase_bonds);
		ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
		
		//SPI-------------------------------------------------------------------------------
	  /*
	   *nrf_drv_config.h
	   *#define SPI0_CONFIG_SCK_PIN         16
	   *#define SPI0_CONFIG_MOSI_PIN        17
     *#define SPI0_CONFIG_MISO_PIN        18
		*/
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG(SPI_INSTANCE);	// Configure SPI nrf_drv_spi.h line 151
    spi_config.ss_pin = SPI_CS_PIN;				//CS is pin 19
    APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler)); //spi_event_handler_init
		//APP_ERROR_CHECK(nrf_drv_spi_init(&spi, &spi_config, spi_event_handler_init));
							
		spi_tx_buff_ptr = adc_idac_config;		// Initialize pointer to the exitation config	
		// Reset rx buffer and transfer done flag
    memset(m_rx_buf, 0, m_length_exc);
		//memset(m_rx_buf, 0, 13);
    spi_xfer_done = false;	
		// Read internal registers into m_rx_buf
		APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (uint8_t const *)spi_tx_buff_ptr, m_length_exc, m_rx_buf, m_length_exc));
		//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, read_reg, length_read_reg, m_rx_buf, length_read_reg));
		while (!spi_xfer_done)
    {
			__WFE();
    }
		nrf_delay_ms(10);
							
    printf("\r\nUART Start!\r\n");				 
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    										
		cntr = 0;
		// The main loop.
    for (;;)
    {								
				if(connected){
					nrf_gpio_pin_set(21);						// START set high			
					nrf_delay_ms(1);					
					if ((cntr % 2) == 0){	//Initialize TC ADC
						nrf_delay_ms(10);	
						spi_tx_buff_ptr = adc_conf_tc;	//Initialize pointer to ADC config: static uint8_t adc_setup_TC[]			= {0x40,0x03,0x01,0x02,0x40,0x71};			
						// Reset rx buffer and transfer done flag
						memset(m_rx_buf, 0, length_rx_buf);
						//memset(m_rx_buf, 0, 13);
						spi_xfer_done = false;
						//Configure TC ADC		
						APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (uint8_t const *)spi_tx_buff_ptr, m_length_setup_tc, m_rx_buf, m_length_setup_tc));
						//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, adc_setup_TC, m_length_setup_tc, m_rx_buf, m_length_setup_tc));
						while (!spi_xfer_done)
						{
							__WFE();					//Configuring TC ADC
						}								
						//nrf_delay_ms(200);	
						if ((cntr%10) == 0 || cntr == 0){
								DisplayConfigTC();							// Print configuration
						}							
						//Read TC data	Read TC data	Read TC data	Read TC data		
						spi_tx_buff_ptr = read_adc;
						// Reset rx buffer and transfer done flag
						memset(m_rx_buf, 0, length_rx_buf);
						//memset(m_rx_buf, 0, 13);
						spi_xfer_done = false;
					
						while(nrf_gpio_pin_read (23)){}//!DRDY read_data_cont
						//power_manage();											
						APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (uint8_t const *)spi_tx_buff_ptr, m_length_conv, m_rx_buf, m_length_conv)); // Read TC data
						while (!spi_xfer_done)
						{
							__WFE();
						}						
					
						read_tc[0] = m_rx_buf[0]; //MSB
						read_tc[1] = m_rx_buf[1];
						read_tc[2] = m_rx_buf[2];				
																
						NmbrOfChar = 10;
						Rx_bufPtr = &Rx_buf[0];
						//Remove front zeros
						cntr_ext = 0;
						while (Rx_buf[cntr_ext] == '0'){ //TC
							NmbrOfChar = NmbrOfChar - 1;
							Rx_bufPtr = &Rx_buf[cntr_ext];
							cntr_ext++;
						}
						ble_nus_string_send(&m_nus, str4, sizeof(str4));//*******************************
						ble_nus_string_send(&m_nus, str5, sizeof(str5));//"TC Data:"			
						//ble_nus_string_send(&m_nus, Rx_bufPtr, NmbrOfChar);	// Rx_buf is filled up in the handler. Send TC data
											
						ble_nus_string_send(&m_nus, Rx_buf, 10);	// Rx_buf is filled up in the handler. Send TC data
				}			
				else{	//Initialize RTD ADC****************************************************					
					nrf_delay_ms(10);	
					spi_tx_buff_ptr = adc_conf_rtd;	//Initialize pointer to ADC config: static uint8_t adc_setup_TC[]			= {0x40,0x03,0x01,0x02,0x40,0x71};			
					// Reset rx buffer and transfer done flag
					memset(m_rx_buf, 0, length_rx_buf);
					//memset(m_rx_buf, 0, 13);
					spi_xfer_done = false;
					//Configure TC ADC		
					APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (uint8_t const *)spi_tx_buff_ptr, m_length_setup_tc, m_rx_buf, m_length_setup_rtd));
					//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, adc_setup_TC, m_length_setup_tc, m_rx_buf, m_length_setup_tc));
					while (!spi_xfer_done)
					{
						__WFE();					//Configuring RTD ADC
					}			
					//nrf_delay_ms(200);	

					if ((cntr%11) == 0 || cntr == 0){
						DisplayConfigRTD();							// Print configuration RTD
					}
					
					//Read RTD data		
					spi_tx_buff_ptr = read_adc;
					// Reset rx buffer and transfer done flag
					memset(m_rx_buf, 0, length_rx_buf);
					//memset(m_rx_buf, 0, 13);
					spi_xfer_done = false;
				
					while(nrf_gpio_pin_read (23)){} //!DRDY read_data_cont
					//power_manage();					
					APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (uint8_t const *)spi_tx_buff_ptr, m_length_conv, m_rx_buf, m_length_conv)); // Read TC data
					while (!spi_xfer_done)
					{
						__WFE();
					}											
																					
					read_rtd[0] = m_rx_buf[0];
					read_rtd[1] = m_rx_buf[1];
					read_rtd[2] = m_rx_buf[2];
					
					NmbrOfChar = 10;
					Rx_bufPtr = &Rx_buf[0];
					//Remove front zeros
					cntr_ext = 0;
					while (Rx_buf[cntr_ext] == '0'){ //RTD
						NmbrOfChar = NmbrOfChar - 1;
						Rx_bufPtr = &Rx_buf[cntr_ext];
						cntr_ext++;
					}	
					ble_nus_string_send(&m_nus, str6, sizeof(str6));
					//ble_nus_string_send(&m_nus, Rx_bufPtr, NmbrOfChar);	// Rx_buf is filled up in the handler. Send TC data
					ble_nus_string_send(&m_nus, Rx_buf, 10);	// Rx_buf is filled up in the handler. Send TC data		

					diff[0] = read_tc[0] + read_rtd[0]; // MSB
					diff[1] = read_tc[1] + read_rtd[1];
					diff[2] = read_tc[2] + read_rtd[2];	// LSB
													
					//ADC result is 3 byte long. 
					difference = (uint32_t) diff[0];						// Save byte 0 of the ADC result in uint32_t Result
					difference = (difference << 8);
					difference = (difference + (uint32_t)diff[1]);	// Save byte 1 of the ADC result in uint32_t Result
					difference = (difference << 8);
					difference = (difference + (uint32_t)diff[2]);	// Save byte 2 of the ADC result in uint32_t Result
					hexdec_long( difference );											// Convert Result into character string. Fills up Rx_buf																									
					NmbrOfChar = 10;
					Rx_bufPtr = &Rx_buf[0];
					cntr_ext = 0;
					while (Rx_buf[cntr_ext] == '0'){
						NmbrOfChar = NmbrOfChar - 1;
						Rx_bufPtr = &Rx_buf[cntr_ext];
						cntr_ext++;
					}
					
					ble_nus_string_send(&m_nus, str7, sizeof(str7));
					ble_nus_string_send(&m_nus, Rx_bufPtr, NmbrOfChar);	// Rx_buf is filled up in the handler. Send TC data
					//ble_nus_string_send(&m_nus, Rx_buf, 10);	// Rx_buf is filled up in the handler. Send TC data		
					LEDS_INVERT(BSP_LED_1_MASK);
					nrf_delay_ms(500);
					
					}																				
					nrf_gpio_pin_clear(21);						// START set low
					nrf_delay_ms(1);
					// ADC is stopped
				//if (cntr % 3 == 0){	
						
				//}																																			
					if (cntr <255)
						cntr++;
					else
						cntr =0;
				}
				else{	// Idle								
					LEDS_OFF(BSP_LED_0_MASK);
					LEDS_OFF(BSP_LED_1_MASK);
					LEDS_OFF(BSP_LED_2_MASK);
					LEDS_OFF(BSP_LED_3_MASK);
					nrf_gpio_pin_clear(22);					// RESET pin low
					nrf_gpio_pin_clear(21);					// START pin low
									
					spi_tx_buff_ptr = adc_sleep;		// Initialize pointer to read register buffer	
					// Reset rx buffer and transfer done flag
					memset(m_rx_buf, 0, m_length_sleep);				
					spi_xfer_done = false;			
					APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, (uint8_t const *)spi_tx_buff_ptr, m_length_sleep, m_rx_buf, m_length_sleep));
					//APP_ERROR_CHECK(nrf_drv_spi_transfer(&spi, read_reg, length_read_reg, m_rx_buf, length_read_reg));
					while (!spi_xfer_done)
					{
						__WFE();
					}	
					
					sleep_mode_enter();
					//power_manage();				
					//__SEV();
					//__WFE();
					//__WFE();								
				}
					
					
					
			}	
}


/** 
 * @}
 */

