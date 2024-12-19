/*******************************************************************************
 * File Name: main.c
 *
 * Description: This file contains the starting point of Bluetooth LE XENSIV
 *                 DPS368 sensing service application.
 *
 * Related Document: See README.md
 *
 *
 *********************************************************************************
 Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
 an affiliate of Cypress Semiconductor Corporation.  All rights reserved.

 This software, including source code, documentation and related
 materials ("Software") is owned by Cypress Semiconductor Corporation
 or one of its affiliates ("Cypress") and is protected by and subject to
 worldwide patent protection (United States and foreign),
 United States copyright laws and international treaty provisions.
 Therefore, you may use this Software only as provided in the license
 agreement accompanying the software package from which you
 obtained this Software ("EULA").
 If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 non-transferable license to copy, modify, and compile the Software
 source code solely for use in connection with Cypress's
 integrated circuit products.  Any reproduction, modification, translation,
 compilation, or representation of this Software except as specified
 above is prohibited without the express written permission of Cypress.

 Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 reserves the right to make changes to the Software without notice. Cypress
 does not assume any liability arising out of the application or use of the
 Software or any product or circuit described in the Software. Cypress does
 not authorize its products for use in any products where a malfunction or
 failure of the Cypress product may reasonably be expected to result in
 significant property damage, injury or death ("High Risk Product"). By
 including Cypress's product in a High Risk Product, the manufacturer
 of such system or application assumes all risk of such use and in doing
 so agrees to indemnify Cypress against all liability.
 *******************************************************************************/


/*******************************************************************************
 *        Header Files
 *******************************************************************************/
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "cybt_platform_trace.h"
#include "cyhal.h"
#include "cyhal_gpio.h"
#include "stdio.h"
#include "cyabs_rtos.h"
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <string.h>
#include <timers.h>
#include "GeneratedSource/cycfg_gatt_db.h"
#include "app_bt_gatt_handler.h"
#include "app_bt_utils.h"
#include "wiced_bt_ble.h"
#include "wiced_bt_uuid.h"
#include "wiced_memory.h"
#include "wiced_bt_stack.h"
#include "cycfg_bt_settings.h"
#include "cycfg_gap.h"
#include "cybsp_bt_config.h"

#include "sensor_task.h"
/*******************************************************************************
 *        Macro Definitions
 *******************************************************************************/
/* Number of advertisment packet */
#define NUM_ADV_PACKETS                 (3u)

#define LED_TIMER_INT_PRIORITY       (7u)
#define LED_TIMER_TARGET_FREQUENCY   (10000u)
#define LED_TIMER_COUNT_PERIOD       (999u)
/******************************************************************************
 *                                 TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 *        Variable Definitions
 *******************************************************************************/
/* Configuring Higher priority for the application */
volatile int uxTopUsedPriority;

/* Manages runtime configuration of Bluetooth stack */
extern const wiced_bt_cfg_settings_t wiced_bt_cfg_settings;

/* FreeRTOS variable to store handle of task created to update and send
 * sensor data */
TaskHandle_t sensor_task_handle;

/* Status variable for connection ID */
uint16_t app_bt_conn_id;

/*timer object used*/
cyhal_timer_t led_timer_obj;

/*timer configuration*/
const cyhal_timer_cfg_t led_timer_cfg =
{
    .compare_value = 0,                  // Timer compare value, not used
    .period        = LED_TIMER_COUNT_PERIOD, // Defines the timer period
    .direction     = CYHAL_TIMER_DIR_UP, // Timer counts up
    .is_compare    = false,              // Don't use compare mode
    .is_continuous = true,               // Run the timer indefinitely
    .value         = 0                   // Initial value of counter
};
/*******************************************************************************
 *        Function Prototypes
 *******************************************************************************/

/* Callback function for Bluetooth stack management type events */
static wiced_bt_dev_status_t
app_bt_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data);

/* This function sets the advertisement data */
static wiced_result_t app_bt_set_advertisement_data(void);

/* This function initializes the required BLE ESS & dps368 task */
static void bt_app_init(void);

/* This function starts the advertisements */
static void app_start_advertisement(void);

/* interrupt for led timer*/
static void isr_led_timer(void* callback_arg, cyhal_timer_event_t event);
/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/

/*
 *  Entry point to the application. Set device configuration and start BT
 *  stack initialization.  The actual application initialization will happen
 *  when stack reports that BT device is ready.
 */
int main(void)
{
    uxTopUsedPriority = configMAX_PRIORITIES - 1;
    wiced_result_t wiced_result;
    BaseType_t rtos_result;

    /* Initialize and Verify the BSP initialization */
    CY_ASSERT(CY_RSLT_SUCCESS == cybsp_init());

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    cy_retarget_io_init(CYBSP_DEBUG_UART_TX,
                        CYBSP_DEBUG_UART_RX,
                        CY_RETARGET_IO_BAUDRATE);

    /* Initialising the HCI UART for Host contol */
    cybt_platform_config_init(&cybsp_bt_platform_cfg);

    /* Debug logs on UART port */
    /* Debug logs on UART port */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("****** XENSIV TM DPS368 Sensing Service ******\r\n");
    printf("********************************************\r\n");
    printf("Application version: %d.%d.%d\r\n",APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_VERSION_BUILD);
    printf("********************************************\r\n\n");

    /* Register call back and configuration with stack */
    wiced_result = wiced_bt_stack_init(app_bt_management_callback,
                                 &wiced_bt_cfg_settings);

    /* Check if stack initialization was successful */
    if (WICED_BT_SUCCESS == wiced_result) {
        printf("Bluetooth Stack Initialization Successful \n");
    } else {
        printf("Bluetooth Stack Initialization failed!!\n");
    }

    /* Start Sensor Task */
    rtos_result = xTaskCreate(sensor_task,
                              SENSOR_TASK_NAME,
                              SENSOR_TASK_STACK_SIZE,
                              NULL,
                              SENSOR_TASK_PRIORITY,
                              &sensor_task_handle);
    if(pdPASS == rtos_result)
    {
        printf("Sensor task created successfully\n");
    }
    else
    {
        printf("Sensor task creation failed\n");
    }

    /* Start the FreeRTOS scheduler */
    vTaskStartScheduler();

    /* Should never get here */
    CY_ASSERT(0);
}

/*
 * Function Name: app_bt_management_callback()
 *
 *@brief
 *  This is a Bluetooth stack event handler function to receive management events
 *  from the Bluetooth LE stack and process as per the application.
 *
 * @param wiced_bt_management_evt_t  Bluetooth LE event code of one byte length
 * @param wiced_bt_management_evt_data_t  Pointer to Bluetooth LE management event
 *                                        structures
 *
 * @return wiced_result_t Error code from WICED_RESULT_LIST or BT_RESULT_LIST
 *
 */
static wiced_result_t
app_bt_management_callback(wiced_bt_management_evt_t event,
                           wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_bt_dev_status_t status = WICED_ERROR;


    switch (event) {

    case BTM_ENABLED_EVT:
    {
        printf("\nThis application implements a custom XENSIV TM DPS368 Sensing\n" \
                "Service and sends XENSIV TM DPS368 sensor data over BLE\n");

        printf("Discover this device with the name:%s\n", app_gap_device_name);

        print_local_bd_address();

        printf("\n");
        printf("Bluetooth Management Event: \t");
        printf("%s", get_btm_event_name(event));
        printf("\n");

        /* Perform application-specific initialization */
        bt_app_init();
    }break;

    case BTM_DISABLED_EVT:
        /* Bluetooth Controller and Host Stack Disabled */
        printf("\n");
        printf("Bluetooth Management Event: \t");
        printf("%s", get_btm_event_name(event));
        printf("\n");
        printf("Bluetooth Disabled\n");
        break;

    case BTM_BLE_ADVERT_STATE_CHANGED_EVT:
    {
        wiced_bt_ble_advert_mode_t *p_adv_mode = &p_event_data->ble_advert_state_changed;
        /* Advertisement State Changed */
        printf("\n");
        printf("Bluetooth Management Event: \t");
        printf("%s", get_btm_event_name(event));
        printf("\n");
        printf("\n");
        printf("Advertisement state changed to ");
        printf("%s", get_btm_advert_mode_name(*p_adv_mode));
        printf("\n");

        if(*p_adv_mode == BTM_BLE_ADVERT_OFF)
        {
            cyhal_timer_stop(&led_timer_obj);
            cyhal_gpio_write(CONNECTION_LED, (app_bt_conn_id) ? CYBSP_LED_STATE_ON : CYBSP_LED_STATE_OFF);
        }
        else
        {
            cyhal_timer_start(&led_timer_obj);
        }
    }break;

    default:
        printf("\nUnhandled Bluetooth Management Event: %d %s\n",
                event,
                get_btm_event_name(event));
        break;
    }

    return (status);
}

/*
 Function name:
 bt_app_init

 Function Description:
 @brief    This function is executed if BTM_ENABLED_EVT event occurs in
           Bluetooth management callback.

 @param    void

 @return    void
 */
static void bt_app_init(void)
{
    wiced_bt_gatt_status_t gatt_status = WICED_BT_GATT_ERROR;
    cy_rslt_t rslt;

    /* Register with stack to receive GATT callback */
    gatt_status = wiced_bt_gatt_register(app_bt_gatt_event_callback);
    printf("\n gatt_register status:\t%s\n",get_gatt_status_name(gatt_status));

    /* Initialize the User LED */
    rslt = cyhal_gpio_init(CONNECTION_LED,
                           CYHAL_GPIO_DIR_OUTPUT,
                           CYHAL_GPIO_DRIVE_STRONG,
                           CYBSP_LED_STATE_OFF);
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Error initializing user led 2\n");
        CY_ASSERT(0);
    }

    /* Initialize the led timer object. Does not use pin output ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL) */
    rslt = cyhal_timer_init(&led_timer_obj, NC, NULL);

    /*Apply timer configuration such as period, count direction, run mode, etc*/
    if (CY_RSLT_SUCCESS == rslt)
    {
        rslt = cyhal_timer_configure(&led_timer_obj, &led_timer_cfg);
    }

    /* Set the frequency of timer to 10000 Hz */
    if (CY_RSLT_SUCCESS == rslt)
    {
        rslt = cyhal_timer_set_frequency(&led_timer_obj, LED_TIMER_TARGET_FREQUENCY);
    }

    /* register timer interrupt and enable */
    if (CY_RSLT_SUCCESS == rslt)
    {
        /* Assign the ISR to execute on timer interrupt */
        cyhal_timer_register_callback(&led_timer_obj, isr_led_timer, NULL);

        /* Set the event on which timer interrupt occurs and enable it */
        cyhal_timer_enable_event(&led_timer_obj, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                LED_TIMER_INT_PRIORITY, true);
    }

    /* Initialize GATT Database */
    gatt_status = wiced_bt_gatt_db_init(gatt_database, gatt_database_len, NULL);
    if (WICED_BT_GATT_SUCCESS != gatt_status) {
        printf("\n GATT DB Initialization not successful err 0x%x\n", gatt_status);
    }

    /* Initialize sensor task */
    rslt = sensor_task_init();
    if (CY_RSLT_SUCCESS != rslt)
    {
        printf("Error initializing sensor task\n");
        CY_ASSERT(0);
    }

    /* Start Bluetooth LE advertisements */
    app_start_advertisement();
}


/**
 * @brief This function starts the Blueooth LE advertisements and describes
 *        the pairing support
 */
static void app_start_advertisement(void)
{
    wiced_result_t wiced_status;

    /* Set Advertisement Data */
    wiced_status = app_bt_set_advertisement_data();
    if (WICED_SUCCESS != wiced_status) {
        printf("Raw advertisement failed err 0x%x\n", wiced_status);
    }

    /* Do not allow peer to pair */
    wiced_bt_set_pairable_mode(WICED_FALSE, FALSE);

    /* Start Undirected LE Advertisements on device startup. */
    wiced_status = wiced_bt_start_advertisements(BTM_BLE_ADVERT_UNDIRECTED_HIGH,
                                                 BLE_ADDR_PUBLIC,
                                                 NULL);

    if (WICED_SUCCESS != wiced_status) {
        printf( "Starting undirected Bluetooth LE advertisements"
                "Failed err 0x%x\n", wiced_status);
    }
}

/*
 Function Name:
 app_bt_set_advertisement_data

 Function Description:
 @brief  Set Advertisement Data

 @param void

 @return wiced_result_t WICED_SUCCESS or WICED_failure
 */
static wiced_result_t app_bt_set_advertisement_data(void)
{

    wiced_result_t wiced_result = WICED_SUCCESS;
    wiced_result = wiced_bt_ble_set_raw_advertisement_data( NUM_ADV_PACKETS,
                                                            cy_bt_adv_packet_data);

    return (wiced_result);
}

/*******************************************************************************
* Function Name: isr_led_timer
********************************************************************************
* Summary:
* This is the timer interrupt callback function. Used to toggle LED2 based on BLE
* Advertisement state
*
* Parameters:
*  void* callback_arg
*  cyhal_timer_event_t event
*
* Return:
* NC
*
*******************************************************************************/
static void isr_led_timer(void* callback_arg, cyhal_timer_event_t event)
{
    (void)callback_arg;
    (void)event;

    /* Invert the USER LED state */
    cyhal_gpio_toggle(CONNECTION_LED);
}
/* [] END OF FILE */
