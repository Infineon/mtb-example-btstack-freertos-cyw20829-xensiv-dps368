/*******************************************************************************
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
#include "cyhal.h"
#include "sensor_task.h"
#include "cy_retarget_io.h"
#include "timers.h"
#include "xensiv_dps3xx_mtb.h"
#include "app_bt_gatt_handler.h"

/*******************************************************************************
 *        Macro Definitions
 *******************************************************************************/
#define POLL_TIMER_IN_MSEC              (1000u)
#define POLL_TIMER_FREQ                 (10000)

/* I2C Clock frequency in Hz */
#define I2C_CLK_FREQ_HZ                 (400000U)

#define PRESSURE_START_INDEX            (0)
#define TEMPERATURE_START_INDEX         (4)

/* Check if notification is enabled for a valid connection ID */
#define IS_NOTIFIABLE(conn_id, cccd)     (((conn_id)!= 0)? (cccd) & GATT_CLIENT_CONFIG_NOTIFICATION: 0)

/*******************************************************************************
 *        Variable Definitions
 *******************************************************************************/
static cyhal_i2c_t i2c_obj;
static xensiv_dps3xx_t pressure_sensor;

/* Handle to timer object */
TimerHandle_t timer_handle;

/******************************************************************************
 *                          Function Prototypes
 ******************************************************************************/
void timer_callback(TimerHandle_t xTimer);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
int32_t sensor_task_init()
{
    int32_t result;

    /* Create timer */
    timer_handle = xTimerCreate("timer",
                                pdMS_TO_TICKS(POLL_TIMER_IN_MSEC),
                                pdTRUE,
                                NULL,
                                timer_callback);
    if (NULL == timer_handle)
    {
        return -1;
    }

    /* Start Timer */
    xTimerStart(timer_handle, 0);

    /* Initialize I2C for pressure sensor */
    cyhal_i2c_cfg_t i2c_cfg = { false, 0, I2C_CLK_FREQ_HZ };

    result = cyhal_i2c_init(&i2c_obj, CYBSP_I2C_SDA, CYBSP_I2C_SCL, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("I2C initialization failed!\r\n");
        CY_ASSERT(0);
    }
    result = cyhal_i2c_configure(&i2c_obj, &i2c_cfg);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("I2C configuration failed!\r\n");
        CY_ASSERT(0);
    }

    /* Initialize pressure sensor */
    result = xensiv_dps3xx_mtb_init_i2c(&pressure_sensor, &i2c_obj, XENSIV_DPS3XX_I2C_ADDR_ALT);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("DPS368 initialization failed!\r\n");
        CY_ASSERT(0);
    }

    return 0;
}

void sensor_task(void *pvParameters)
{
    cy_rslt_t result;

    bool pressure_ready;
    bool temperature_ready;

    float pressure = 0.0f;
    float temperature = 0.0f;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        result = xensiv_dps3xx_check_ready(&pressure_sensor, &pressure_ready, &temperature_ready);
        if (CY_RSLT_SUCCESS != result)
        {
            printf("Error checking if DPS368 is ready\r\n");
            continue;
        }

        if (!pressure_ready && !temperature_ready)
        {
            /* Data not ready, re-check ready flag */
            continue;
        }

        result = xensiv_dps3xx_read(&pressure_sensor, &pressure, &temperature);
        if (CY_RSLT_SUCCESS != result)
        {
            printf("Error reading from DPS368\r\n");
            continue;
        }

        printf("Pressure   : %.2f hPa\r\n", pressure);
        printf("Temperature: %.2f degC\r\n\n", temperature);

        /* Copy values to BLE buffer */
        memcpy(&app_xensiv_sensor_shield_dps368[PRESSURE_START_INDEX], &pressure, sizeof(pressure));
        memcpy(&app_xensiv_sensor_shield_dps368[TEMPERATURE_START_INDEX], &temperature, sizeof(temperature));

        if (IS_NOTIFIABLE (app_bt_conn_id, app_xensiv_sensor_shield_dps368_client_char_config[0]) == 0)
        {
            if(!app_bt_conn_id)
            {
                printf("This device is not connected to a central device\n");
            }else{
                printf("This device is connected to a central device but\n"
                        "GATT client notifications are not enabled\n");
            }
        }
        else
        {
            wiced_bt_gatt_status_t gatt_status;

            /*
            * Sending notification, set the pv_app_context to NULL, since the
            * data 'app_xensiv_sensor_shield_dps368' is not to be freed
            */
            gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                 HDLC_XENSIV_SENSOR_SHIELD_DPS368_VALUE,
                                                                 app_xensiv_sensor_shield_dps368_len,
                                                                 (uint8_t *) app_xensiv_sensor_shield_dps368,
                                                                 NULL);

            printf("Sent notification status 0x%x\n", gatt_status);
        }
    }
}

void timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;

    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
