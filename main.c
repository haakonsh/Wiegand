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
 * @defgroup nrf_dev_timer_example_main main.c
 * @{
 * @ingroup nrf_dev_timer_example
 * @brief Timer Example Application main file.
 *
 * This file contains the source code for a sample application using Timer0.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"
#include "nrf_delay.h"

#define DATA0_pin   11
#define DATA1_pin   12
#define Tpw         50  //us
#define Tpi         500 //us

const nrf_drv_timer_t TIMER_WIEGAND = NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_timer_t TIMER_WIEGAND_read = NRF_DRV_TIMER_INSTANCE(1);

volatile bool DATA0 = 0;
volatile bool DATA1 = 0;
volatile bool bit_transfer_complete = 0;
uint8_t DATA0_buffer[5] = {170,85,170,85,170};
uint8_t DATA1_buffer[5] = {85,170,85,170,85};

#define GPIO_OUTPUT_PIN_NUMBER BSP_LED_0  /**< Pin number for output. */

static void wiegand_read_init()
{
    uint32_t compare_evt_addr;
    uint32_t gpiote_task_addr;
    nrf_ppi_channel_t ppi_channel;
    ret_code_t err_code;
    nrf_drv_gpiote_out_config_t config = GPIOTE_CONFIG_OUT_TASK_TOGGLE(false);

    err_code = nrf_drv_gpiote_out_init(GPIO_OUTPUT_PIN_NUMBER, &config);
    APP_ERROR_CHECK(err_code);


    nrf_drv_timer_extended_compare(&timer, (nrf_timer_cc_channel_t)0, 200 * 1000UL, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

    err_code = nrf_drv_ppi_channel_alloc(&ppi_channel);
    APP_ERROR_CHECK(err_code);

    compare_evt_addr = nrf_drv_timer_event_address_get(&timer, NRF_TIMER_EVENT_COMPARE0);
    gpiote_task_addr = nrf_drv_gpiote_out_task_addr_get(GPIO_OUTPUT_PIN_NUMBER);

    err_code = nrf_drv_ppi_channel_assign(ppi_channel, compare_evt_addr, gpiote_task_addr);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(ppi_channel);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_task_enable(GPIO_OUTPUT_PIN_NUMBER);
}


void timer_wiegand_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            nrf_gpio_pin_set(DATA0_pin);
            nrf_gpio_pin_set(DATA1_pin);
            break;
        case NRF_TIMER_EVENT_COMPARE1:
            if(DATA0)nrf_gpio_pin_clear(DATA0_pin);
            if(DATA1)nrf_gpio_pin_clear(DATA1_pin);
            break;
        case NRF_TIMER_EVENT_COMPARE2:
            nrf_gpio_pin_set(DATA0_pin);
            nrf_gpio_pin_set(DATA1_pin);
            break;
        case NRF_TIMER_EVENT_COMPARE3:
            bit_transfer_complete = true;
            nrf_drv_timer_clear(&TIMER_WIEGAND);
            break;
        default:
            //Do nothing.
            break;
    }
}

void wiegand_transfer(uint8_t *buffer_0, uint8_t *buffer_1, uint8_t length)
{
    nrf_gpio_pin_clear(DATA0_pin);
    nrf_gpio_pin_clear(DATA1_pin);
    for(uint8_t i = 0 ; i < length ; i++)
    {
        for(uint8_t j = 0 ; j < 8 ; j++)
        {
            DATA0 = ((buffer_0[i] >> j) & 1);
            DATA1 = ((buffer_1[i] >> j) & 1);

            bit_transfer_complete = false;

            nrf_drv_timer_enable(&TIMER_WIEGAND);

            if(DATA0)nrf_gpio_pin_set(DATA0_pin);
            if(DATA1)nrf_gpio_pin_set(DATA1_pin);
            while(!bit_transfer_complete)
            {
                __WFE();
                __WFI();
                __WFE();
            }
        }
    }
}
void timer_init(void)
{
    uint32_t time_us_compare_0 = Tpw;
    uint32_t time_ticks_0;
    uint32_t time_us_compare_1 = Tpw+Tpi;
    uint32_t time_ticks_1;
    uint32_t time_us_compare_2 = (2*Tpw)+Tpi;
    uint32_t time_ticks_2;
    uint32_t time_us_compare_3 = 2*(Tpw+Tpi);
    uint32_t time_ticks_3;

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;


    APP_ERROR_CHECK(nrf_drv_timer_init(&TIMER_WIEGAND, &timer_cfg, timer_wiegand_event_handler));

    time_ticks_0 = nrf_drv_timer_us_to_ticks(&TIMER_WIEGAND, time_us_compare_0);
    time_ticks_1 = nrf_drv_timer_us_to_ticks(&TIMER_WIEGAND, time_us_compare_1);
    time_ticks_2 = nrf_drv_timer_us_to_ticks(&TIMER_WIEGAND, time_us_compare_2);
    time_ticks_3 = nrf_drv_timer_us_to_ticks(&TIMER_WIEGAND, time_us_compare_3);

    // extanded compare stops the timer on compare 3
    nrf_drv_timer_extended_compare(&TIMER_WIEGAND, NRF_TIMER_CC_CHANNEL0, time_ticks_0, NRF_TIMER_SHORT_COMPARE3_STOP_MASK, true);
    nrf_drv_timer_compare(&TIMER_WIEGAND,NRF_TIMER_CC_CHANNEL1,time_ticks_1, true);
    nrf_drv_timer_compare(&TIMER_WIEGAND,NRF_TIMER_CC_CHANNEL2,time_ticks_2, true);
    nrf_drv_timer_compare(&TIMER_WIEGAND,NRF_TIMER_CC_CHANNEL3,time_ticks_3, true);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    uint32_t err_code = NRF_SUCCESS;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    // set up DATA pins
    nrf_gpio_cfg_output(DATA0_pin);
    nrf_gpio_cfg_output(DATA1_pin);

    timer_init();

    while (1)
    {
        wiegand_transfer(DATA0_buffer, DATA1_buffer, 5);
        nrf_delay_ms(100);
    }
}

/** @} */
