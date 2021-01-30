/**
 * @author Cristóvão Zuppardo Rufino <cristovaozr@gmail.com>
 * @version 0.1
 *
 * @copyright Copyright Cristóvão Zuppardo Rufino (c) 2020
 * Please see LICENCE file to information regarding licensing
 */

#include "stm32f103xb.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_utils.h"

#include "include/device/device.h"

#include "include/errors.h"

#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0    ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority, 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1    ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority, 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2    ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority, 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3    ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority, 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4    ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority, 0 bit  for subpriority */
#endif

static void SystemClock_Config(void);

int32_t hw_init_early_config(void)
{
    NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
    NVIC_SetPriority(PendSV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 15, 0));
    return E_SUCCESS;
}

int32_t hw_init(void)
{
    SystemClock_Config();
    return E_SUCCESS;
}

extern const struct gpio_device led_gpio;
extern const struct gpio_device nrf24l01p_ce;
extern const struct gpio_device nrf24l01p_ce2;
extern const struct usart_device usart1;
extern const struct usart_device usart2;
extern const struct usart_device usart3;
extern const struct spi_device spi1;
extern const struct spi_device spi2;
extern const struct i2c_device i2c1;

// Available at pwm_impl.c
extern const struct pwm_device pwm_tim1;

int32_t hw_init_late_config(void)
{
    int32_t ret;
    if ((ret = device_init(&led_gpio)) != E_SUCCESS) goto exit;
    if ((ret = device_init(&usart2)) != E_SUCCESS) goto exit;
    if ((ret = device_init(&spi1)) != E_SUCCESS) goto exit;
    if ((ret = device_init(&i2c1)) != E_SUCCESS) goto exit;
    // if ((ret = device_init(&spi2)) != E_SUCCESS) goto exit;
    // if ((ret = device_init(&nrf24l01p_ce)) != E_SUCCESS) goto exit;
    // if ((ret = device_init(&nrf24l01p_ce2)) != E_SUCCESS) goto exit;
    // device_init(&usart1);
    // device_init(&usart3);
    // device_init(&pwm_tim1);

    /* Initialize all configured peripherals */
    // MX_ADC1_Init();
    // MX_I2C1_Init();
    // MX_RTC_Init();

    exit:
    return ret;
}

/**
* @brief System Clock Configuration
*/
static void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

    if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
        while(1);

    LL_RCC_HSE_Enable();
    /* Wait till HSE is ready */
    while(LL_RCC_HSE_IsReady() != 1);

    LL_PWR_EnableBkUpAccess();
    LL_RCC_ForceBackupDomainReset();
    LL_RCC_ReleaseBackupDomainReset();
    LL_RCC_LSE_Enable();
    // /* Wait till LSE is ready */
    // FIXME: Delaying (or hanging) due to LSE not ready
    // It is unkown why the LSE is not stabilizing therefore the loop that waits for stablization
    // is disabled for now.
    // while(LL_RCC_LSE_IsReady() != 1);

    LL_RCC_SetRTCClockSource(LL_RCC_RTC_CLKSOURCE_LSE);
    LL_RCC_EnableRTC();
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSE_DIV_1, LL_RCC_PLL_MUL_9);
    LL_RCC_PLL_Enable();
    /* Wait till PLL is ready */
    while(LL_RCC_PLL_IsReady() != 1);

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
    LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    /* Wait till System clock is ready */
    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    LL_SetSystemCoreClock(72000000);
    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSRC_PCLK2_DIV_6);
}
