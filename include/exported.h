/**
 * @author Cristóvão Zuppardo Rufino <cristovaozr@gmail.com>
 * @version 0.1
 *
 * @copyright Copyright Cristóvão Zuppardo Rufino (c) 2020
 * Please see LICENCE file to information regarding licensing
 */

#ifndef ARCH_STM32F103XB_EXPORTED_H_
#define ARCH_STM32F103XB_EXPORTED_H_

#include "include/device/gpio.h"
#include "include/device/usart.h"
#include "include/device/spi.h"

// Available at gpio_impl.c
extern const struct gpio_device led_gpio;
extern const struct gpio_device nrf24l01p_ce;
extern const struct gpio_device nrf24l01p_ce2;

// Available at usart_impl.c
extern const struct usart_device usart1;
extern const struct usart_device usart2;
extern const struct usart_device usart3;

// Available at spi_impl.c
extern const struct spi_device spi1;
extern const struct spi_device spi2;

// Available at pwm_impl.c
extern const struct pwm_device pwm_tim1;

#endif // ARCH_STM32F103XB_EXPORTED_H_