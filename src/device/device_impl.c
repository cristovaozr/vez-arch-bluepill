/**
 * @author Cristóvão Zuppardo Rufino <cristovaozr@gmail.com>
 * @version 0.1
 *
 * @copyright Copyright Cristóvão Zuppardo Rufino (c) 2020-2021
 * Please see LICENCE file to information regarding licensing
 */

#include "include/device/device.h"

#include "include/device/gpio.h"
#include "include/device/usart.h"
#include "include/device/i2c.h"
#include "include/device/cpu.h"

#include "ulibc/include/utils.h"

#include <string.h>

extern const struct gpio_device led_gpio;
extern const struct gpio_device nrf24l01p_ce;
extern const struct gpio_device nrf24l01p_ce2;
extern const struct gpio_device spi1_cs;
extern const struct usart_device usart1;
extern const struct usart_device usart2;
extern const struct usart_device usart3;
extern const struct spi_device spi1;
extern const struct spi_device spi2;
extern const struct i2c_device i2c1;
extern const struct cpu stm32f103xb_cpu;

struct device_tree {
    const char *name;
    const void *device;
};

static const struct device_tree tree[8] = {
    {DEFAULT_CPU,   &stm32f103xb_cpu},
    {DEFAULT_USART, &usart2},
    {DEFAULT_LED,   &led_gpio},
    {"spi1",        &spi1},
    {"i2c1",        &i2c1},
    {"spi1_cs",     &spi1_cs},
    {"usart1",      &usart1},
    {"nrf24l01p_ce", &nrf24l01p_ce}
};

const void *device_get_by_name(const char *dev_name)
{
    for(int i = 0; i < ARRAY_SIZE(tree); i++) {
        if (strcmp(tree[i].name, dev_name) == 0) return tree[i].device;
    }

    return NULL;
}