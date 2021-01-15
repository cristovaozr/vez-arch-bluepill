#include "include/device/device.h"

#include "include/device/gpio.h"
#include "include/device/usart.h"
#include "include/device/i2c.h"

#include "ulibc/include/utils.h"

#include <string.h>

extern const struct gpio_device led_gpio;
extern const struct gpio_device nrf24l01p_ce;
extern const struct gpio_device nrf24l01p_ce2;
extern const struct usart_device usart1;
extern const struct usart_device usart2;
extern const struct usart_device usart3;
extern const struct spi_device spi1;
extern const struct spi_device spi2;

struct device_tree {
    const char *name;
    const void *device;
};

static struct device_tree tree[5] = {
    {DEFAULT_USART, &usart2},
    {DEFAULT_LED,   &led_gpio},
    {"spi1",        &spi1},
    {"spi2",        &spi2}
};

const void *device_get_by_name(const char *dev_name)
{
    for(int i = 0; i < ARRAY_SIZE(tree); i++) {
        if (strcmp(tree[i].name, dev_name) == 0) return tree[i].device;
    }

    return NULL;
}