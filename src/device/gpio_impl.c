/**
 * @author Cristóvão Zuppardo Rufino <cristovaozr@gmail.com>
 * @version 0.1
 *
 * @copyright Copyright Cristóvão Zuppardo Rufino (c) 2020
 * Please see LICENCE file to information regarding licensing
 */

#include "include/device/gpio.h"

#include "include/errors.h"

#include <stdint.h>

#include "stm32f103xb.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"

#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOC

struct gpio_priv {
    LL_GPIO_InitTypeDef config;
    uint32_t            apb2_grp1_periph;
    GPIO_TypeDef        *gpio;
    uint32_t            default_value;
};

static const struct gpio_priv led_priv = {
    .config = {
        .Pin = LED_Pin,
        .Mode = LL_GPIO_MODE_OUTPUT,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOC,
    .gpio = LED_GPIO_Port,
};

static const struct gpio_priv nrf24l01p_ce_priv = {
    .config = {
        .Pin = LL_GPIO_PIN_8,
        .Mode = LL_GPIO_MODE_OUTPUT,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL
    },
    .apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOA,
    .gpio = GPIOA,
};

static const struct gpio_priv spi1_cs_priv = {
    .config = {.Pin = LL_GPIO_PIN_4, .Mode = LL_GPIO_MODE_OUTPUT, .Speed = LL_GPIO_SPEED_FREQ_LOW, .OutputType = LL_GPIO_OUTPUT_PUSHPULL},
    .apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOA,
    .gpio = GPIOA,
    .default_value = GPIO_HIGH
};

static const struct gpio_priv nrf24l01p_ce2_priv = {
    .config = {
        .Pin = LL_GPIO_PIN_0,
        .Mode = LL_GPIO_MODE_OUTPUT,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL
    },
    .apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOB,
    .gpio = GPIOB,
};

static int32_t stm32f103xb_gpio_init(const struct gpio_device * const gpio);
static void stm32f103xb_gpio_write(const struct gpio_device * const gpio, int32_t value);
static int32_t stm32f103xb_gpio_read(const struct gpio_device * const gpio);
static void stm32f103xb_gpio_toggle(const struct gpio_device * const gpio);

static const struct gpio_operations gpio_ops = {
    .gpio_init = stm32f103xb_gpio_init,
    .gpio_write_op = stm32f103xb_gpio_write,
    .gpio_read_op = stm32f103xb_gpio_read,
    .gpio_toggle_op = stm32f103xb_gpio_toggle
};

const struct gpio_device led_gpio = {
    .ops = &gpio_ops,
    .priv = &led_priv,
};

const struct gpio_device nrf24l01p_ce = {
    .ops = &gpio_ops,
    .priv = &nrf24l01p_ce_priv
};

const struct gpio_device spi1_cs = {
    .ops = &gpio_ops,
    .priv = &spi1_cs_priv
};

const struct gpio_device nrf24l01p_ce2 = {
    .ops = &gpio_ops,
    .priv = &nrf24l01p_ce2_priv
};

static int32_t stm32f103xb_gpio_init(const struct gpio_device * const gpio)
{
    const struct gpio_priv *priv = (const struct gpio_priv *)gpio->priv;
    LL_APB2_GRP1_EnableClock(priv->apb2_grp1_periph);
    LL_GPIO_Init(priv->gpio, (LL_GPIO_InitTypeDef *)&priv->config);

    if (priv->default_value == GPIO_HIGH) LL_GPIO_SetOutputPin(priv->gpio, priv->config.Pin);
    else                                  LL_GPIO_ResetOutputPin(priv->gpio, priv->config.Pin);

    return E_SUCCESS;
}

static void stm32f103xb_gpio_write(const struct gpio_device * const gpio, int32_t value)
{
    const struct gpio_priv *priv = (const struct gpio_priv *)gpio->priv;
    if (value)  LL_GPIO_SetOutputPin(priv->gpio, priv->config.Pin);
    else        LL_GPIO_ResetOutputPin(priv->gpio, priv->config.Pin);
}

static int32_t stm32f103xb_gpio_read(const struct gpio_device * const gpio)
{
    const struct gpio_priv *priv = (const struct gpio_priv *)gpio->priv;
    return LL_GPIO_IsInputPinSet(priv->gpio, priv->config.Pin);
}

static void stm32f103xb_gpio_toggle(const struct gpio_device * const gpio)
{
    const struct gpio_priv *priv = (const struct gpio_priv *)gpio->priv;
    LL_GPIO_TogglePin(priv->gpio, priv->config.Pin);
}