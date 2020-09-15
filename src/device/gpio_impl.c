/**
 * @author Cristóvão Zuppardo Rufino <cristovaozr@gmail.com>
 * @version 0.1
 *
 * @copyright Copyright Cristóvão Zuppardo Rufino (c) 2020
 * Please see LICENCE file to information regarding licensing
 */

#include "include/device/gpio.h"

#include <stdint.h>

#include "include/utils.h"
#include "stm32f103xb.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"

#define LED_Pin LL_GPIO_PIN_13
#define LED_GPIO_Port GPIOC

struct gpio_priv {
    LL_GPIO_InitTypeDef config;
    uint32_t            apb2_grp1_periph;
    GPIO_TypeDef        *gpio;
    uint32_t            pin;
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
    .pin = LED_Pin,
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
    .pin = LL_GPIO_PIN_8
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

EXPORTED const struct gpio_device led_gpio = {
    .ops = &gpio_ops,
    .priv = &led_priv,
};

EXPORTED const struct gpio_device nrf24l01p_ce = {
    .ops = &gpio_ops,
    .priv = &nrf24l01p_ce_priv
};

// Implementation

static int32_t stm32f103xb_gpio_init(const struct gpio_device * const gpio)
{
    const struct gpio_priv *priv = (const struct gpio_priv *)gpio->priv;
    LL_APB2_GRP1_EnableClock(priv->apb2_grp1_periph);
    LL_GPIO_Init(priv->gpio, (LL_GPIO_InitTypeDef *)&priv->config);
    return 0;

    // TODO: Old stuff here to remember what else had here. Should be erased in the future
    /* GPIO Ports Clock Enable */
    // LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);
    // LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    // LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    // LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin);
}

static void stm32f103xb_gpio_write(const struct gpio_device * const gpio, int32_t value)
{
    const struct gpio_priv *priv = (const struct gpio_priv *)gpio->priv;
    if (value)  LL_GPIO_SetOutputPin(priv->gpio, priv->pin);
    else        LL_GPIO_ResetOutputPin(priv->gpio, priv->pin);
}

static int32_t stm32f103xb_gpio_read(const struct gpio_device * const gpio)
{
    const struct gpio_priv *priv = (const struct gpio_priv *)gpio->priv;
    return LL_GPIO_IsInputPinSet(priv->gpio, priv->pin);
}

static void stm32f103xb_gpio_toggle(const struct gpio_device * const gpio)
{
    const struct gpio_priv *priv = (const struct gpio_priv *)gpio->priv;
    LL_GPIO_TogglePin(priv->gpio, priv->pin);
}