#include "include/device/pwm.h"

#include "include/errors.h"

#include "stm32f103xb.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_tim.h"

#include <stdint.h>

struct pwm_priv {
    LL_GPIO_InitTypeDef     pwm_pin_config;
    uint32_t                pwm_pin_apb2_grp1_periph;
    GPIO_TypeDef            *pwm_pin_gpio;
    LL_TIM_InitTypeDef      config;
    LL_TIM_OC_InitTypeDef   oc_config;
    LL_TIM_BDTR_InitTypeDef bdtr_config;
    TIM_TypeDef             *tim;
};

static const struct pwm_priv tim1_priv = {
    .pwm_pin_config = {
        .Pin = LL_GPIO_PIN_8,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_LOW,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .pwm_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOA,
    .pwm_pin_gpio = GPIOA,
    .config = {
        .Prescaler = 40909, // ~440Hz
        .CounterMode = LL_TIM_COUNTERMODE_UP,
        .Autoreload = 0,
        .ClockDivision = LL_TIM_CLOCKDIVISION_DIV4, // 18MHz on TIM1
        .RepetitionCounter = 0,
    },
    .oc_config = {
        .OCMode = LL_TIM_OCMODE_PWM1,
        .OCState = LL_TIM_OCSTATE_ENABLE,
        .OCNState = LL_TIM_OCSTATE_DISABLE,
        .CompareValue = 0,
        .OCPolarity = LL_TIM_OCPOLARITY_HIGH,
        .OCNPolarity = LL_TIM_OCPOLARITY_HIGH,
        .OCIdleState = LL_TIM_OCIDLESTATE_LOW,
        .OCNIdleState = LL_TIM_OCIDLESTATE_LOW,
    },
    .bdtr_config = {
        .OSSRState = LL_TIM_OSSR_DISABLE,
        .OSSIState = LL_TIM_OSSI_DISABLE,
        .LockLevel = LL_TIM_LOCKLEVEL_OFF,
        .DeadTime = 0,
        .BreakState = LL_TIM_BREAK_DISABLE,
        .BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH,
        .AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE,
    },
    .tim = TIM1,
};

static int32_t pwm_tim1_init(const struct pwm_device * const pwm);
static int32_t pwm_tim1_set_duty(const struct pwm_device * const pwm, uint32_t duty);
static int32_t pwm_tim1_set_frequency(const struct pwm_device * const pwm, uint32_t freq);

static const struct pwm_operations pwm_tim1_ops = {
    .pwm_init = pwm_tim1_init,
    .pwm_set_duty = pwm_tim1_set_duty,
    .pwm_set_frequency = pwm_tim1_set_frequency,
};

const struct pwm_device pwm_tim1 = {
    .ops = &pwm_tim1_ops,
    .priv = &tim1_priv,
};

static int32_t pwm_tim1_init(const struct pwm_device * const pwm)
{
    const struct pwm_priv *priv = (const struct pwm_priv *)pwm->priv;

    // GPIO for PWM output
    LL_APB2_GRP1_EnableClock(priv->pwm_pin_apb2_grp1_periph);
    LL_GPIO_Init(priv->pwm_pin_gpio, (LL_GPIO_InitTypeDef *)&priv->pwm_pin_config);

    // TIM1 related stuff
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_TIM_Init(priv->tim, (LL_TIM_InitTypeDef *)&priv->config);
    LL_TIM_DisableARRPreload(priv->tim);
    LL_TIM_SetClockSource(priv->tim, LL_TIM_CLOCKSOURCE_INTERNAL);

    LL_TIM_OC_EnablePreload(priv->tim, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_Init(priv->tim, LL_TIM_CHANNEL_CH1, (LL_TIM_OC_InitTypeDef *)&priv->oc_config);
    LL_TIM_OC_DisableFast(priv->tim, LL_TIM_CHANNEL_CH1);
    LL_TIM_SetTriggerOutput(priv->tim, LL_TIM_TRGO_RESET);
    LL_TIM_DisableMasterSlaveMode(priv->tim);

    LL_TIM_BDTR_Init(priv->tim, (LL_TIM_BDTR_InitTypeDef *)&priv->bdtr_config);

    LL_TIM_EnableCounter(priv->tim);

    return E_SUCCESS;
}

static int32_t pwm_tim1_set_duty(const struct pwm_device * const pwm, uint32_t duty)
{
    int ret = E_SUCCESS;
    const struct pwm_priv *priv = (const struct pwm_priv *)pwm->priv;

    if (duty > 65535) {
        ret = E_INVALID_DUTY_CYCLE;
        goto exit;
    }

    LL_TIM_OC_SetCompareCH1(priv->tim, duty);

    exit:
    return ret;
}

static int32_t pwm_tim1_set_frequency(const struct pwm_device * const pwm, uint32_t freq)
{
    int ret = E_SUCCESS;
    const struct pwm_priv *priv = (const struct pwm_priv *)pwm->priv;

    if (freq > 4000) {
        ret = E_PWM_FREQ_TOO_HIGH;
        goto exit;
    } else if (freq < 275) {
        ret = E_PWM_FREQ_TOO_LOW;
        goto exit;
    }

    int32_t psc = 18000000 / freq;
    LL_TIM_SetPrescaler(priv->tim, psc);

    exit:
    return ret;
}
