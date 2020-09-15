/**
 * @author Cristóvão Zuppardo Rufino <cristovaozr@gmail.com>
 * @version 0.1
 *
 * @copyright Copyright Cristóvão Zuppardo Rufino (c) 2020
 * Please see LICENCE file to information regarding licensing
 */

#include "include/device/spi.h"

#include <stdint.h>

#include "include/errors.h"
#include "include/utils.h"
#include "include/device/pool_op.h"

#include "stm32f103xb.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_spi.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

struct spi_priv_rtos {
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
    SemaphoreHandle_t mutex;
};

static struct spi_priv_rtos spi1_priv_rtos, spi2_priv_rtos;

struct spi_priv {
    uint32_t irqn;
    SPI_TypeDef *spi;
    GPIO_TypeDef *cs_gpio;
    uint32_t cs_pin;
    struct spi_priv_rtos *priv_rtos;
};

static const struct spi_priv spi1_priv = {
    .irqn = SPI1_IRQn,
    .spi = SPI1,
    .cs_gpio = GPIOA,
    .cs_pin = LL_GPIO_PIN_4,
    .priv_rtos = &spi1_priv_rtos
};

static const struct spi_priv spi2_priv = {
    .irqn = SPI2_IRQn,
    .spi = SPI2,
    .cs_gpio = GPIOB,
    .cs_pin = LL_GPIO_PIN_12,
    .priv_rtos = &spi2_priv_rtos
};

static int32_t stm32f103xb_spi1_init(const struct spi_device * const spi);
static int32_t stm32f103xb_spi2_init(const struct spi_device * const spi);
static int32_t stm32f103xb_spi_write(const struct spi_device * const spi, uint32_t size, const void *data, uint32_t timeout);
static int32_t stm32f103xb_spi_transact(const struct spi_device * const spi, struct transaction * const transaction,
    uint32_t timeout);

static const struct spi_operations spi1_ops = {
    .spi_init = stm32f103xb_spi1_init,
    .spi_write_op = stm32f103xb_spi_write,
    .spi_transact_op = stm32f103xb_spi_transact
};

EXPORTED const struct spi_device spi1 = {
    .ops = &spi1_ops,
    .priv = &spi1_priv
};

static const struct spi_operations spi2_ops = {
    .spi_init = stm32f103xb_spi2_init,
    .spi_write_op = stm32f103xb_spi_write,
    .spi_transact_op = stm32f103xb_spi_transact
};

EXPORTED const struct spi_device spi2 = {
    .ops = &spi2_ops,
    .priv = &spi2_priv
};

// Implementation

static int32_t stm32f103xb_spi1_init(const struct spi_device * const spi)
{
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
    /**SPI1 GPIO Configuration  
    PA4   ------> GPIO CS
    PA5   ------> SPI1_SCK
    PA6   ------> SPI1_MISO
    PA7   ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_5|LL_GPIO_PIN_7;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_6;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_4;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;
    LL_SPI_Init(SPI1, &SPI_InitStruct);
    LL_SPI_Enable(SPI1);

    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_4);

    // FIXME: Not using RTOS for now
    // spi1_priv_rtos.rx_queue = xQueueCreate(32, sizeof(uint8_t));
    // spi1_priv_rtos.tx_queue = xQueueCreate(32, sizeof(uint8_t));

    // NVIC_SetPriority(SPI1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    // NVIC_EnableIRQ(SPI1_IRQn);

    return 0;
}

static int32_t stm32f103xb_spi2_init(const struct spi_device * const spi)
{
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    /**SPI2 GPIO Configuration  
    PB12   ------> GPIO CS
    PB13   ------> SPI2_SCK
    PB14   ------> SPI2_MISO
    PB15   ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_13|LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_FLOATING;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
    SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
    SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
    SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_LOW;
    SPI_InitStruct.ClockPhase = LL_SPI_PHASE_1EDGE;
    SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
    SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV256;
    SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
    SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
    SPI_InitStruct.CRCPoly = 10;
    LL_SPI_Init(SPI2, &SPI_InitStruct);
    LL_SPI_Enable(SPI2);

    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12);

    // FIXME: Not using RTOS for now
    // spi1_priv_rtos.rx_queue = xQueueCreate(32, sizeof(uint8_t));
    // spi1_priv_rtos.tx_queue = xQueueCreate(32, sizeof(uint8_t));

    // NVIC_SetPriority(SPI2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    // NVIC_EnableIRQ(SPI2_IRQn);

    return 0;
}

static int32_t stm32f103xb_spi_write(const struct spi_device * const spi, uint32_t size, const void *data, uint32_t timeout)
{
    (void)timeout;
    int32_t i;
    struct spi_priv *priv = (struct spi_priv *)spi->priv;
    const uint8_t *udata = (const uint8_t *)data;

    if (udata == NULL) {
        i = E_INVALID_PARAMETER;
        goto exit;
    }

    LL_GPIO_ResetOutputPin(priv->cs_gpio, priv->cs_pin);
    for (i = 0; i < size; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(priv->spi));
        LL_SPI_TransmitData8(priv->spi, udata[i]);
    }
    while (LL_SPI_IsActiveFlag_BSY(priv->spi));
    LL_GPIO_SetOutputPin(priv->cs_gpio, priv->cs_pin);

    exit:
    return i;
}

static int32_t stm32f103xb_spi_transact(const struct spi_device * const spi, struct transaction * const transaction,
    uint32_t timeout)
{
    (void)timeout;
    struct spi_priv *priv = (struct spi_priv *)spi->priv;
    const uint8_t *uwrite_data = (const uint8_t *)transaction->write_data;
    uint8_t *uread_data = (uint8_t *)transaction->read_data;
    int32_t ret = E_SUCCESS;

    if (transaction == NULL) {
        ret = E_INVALID_PARAMETER;
        goto exit;
    }

    if (transaction->read_data == NULL || transaction->write_data == NULL) {
        ret = E_INVALID_PARAMETER;
        goto exit;
    }

    // Clears SPI RX buffer
    LL_SPI_ReceiveData8(priv->spi);
    LL_GPIO_ResetOutputPin(priv->cs_gpio, priv->cs_pin);
    for (uint32_t i = 0; i < transaction->transaction_size; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(priv->spi));
        LL_SPI_TransmitData8(priv->spi, uwrite_data[i]);
        while (!LL_SPI_IsActiveFlag_RXNE(priv->spi));
        uread_data[i] = LL_SPI_ReceiveData8(priv->spi);
    }
    while (LL_SPI_IsActiveFlag_BSY(priv->spi));
    LL_GPIO_SetOutputPin(priv->cs_gpio, priv->cs_pin);

    exit:
    return ret;
}

// IRQ Handler FIXME: Unused for now
// static void SPI_IRQHandler(const struct spi_device * const spi)
// {
//     const struct spi_priv * priv = (const struct spi_priv *)spi->priv;
//     struct spi_priv_rtos *priv_rtos = priv->priv_rtos;
//     BaseType_t context_switch;

//     if (LL_SPI_IsActiveFlag_TXE(priv->spi)) {
//         uint8_t byte;
//         while (LL_SPI_IsActiveFlag_TXE(priv->spi)) {
//             if (xQueueReceiveFromISR(priv_rtos->tx_queue, &byte, &context_switch) == pdFAIL) {
//                 LL_SPI_DisableIT_TXE(priv->spi);
//                 break;
//             }
//             LL_SPI_TransmitData8(priv->spi, byte);
//         }
//     }

//     if (LL_SPI_IsActiveFlag_RXNE(priv->spi)) {
//         BaseType_t context_switch;
//         while (LL_SPI_IsActiveFlag_RXNE(priv->spi)) {
//             uint8_t byte = LL_SPI_ReceiveData8(priv->spi);
//             if (xQueueSendFromISR(priv_rtos->rx_queue, &byte, &context_switch) == pdFAIL) {
//                 // TODO: Receive queue is full... what should be done?
//             }
//         }
//     }

//     portYIELD_FROM_ISR(context_switch);
// }

/**
  * @brief This function handles SPI1 global interrupt.
  */
// FIXME: Unused for now
// void SPI1_IRQHandler(void)
// {
//     SPI_IRQHandler(&spi1);
// }
//
// void SPI2_IRQHandler(void)
// {
//     SPI_IRQHandler(&spi2);
// }