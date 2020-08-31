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

static struct spi_priv_rtos spi1_priv_rtos;

struct spi_priv {
    // SCK
    LL_GPIO_InitTypeDef sck_pin_config;
    uint32_t            sck_pin_apb2_grp1_periph;
    GPIO_TypeDef        *sck_pin_gpio;
    // CIPO
    LL_GPIO_InitTypeDef cipo_pin_config;
    uint32_t            cipo_pin_apb2_grp1_periph;
    GPIO_TypeDef        *cipo_pin_gpio;
    // COPI
    LL_GPIO_InitTypeDef copi_pin_config;
    uint32_t            copi_pin_apb2_grp1_periph;
    GPIO_TypeDef        *copi_pin_gpio;
    // SPI
    LL_SPI_InitTypeDef config;
    uint32_t apb2_grp1_periph;
    uint32_t irqn;
    SPI_TypeDef *spi;
    struct spi_priv_rtos *priv_rtos;
};

static const struct spi_priv spi1_priv = {
    .sck_pin_config = {
        .Pin = LL_GPIO_PIN_3,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .sck_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOB,
    .sck_pin_gpio = GPIOB,

    .cipo_pin_config = {
        .Pin = LL_GPIO_PIN_4,
        .Mode = LL_GPIO_MODE_FLOATING,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .cipo_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOB,
    .cipo_pin_gpio = GPIOB,

    .copi_pin_config = {
        .Pin = LL_GPIO_PIN_5,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .copi_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOB,
    .copi_pin_gpio = GPIOB,

    .config = {
        .TransferDirection = LL_SPI_FULL_DUPLEX,
        .Mode = LL_SPI_MODE_MASTER,
        .DataWidth = LL_SPI_DATAWIDTH_8BIT,
        .ClockPolarity = LL_SPI_POLARITY_LOW,
        .ClockPhase = LL_SPI_PHASE_1EDGE,
        .NSS = LL_SPI_NSS_HARD_OUTPUT,
        .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV32,
        .BitOrder = LL_SPI_MSB_FIRST,
        .CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
        .CRCPoly = 10,
    },
    .apb2_grp1_periph = LL_APB2_GRP1_PERIPH_SPI1,
    .irqn = SPI1_IRQn,
    .spi = SPI1,
    .priv_rtos = &spi1_priv_rtos
};

static int32_t stm32f103xb_spi_1_init(const struct spi_device * const spi);
static int32_t stm32f103xb_spi_write(const struct spi_device * const spi, const void *data, uint32_t size,
    uint32_t timeout);
static int32_t stm32f103xb_spi_read(const struct spi_device * const spi, void *data, uint32_t size, uint32_t timeout);
static int32_t stm32f103xb_spi_transact(const struct spi_device * const spi, struct transaction * const transaction,
    uint32_t timeout);

static const struct spi_operations spi1_ops = {
    .spi_init = stm32f103xb_spi_1_init,
    .spi_write_op = stm32f103xb_spi_write,
    .spi_read_op = stm32f103xb_spi_read,
    .spi_transact_op = stm32f103xb_spi_transact
};

EXPORTED const struct spi_device spi1 = {
    .ops = &spi1_ops,
    .priv = &spi1_priv
};

// Implementation

static int32_t stm32f103xb_spi_1_init(const struct spi_device * const spi)
{
    const struct spi_priv *priv = (const struct spi_priv *)spi->priv;

    // SCK
    LL_APB2_GRP1_EnableClock(priv->sck_pin_apb2_grp1_periph);
    LL_GPIO_Init(priv->sck_pin_gpio, &priv->sck_pin_config);
    // CIPO
    LL_APB2_GRP1_EnableClock(priv->cipo_pin_apb2_grp1_periph);
    LL_GPIO_Init(priv->cipo_pin_gpio, &priv->cipo_pin_config);
    // COPI
    LL_APB2_GRP1_EnableClock(priv->copi_pin_apb2_grp1_periph);
    LL_GPIO_Init(priv->copi_pin_gpio, &priv->copi_pin_config);
    // SPI
    LL_APB2_GRP1_EnableClock(priv->apb2_grp1_periph);
    LL_GPIO_AF_EnableRemap_SPI1();
    LL_SPI_Init(priv->spi, &priv->config);

    // FIXME: Not using RTOS for now
    // spi1_priv_rtos.rx_queue = xQueueCreate(32, sizeof(uint8_t));
    // spi1_priv_rtos.tx_queue = xQueueCreate(32, sizeof(uint8_t));
    
    // NVIC_SetPriority(priv->irqn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    // NVIC_EnableIRQ(priv->irqn);

    return 0;
}

static int32_t stm32f103xb_spi_write(const struct spi_device * const spi, const void *data, uint32_t size, uint32_t timeout)
{
    (void)timeout;
    int32_t i;
    struct spi_priv *priv = (struct spi_priv *)spi->priv;
    const uint8_t *udata = (const uint8_t *)data;

    for (i = 0; i < size; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(priv->spi));
        LL_SPI_TransmitData8(priv->spi, udata[i]);
    }
    while (LL_SPI_IsActiveFlag_BSY(priv->spi));

    return i;
}

static int32_t stm32f103xb_spi_read(const struct spi_device * const spi, void *data, uint32_t size, uint32_t timeout)
{
    (void)timeout;
    int32_t i;
    struct spi_priv *priv = (struct spi_priv *)spi->priv;
    uint8_t *udata = (uint8_t *)data;

    for (i = 0; i < size; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(priv->spi));
        LL_SPI_TransmitData8(priv->spi, 0xff);
        while (!LL_SPI_IsActiveFlag_RXNE(priv->spi));
        udata[i] = LL_SPI_ReceiveData8(priv->spi);
    }
    while (LL_SPI_IsActiveFlag_BSY(priv->spi));

    return i;
}

static int32_t stm32f103xb_spi_transact(const struct spi_device * const spi, struct transaction * const transaction,
    uint32_t timeout)
{
    (void)timeout;
    struct spi_priv *priv = (struct spi_priv *)spi->priv;
    const uint8_t *uwrite_data = (const uint8_t *)transaction->write_data;
    uint8_t *uread_data = (uint8_t *)transaction->read_data;
    for (uint32_t i = 0; i < transaction->transaction_size; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(priv->spi));
        LL_SPI_TransmitData8(priv->spi, uwrite_data[i]);
        while (!LL_SPI_IsActiveFlag_RXNE(priv->spi));
        uread_data[i] = LL_SPI_ReceiveData8(priv->spi);
    }
    while (LL_SPI_IsActiveFlag_BSY(priv->spi));

    return E_SUCCESS;
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