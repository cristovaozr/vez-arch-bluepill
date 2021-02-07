/**
 * @author Cristóvão Zuppardo Rufino <cristovaozr@gmail.com>
 * @version 0.1
 *
 * @copyright Copyright Cristóvão Zuppardo Rufino (c) 2020
 * Please see LICENCE file to information regarding licensing
 */

#include "include/device/spi.h"

#include <stdint.h>
#include <stddef.h>

#include "include/errors.h"
#include "include/device/pool_op.h"
#include "ulibc/include/utils.h"

#include "stm32f103xb.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_spi.h"

struct spi_priv {
    SPI_TypeDef *spi;
};

static const struct spi_priv spi1_priv = {
    .spi = SPI1,
};

static const struct spi_priv spi2_priv = {
    .spi = SPI2,
};

static int32_t stm32f103xb_spi1_init(const struct spi_device * const spi);
static int32_t stm32f103xb_spi2_init(const struct spi_device * const spi);
static int32_t stm32f103xb_spi_write(const struct spi_device * const spi, const void *data, uint32_t size, uint32_t timeout);
static int32_t stm32f103xb_spi_read(const struct spi_device * const spi, void *data, uint32_t size, uint32_t timeout);
static int32_t stm32f103xb_spi_transact(const struct spi_device * const spi, struct spi_transaction * const transaction,
    uint32_t timeout);

static const struct spi_operations spi1_ops = {
    .spi_init = stm32f103xb_spi1_init,
    .spi_write_op = stm32f103xb_spi_write,
    .spi_read_op = stm32f103xb_spi_read,
    .spi_transact_op = stm32f103xb_spi_transact
};

const struct spi_device spi1 = {
    .ops = &spi1_ops,
    .priv = &spi1_priv
};

static const struct spi_operations spi2_ops = {
    .spi_init = stm32f103xb_spi2_init,
    .spi_write_op = stm32f103xb_spi_write,
    .spi_read_op = stm32f103xb_spi_read,
    .spi_transact_op = stm32f103xb_spi_transact
};

const struct spi_device spi2 = {
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

    return E_SUCCESS;
}

static int32_t stm32f103xb_spi2_init(const struct spi_device * const spi)
{
    LL_SPI_InitTypeDef SPI_InitStruct = {0};
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);

    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
    /**SPI2 GPIO Configuration  
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

    return E_SUCCESS;
}

static int32_t stm32f103xb_spi_write(const struct spi_device * const spi, const void *data, uint32_t size, uint32_t timeout)
{
    (void)timeout;
    int32_t i;
    const struct spi_priv *priv = (const struct spi_priv *)spi->priv;
    const uint8_t *udata = (const uint8_t *)data;

    if (udata == NULL) {
        i = E_INVALID_PARAMETER;
        goto exit;
    }

    for (i = 0; i < size; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(priv->spi));
        LL_SPI_TransmitData8(priv->spi, udata[i]);
    }
    while (LL_SPI_IsActiveFlag_BSY(priv->spi));

    exit:
    return i;
}

static int32_t stm32f103xb_spi_read(const struct spi_device * spi, void *data, uint32_t size, uint32_t timeout)
{
    (void)timeout;
    int32_t i;
    const struct spi_priv *priv = (const struct spi_priv *)spi->priv;
    uint8_t *udata = (uint8_t *)data;

    if (udata == NULL) {
        i = E_INVALID_PARAMETER;
        goto exit;
    }

    // Clears the RX buffer
    LL_SPI_ReceiveData8(priv->spi);
    for (i = 0; i < size; i++) {
        while (LL_SPI_IsActiveFlag_TXE(priv->spi) == 0);
        LL_SPI_TransmitData8(priv->spi, 0xff);
        while (LL_SPI_IsActiveFlag_RXNE(priv->spi) == 0);
        udata[i] = LL_SPI_ReceiveData8(priv->spi);
    }
    while (LL_SPI_IsActiveFlag_BSY(priv->spi));

    exit:
    return i;
}

static int32_t stm32f103xb_spi_transact(const struct spi_device * const spi, struct spi_transaction * const transaction,
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
    for (uint32_t i = 0, j = 0; i < transaction->write_size || j < transaction->read_size; i++) {
        while (!LL_SPI_IsActiveFlag_TXE(priv->spi));
        LL_SPI_TransmitData8(priv->spi, uwrite_data[i]);
        if (j < transaction->read_size) {
            while (!LL_SPI_IsActiveFlag_RXNE(priv->spi));
            uread_data[j++] = LL_SPI_ReceiveData8(priv->spi);
        }

    }
    while (LL_SPI_IsActiveFlag_BSY(priv->spi));

    exit:
    return ret;
}
