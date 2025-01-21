/**
 * @author Cristóvão Zuppardo Rufino <cristovaozr@gmail.com>
 * @version 0.1
 *
 * @copyright Copyright Cristóvão Zuppardo Rufino (c) 2020-2021
 * Please see LICENCE file to information regarding licensing
 */

#include "include/device/usart.h"

#include <stdint.h>

#include "include/errors.h"
#include "include/device/pool_op.h"

#include "stm32f103xb.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_usart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Number of USARTs available
#define AVAILABLE_USARTS    3

struct usart_priv {
    // TX pin
    LL_GPIO_InitTypeDef tx_pin_config;
    uint32_t            tx_pin_apb2_grp1_periph;
    GPIO_TypeDef        *tx_pin_gpio;
    // RX pin
    LL_GPIO_InitTypeDef rx_pin_config;
    uint32_t            rx_pin_apb2_grp1_periph;
    GPIO_TypeDef        *rx_pin_gpio;
    // USART
    LL_USART_InitTypeDef config;
    uint32_t apb2_grp1_periph;
    uint32_t irqn;
    USART_TypeDef *usart;
    uint32_t index;
};

// Special structure to map RAM stuff for each usart_priv
struct usart_priv_rtos {
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
    SemaphoreHandle_t mutex;
};

static const struct usart_priv usart1_priv = {
    .tx_pin_config = {
        .Pin = LL_GPIO_PIN_9,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .tx_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOA,
    .tx_pin_gpio = GPIOA,

    .rx_pin_config = {
        .Pin = LL_GPIO_PIN_10,
        .Mode = LL_GPIO_MODE_FLOATING,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .rx_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOA,
    .rx_pin_gpio = GPIOA,

    .config = {
        .BaudRate = 115200,
        .DataWidth = LL_USART_DATAWIDTH_8B,
        .StopBits = LL_USART_STOPBITS_1,
        .Parity = LL_USART_PARITY_NONE,
        .TransferDirection = LL_USART_DIRECTION_TX_RX,
        .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
        .OverSampling = LL_USART_OVERSAMPLING_16,
    },
    .apb2_grp1_periph = LL_APB2_GRP1_PERIPH_USART1,
    .irqn = USART1_IRQn,
    .usart = USART1,
    .index = 0
};

static const struct usart_priv usart2_priv = {
    .tx_pin_config = {
        .Pin = LL_GPIO_PIN_2,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .tx_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOA,
    .tx_pin_gpio = GPIOA,

    .rx_pin_config = {
        .Pin = LL_GPIO_PIN_3,
        .Mode = LL_GPIO_MODE_FLOATING,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .rx_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOA,
    .rx_pin_gpio = GPIOA,

    .config = {
        .BaudRate = 115200,
        .DataWidth = LL_USART_DATAWIDTH_8B,
        .StopBits = LL_USART_STOPBITS_1,
        .Parity = LL_USART_PARITY_NONE,
        .TransferDirection = LL_USART_DIRECTION_TX_RX,
        .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
        .OverSampling = LL_USART_OVERSAMPLING_16,
    },
    .apb2_grp1_periph = LL_APB1_GRP1_PERIPH_USART2,
    .irqn = USART2_IRQn,
    .usart = USART2,
    .index = 1
};

static const struct usart_priv usart3_priv = {
    .tx_pin_config = {
        .Pin = LL_GPIO_PIN_10,
        .Mode = LL_GPIO_MODE_ALTERNATE,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .tx_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOB,
    .tx_pin_gpio = GPIOB,

    .rx_pin_config = {
        .Pin = LL_GPIO_PIN_11,
        .Mode = LL_GPIO_MODE_FLOATING,
        .Speed = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
    },
    .rx_pin_apb2_grp1_periph = LL_APB2_GRP1_PERIPH_GPIOB,
    .rx_pin_gpio = GPIOB,

    .config = {
        .BaudRate = 115200,
        .DataWidth = LL_USART_DATAWIDTH_8B,
        .StopBits = LL_USART_STOPBITS_1,
        .Parity = LL_USART_PARITY_NONE,
        .TransferDirection = LL_USART_DIRECTION_TX_RX,
        .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
        .OverSampling = LL_USART_OVERSAMPLING_16,
    },
    .apb2_grp1_periph = LL_APB1_GRP1_PERIPH_USART3,
    .irqn = USART3_IRQn,
    .usart = USART3,
    .index = 2
};

static int32_t stm32f103xb_usart_2_3_init(const struct usart_device * const usart);
static int32_t stm32f103xb_usart_1_init(const struct usart_device * const usart);
static int32_t stm32f103xb_usart_write(const struct usart_device * const usart, const void *data, uint32_t size,
    uint32_t timeout);
static int32_t stm32f103xb_usart_read(const struct usart_device * const usart, void *data, uint32_t size,
    uint32_t timeout);
static int32_t stm32f103xb_usart_poll(const struct usart_device * const usart, enum poll_op op, void *answer);

static const struct usart_operations usart1_ops = {
    .usart_init = stm32f103xb_usart_1_init,
    .usart_write_op = stm32f103xb_usart_write,
    .usart_read_op = stm32f103xb_usart_read,
    .usart_poll_op = stm32f103xb_usart_poll,
};

static const struct usart_operations usart_2_3_ops = {
    .usart_init = stm32f103xb_usart_2_3_init,
    .usart_write_op = stm32f103xb_usart_write,
    .usart_read_op = stm32f103xb_usart_read,
    .usart_poll_op = stm32f103xb_usart_poll,
};

/*
 * This array contains all FreeRTOS stuff because they cannot be inside the priv structure. If they
 * were there they would **not** be writable.
 */
static struct usart_priv_rtos priv_rtos[AVAILABLE_USARTS];

const struct usart_device usart1 = {
    .ops = &usart1_ops,
    .priv = &usart1_priv,
};

const struct usart_device usart2 = {
    .ops = &usart_2_3_ops,
    .priv = &usart2_priv,
};

const struct usart_device usart3 = {
    .ops = &usart_2_3_ops,
    .priv = &usart3_priv,
};

// Implementation

static int32_t stm32f103xb_usart_2_3_init(const struct usart_device * const usart)
{
    const struct usart_priv *priv = (const struct usart_priv *)usart->priv;

    LL_APB1_GRP1_EnableClock(priv->apb2_grp1_periph); // Enables USART clock
    // TX pin
    LL_APB2_GRP1_EnableClock(priv->tx_pin_apb2_grp1_periph);
    LL_GPIO_Init(priv->tx_pin_gpio, (LL_GPIO_InitTypeDef *)&priv->tx_pin_config);
    // RX pin
    LL_APB2_GRP1_EnableClock(priv->rx_pin_apb2_grp1_periph);
    LL_GPIO_Init(priv->rx_pin_gpio, (LL_GPIO_InitTypeDef *)&priv->rx_pin_config);
    // USART
    LL_USART_Init(priv->usart, (LL_USART_InitTypeDef *)&priv->config);
    LL_USART_ConfigAsyncMode(priv->usart);
    LL_USART_Enable(priv->usart);

    // Enables FreeRTOS queues and mutexes
    priv_rtos[priv->index].tx_queue = xQueueCreate(64, sizeof(uint8_t));
    priv_rtos[priv->index].rx_queue = xQueueCreate(64, sizeof(uint8_t));
    priv_rtos[priv->index].mutex = xSemaphoreCreateMutex();

    // Enables IRQ for USART
    LL_USART_EnableIT_RXNE(priv->usart);
    NVIC_SetPriority(priv->irqn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(priv->irqn);

    return 0;
}

// This has to be different because the EnableClock for USART1 is different from USART2 and USART3
static int32_t stm32f103xb_usart_1_init(const struct usart_device * const usart)
{
    const struct usart_priv *priv = (const struct usart_priv *)usart->priv;
    LL_APB2_GRP1_EnableClock(priv->apb2_grp1_periph); // Enables USART clock
    // TX pin
    LL_APB2_GRP1_EnableClock(priv->tx_pin_apb2_grp1_periph);
    LL_GPIO_Init(priv->tx_pin_gpio, (LL_GPIO_InitTypeDef *)&priv->tx_pin_config);
    // RX pin
    LL_APB2_GRP1_EnableClock(priv->rx_pin_apb2_grp1_periph);
    LL_GPIO_Init(priv->rx_pin_gpio, (LL_GPIO_InitTypeDef *)&priv->rx_pin_config);
    // USART
    LL_USART_Init(priv->usart, (LL_USART_InitTypeDef *)&priv->config);
    LL_USART_ConfigAsyncMode(priv->usart);
    LL_USART_Enable(priv->usart);

    priv_rtos[priv->index].tx_queue = xQueueCreate(64, sizeof(uint8_t));
    priv_rtos[priv->index].rx_queue = xQueueCreate(64, sizeof(uint8_t));
    priv_rtos[priv->index].mutex = xSemaphoreCreateMutex();

    LL_USART_EnableIT_RXNE(priv->usart);
    NVIC_SetPriority(priv->irqn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 5, 0));
    NVIC_EnableIRQ(priv->irqn);

    return 0;
}

static int32_t stm32f103xb_usart_write(const struct usart_device * const usart, const void *data, uint32_t size,
    uint32_t timeout)
{
    const struct usart_priv *priv = (const struct usart_priv *)usart->priv;
    const uint8_t *udata = (const uint8_t *)data;
    uint32_t i;
    int32_t ret;

    if (priv_rtos[priv->index].tx_queue == NULL || priv_rtos[priv->index].mutex == NULL) {
        return E_NOT_INITIALIZED;
    }

    if (xSemaphoreTake(priv_rtos[priv->index].mutex, timeout) == pdFAIL) {
        ret = E_TIMEOUT;
        goto exit;
    }
    for(i = 0; i < size; i++) {
        if (xQueueSend(priv_rtos[priv->index].tx_queue, &udata[i], timeout) == pdFAIL) {
            break; // Timed-out. Must stop and return now. A timeout is not an error!
        } else {
            LL_USART_EnableIT_TXE(priv->usart);
        }
    }
    ret = i;

    exit:
    xSemaphoreGive(priv_rtos[priv->index].mutex);
    return ret;
}

static int32_t stm32f103xb_usart_read(const struct usart_device * const usart, void *data, uint32_t size,
    uint32_t timeout)
{
    const struct usart_priv *priv = (const struct usart_priv *)usart->priv;
    uint8_t *udata = (uint8_t *)data;
    uint32_t i;
    int32_t ret;

    if (priv_rtos[priv->index].rx_queue == NULL || priv_rtos[priv->index].mutex == NULL) {
        ret = E_NOT_INITIALIZED;
        goto exit;
    }

    if (xSemaphoreTake(priv_rtos[priv->index].mutex, timeout) == pdFAIL) {
        ret = E_TIMEOUT;
        goto exit;
    }
    for(i = 0; i < size; i++) {
        if (xQueueReceive(priv_rtos[priv->index].rx_queue, &udata[i], timeout) == pdFAIL) {
            ret = E_TIMEOUT;
            goto exit;
        }
    }
    ret = i;

exit:
    xSemaphoreGive(priv_rtos[priv->index].mutex);
    return ret;
}

static int32_t stm32f103xb_usart_poll(const struct usart_device * const usart, enum poll_op op, void *answer)
{
    const struct usart_priv *priv = (const struct usart_priv *)usart->priv;
    int32_t ret;

    switch (op) {
    case POLL_RX_QUEUE_SIZE: {
        if (priv_rtos[priv->index].rx_queue == NULL) {
            ret = E_NOT_INITIALIZED;
            goto exit;
        }
        *((uint32_t *)answer) = uxQueueMessagesWaiting(priv_rtos[priv->index].rx_queue);
        ret = E_SUCCESS;
        break;
    }
    
    default:
        ret = E_POLLOP_INVALID;
        goto exit;
    }

exit:
    return ret;
}

// IRQ Handlers

static void USART_IRQHandler(const struct usart_device * const usart)
{
    BaseType_t context_switch;
    const struct usart_priv *priv = (const struct usart_priv *)usart->priv;

    if (LL_USART_IsActiveFlag_TXE(priv->usart)) {
        while (LL_USART_IsActiveFlag_TXE(priv->usart)) {
            uint8_t byte;
            if (xQueueReceiveFromISR(priv_rtos[priv->index].tx_queue, &byte, &context_switch) == pdFAIL) {
                LL_USART_DisableIT_TXE(priv->usart);
                break; // Queue empty and can safelly disable TXE
            }
            LL_USART_TransmitData8(priv->usart, byte);
        }
    }

    if (LL_USART_IsActiveFlag_RXNE(priv->usart)) {
        while(LL_USART_IsActiveFlag_RXNE(priv->usart)) {
            uint8_t byte = LL_USART_ReceiveData8(priv->usart);
            xQueueSendFromISR(priv_rtos[priv->index].rx_queue, &byte, &context_switch);
            // Could not enqueue anymore because queue is full
        }
    }

    portYIELD_FROM_ISR(context_switch);
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
    USART_IRQHandler(&usart1);
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
    USART_IRQHandler(&usart2);
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
    USART_IRQHandler(&usart3);
}
