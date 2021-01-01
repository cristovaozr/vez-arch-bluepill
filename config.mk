##
# @author Cristóvão Zuppardo Rufino <cristovaozr@gmail.com>
# @version 0.1
#
# @copyright Copyright Cristóvão Zuppardo Rufino (c) 2020
# Please see LICENCE file to information regarding licensing

# Configuration flags for STM32F103xB

R_PATH = arch/$(ARCH)

CPU = -mcpu=cortex-m3

ARCH_MCU = $(CPU) -mthumb

# ARCH specific ASM sources
ARCH_ASM_SOURCES += \
	$(R_PATH)/src/startup_stm32f103xb.s

ARCH_AS_INCLUDES +=

ARCH_C_DEFS += \
	-DUSE_FULL_LL_DRIVER \
	-DHSE_VALUE=8000000 \
	-DHSE_STARTUP_TIMEOUT=100 \
	-DLSE_STARTUP_TIMEOUT=5000 \
	-DLSE_VALUE=32768 \
	-DHSI_VALUE=8000000 \
	-DLSI_VALUE=40000 \
	-DVDD_VALUE=3300 \
	-DPREFETCH_ENABLE=1 \
	-DSTM32F103xB

ARCH_C_INCLUDES += \
	-I$(R_PATH) \
	-I$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Inc \
	-I$(R_PATH)/freertos/portable/GCC/ARM_CM3 \
	-I$(R_PATH)/Drivers/CMSIS/Device/ST/STM32F1xx/Include \
	-I$(R_PATH)/Drivers/CMSIS/Include

ARCH_LDSCRIPT = $(R_PATH)/STM32F103C8Tx_FLASH.ld

ARCH_C_SOURCES += \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_gpio.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_adc.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_dma.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rcc.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_utils.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_exti.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_i2c.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_rtc.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_spi.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_pwr.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_usart.c \
	$(R_PATH)/Drivers/STM32F1xx_HAL_Driver/Src/stm32f1xx_ll_tim.c

ARCH_C_SOURCES += \
	$(R_PATH)/freertos/portable/GCC/ARM_CM3/port.c

ARCH_C_SOURCES += \
	$(R_PATH)/src/stm32f1xx_it.c \
	$(R_PATH)/src/system_stm32f1xx.c \
	$(R_PATH)/src/hw_init.c \
	$(R_PATH)/src/device/gpio_impl.c \
	$(R_PATH)/src/device/usart_impl.c \
	$(R_PATH)/src/device/pwm_impl.c \
	$(R_PATH)/src/device/spi_impl.c