# vez-arch-bluepill
VEZ - Volta à estaca zero - configuração ARCH para Bluepill

# Pinos usados na plataforma

Os seguintes pinos são usados na plataforma atualmente

| Pino STM32 |    Pino no FW    |  Usado em  |
|------------|------------------|------------|
| PA2        | usart2_prix (TX) |   Shell    |
| PA3        | usart2_priv (RX) |   Shell    |
| PA9        | usart1_priv (TX) |   USART1   |
| PA10       | usart1_priv (RX) |   USART1   |
| PC13       | led_gpio         | LED padrão |
| PA8        | SPI1 CE          | nRF24L01+  |
| PA4        | SPI1 CS          | nRF24L01+  |
| PA5        | SPI1 SCK         | nRF24L01+  |
| PA6        | SPI1 SDI         | nRF24L01+  |
| PA7        | SPI1 SDO         | nRF24L01+  |
| PB6        | I2C SCL          |    ???     |
| PB7        | I2C SDA          |    ???     |