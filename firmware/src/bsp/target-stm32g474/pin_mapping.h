#pragma once

/**
 * Current PIN mapping as per Schematic and Cube:
 * Device           |  Label       |  Pin   |  Peripheral
 * ---------------- | ------------ | ------ | ---------------------------
 * Button 1         |  BTN_1       |  PC13  |  GPIO_EXTI13
 * Button 2         |  BTN_2       |  PB9   |  GPIO_EXTI9
 * LED              |  LED         |  PB13  |  GPIO_Output
 * Battery Sensor   |  ADC_BAT     |  PB0   |  ADC1_IN15
 * Phototransistor  |  SEN_L_F     |  PA1   |  ADC1_IN2
 * Phototransistor  |  SEN_L_S     |  PA0   |  ADC1_IN1
 * Phototransistor  |  SEN_R_F     |  PB11  |  ADC1_IN14
 * Phototransistor  |  SEN_R_S     |  PB12  |  ADC1_IN11
 * LED              |  LED_L_F     |  PB4   |  GPIO_Output
 * LED              |  LED_L_S     |  PB3   |  GPIO_Output
 * LED              |  LED_R_F     |  PC12  |  GPIO_Output
 * LED              |  LED_R_S     |  PD2   |  GPIO_Output
 * WS2812           |  WS2812_IN1  |  PA15  |  TIM2_CH1 (PWM Generation)
 * Buzzer           |  BUZZER      |  PC6   |  TIM8_CH1 (PWM Generation)
 * IMU              |  I2C2_SCL    |  PA9   |  I2C2_SCL
 *                  |  I2C2_SDA    |  PA8   |  I2C2_SDA
 * EEPROM           |  EEPROM_SCL  |  PC8   |  I2C3_SCL
 *                  |  EEPROM_SDA  |  PC9   |  I2C3_SDA
 * BLE              |  BLE_TX      |  PC5   |  USART1_RX
 *                  |  BLE_RX      |  PC4   |  USART1_TX=
 * DRV8874PWPR      |  MOT1_PWM    |  PC3   |  TIM1_CH4 (PWM Generation)
 *                  |  MOT1_DIR    |  PC2   |  GPIO_Output
 *                  |  MOT1_ADC    |  PA4   |  ADC2_IN17
 * DRV8874PWPR      |  MOT2_PWM    |  PC1   |  TIM1_CH2 (PWM Generation)
 *                  |  MOT2_DIR    |  PC0   |  GPIO_Output
 *                  |  MOT2_ADC    |  PB2   |  ADC2_IN12
 * Encoder          |  SPI_CS_1    |  PB1   |  GPIO_Output
 *                  |  SPI_CLK     |  PA5   |  SPI1_SCK
 *                  |  SPI_MISO    |  PA6   |  SPI1_MISO
 *                  |  SPI_MOSI    |  PA7   |  SPI1_MOSI
 *                  |  ENC_1_A     |  PA2   |  GPIO_EXTI2
 *                  |  ENC_1_B     |  PA3   |  GPIO_Input
 * Encoder          |  SPI_CS_2    |  PB10  |  GPIO_Output
 *                  |  ENC_2_A     |  PC7   |  GPIO_EXTI7
 *                  |  ENC_2_B     |  PB15  |  GPIO_Input
 * Fan              |  MOT_FAN     |  PB5   |  TIM3_CH2 (PWM Generation)
 * USB              |  USB_D-      |  PA11  |  USB_DM
 *                  |  USB_D+      |  PA12  |  USB_DP
 */

#ifdef __cplusplus
extern "C" {
#endif

// Button
#define GPIO_BUTTON_1_PIN GPIO_PIN_9
#define GPIO_BUTTON_1_PORT GPIOB

#define GPIO_BUTTON_2_PIN GPIO_PIN_13
#define GPIO_BUTTON_2_PORT GPIOC

// LED
#define GPIO_LED_PIN GPIO_PIN_13
#define GPIO_LED_PORT GPIOB

// Encoders
#define ENCODER_LEFT_A_PIN GPIO_PIN_2
#define ENCODER_LEFT_A_PORT GPIOA

#define ENCODER_LEFT_B_PIN GPIO_PIN_3
#define ENCODER_LEFT_B_PORT GPIOA

#define ENCODER_RIGHT_A_PIN GPIO_PIN_7
#define ENCODER_RIGHT_A_PORT GPIOC

#define ENCODER_RIGHT_B_PIN GPIO_PIN_15
#define ENCODER_RIGHT_B_PORT GPIOB

// LEDs IR
#define GPIO_LED_IR_L_S_PIN GPIO_PIN_3
#define GPIO_LED_IR_L_S_PORT GPIOB

#define GPIO_LED_IR_L_F_PIN GPIO_PIN_4
#define GPIO_LED_IR_L_F_PORT GPIOB

#define GPIO_LED_IR_R_F_PIN GPIO_PIN_12
#define GPIO_LED_IR_R_F_PORT GPIOC

#define GPIO_LED_IR_R_S_PIN GPIO_PIN_2
#define GPIO_LED_IR_R_S_PORT GPIOD

// AS5047
#define SPI_CS_1_PIN GPIO_PIN_1
#define SPI_CS_1_PORT GPIOB

#define SPI_CS_2_PIN GPIO_PIN_10
#define SPI_CS_2_PORT GPIOB

// IMU
#define LSM6DSR_I2C_ADDR 0xD4

#ifdef __cplusplus
}
#endif
