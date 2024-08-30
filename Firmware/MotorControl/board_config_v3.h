/*
* @brief Contains board specific configuration for ODrive v3.x
*/

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

// STM specific includes
#include <gpio.h>
#include <spi.h>
#include <tim.h>
#include <main.h>
#include <usart.h>

#if HW_VERSION_MAJOR == 3
#if HW_VERSION_MINOR <= 3
#define SHUNT_RESISTANCE (675e-6f)
#else
#define SHUNT_RESISTANCE (2000e-6f)
#endif
#endif


typedef struct {
    uint16_t step_gpio_pin;
    uint16_t dir_gpio_pin;
    osPriority thread_priority;
} AxisHardwareConfig_t;

typedef struct {
    TIM_HandleTypeDef* timer;
    GPIO_TypeDef* index_port;
    uint16_t index_pin;
    GPIO_TypeDef* hallA_port;
    uint16_t hallA_pin;
    GPIO_TypeDef* hallB_port;
    uint16_t hallB_pin;
    GPIO_TypeDef* hallC_port;
    uint16_t hallC_pin;
    SPI_HandleTypeDef* motor_spi;
    SPI_HandleTypeDef* GearboxOutputEncoder_spi;
  //  UART_HandleTypeDef *uart;

} EncoderHardwareConfig_t;
typedef struct {
    TIM_HandleTypeDef* timer;
    uint16_t control_deadline;
    float shunt_conductance;
} MotorHardwareConfig_t;
typedef struct {
    const float* const coeffs;
    size_t num_coeffs;
    size_t adc_ch;
    size_t aux_temp;
} ThermistorHardwareConfig_t;
typedef struct {
    SPI_HandleTypeDef* spi;
    GPIO_TypeDef* enable_port;
    uint16_t enable_pin;
    GPIO_TypeDef* nCS_port;
    uint16_t nCS_pin;
    GPIO_TypeDef* nFAULT_port;
    uint16_t nFAULT_pin;
} GateDriverHardwareConfig_t;
typedef struct {
    AxisHardwareConfig_t axis_config;
    EncoderHardwareConfig_t encoder_config;
    MotorHardwareConfig_t motor_config;
    ThermistorHardwareConfig_t thermistor_config;
    GateDriverHardwareConfig_t gate_driver_config;
} BoardHardwareConfig_t;

extern const BoardHardwareConfig_t hw_configs[2];

//TODO stick this in a C file
#ifdef __MAIN_CPP__
const float fet_thermistor_poly_coeffs[] =
    {257.61f, -324.08f,  258.34f, -23.12f};  //3300 Ohm
const float fet_thermistor_poly_coeffs2[] =
    {447.04f ,-646.84f, 404.97f , -71.47f};  //10000 Ohm
const size_t fet_thermistor_num_coeffs = sizeof(fet_thermistor_poly_coeffs)/sizeof(fet_thermistor_poly_coeffs[1]);

const BoardHardwareConfig_t hw_configs[2] = { {
    //M0
    .axis_config = {
        .step_gpio_pin = 1,
        .dir_gpio_pin = 2,
        .thread_priority = (osPriority)(osPriorityHigh + (osPriority)1),
    },
    .encoder_config = {
        .timer = &htim3,
        .index_port = GPIO_3_GPIO_Port,
        .index_pin = GPIO_3_Pin,
        .hallA_port = GPIO_3_GPIO_Port,
        .hallA_pin = GPIO_3_Pin,
        .hallB_port = GPIO_3_GPIO_Port,
        .hallB_pin = GPIO_3_Pin,
        .hallC_port = GPIO_3_GPIO_Port,
        .hallC_pin = GPIO_3_Pin,
        .motor_spi = &hspi3,
        .GearboxOutputEncoder_spi = &hspi1,
    //    .uart = &huart4,
    },
    .motor_config = {
        .timer = &htim1,
        .control_deadline = TIM_1_8_PERIOD_CLOCKS,
        .shunt_conductance = 1.0f / SHUNT_RESISTANCE,  //[S]
    },
    .thermistor_config = {
        .coeffs = &fet_thermistor_poly_coeffs[0],
        .num_coeffs = fet_thermistor_num_coeffs,
        .adc_ch = 15,
        .aux_temp = 14,
    },
    .gate_driver_config = {
        .spi = &hspi3,
        // Note: this board has the EN_Gate pin shared!
        .enable_port = EN_GATE_GPIO_Port,
        .enable_pin = EN_GATE_Pin,
        .nCS_port = M0_nCS_GPIO_Port,
        .nCS_pin = M0_nCS_Pin,
        .nFAULT_port = nFAULT_GPIO_Port, // the nFAULT pin is shared between both motors
        .nFAULT_pin = nFAULT_Pin,
    }
},{
    //M1
    .axis_config = {
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5
        .step_gpio_pin = 7,
        .dir_gpio_pin = 8,
#else
        .step_gpio_pin = 3,
        .dir_gpio_pin = 4,
#endif
        .thread_priority = osPriorityHigh,
    },
    .encoder_config = {
        .timer = &htim4,
        .index_port = GPIO_3_GPIO_Port,
        .index_pin = GPIO_3_Pin,
        .hallA_port = GPIO_3_GPIO_Port,
        .hallA_pin = GPIO_3_Pin,
        .hallB_port = GPIO_3_GPIO_Port,
        .hallB_pin = GPIO_3_Pin,
        .hallC_port = GPIO_3_GPIO_Port,
        .hallC_pin = GPIO_3_Pin,
        .motor_spi = &hspi3,
    },
    .motor_config = {
        .timer = &htim8,
        .control_deadline = (3 * TIM_1_8_PERIOD_CLOCKS) / 2,
        .shunt_conductance = 1.0f / SHUNT_RESISTANCE,  //[S]
    },
    .thermistor_config = {
        .coeffs = &fet_thermistor_poly_coeffs[0],
        .num_coeffs = fet_thermistor_num_coeffs,
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
        .adc_ch = 4,
#else
        .adc_ch = 1,
#endif
    },
    .gate_driver_config = {
        .spi = &hspi3,
        // Note: this board has the EN_Gate pin shared!
        .enable_port = EN_GATE_GPIO_Port,
        .enable_pin = EN_GATE_Pin,
        .nCS_port = M0_nCS_GPIO_Port,
        .nCS_pin = M0_nCS_Pin,
        .nFAULT_port = nFAULT_GPIO_Port, // the nFAULT pin is shared between both motors
        .nFAULT_pin = nFAULT_Pin,
    }
} };
#endif



#define I2C_A0_PORT GPIO_3_GPIO_Port
#define I2C_A0_PIN GPIO_3_Pin
#define I2C_A1_PORT GPIO_4_GPIO_Port
#define I2C_A1_PIN GPIO_4_Pin
#define I2C_A2_PORT GPIO_5_GPIO_Port
#define I2C_A2_PIN GPIO_5_Pin

#endif // __BOARD_CONFIG_H
