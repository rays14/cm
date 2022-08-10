/**
 ******************************************************************************
 * @file ms-5837-i2c.h
 * @brief TE 5837 Pressure/Temperature Sensor
 * @link https://www.te.com/usa-en/product-CAT-BLPS0017.html
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 Logiq Air Suspension Systems.
 * All rights reserved.</center></h2>
 *
 * Proprietary Information
 ******************************************************************************/
/******************************************************************************/
#include <ms5837-error.h>
#include <stdint.h>
#include <stm32l4xx_hal.h>
/******************************************************************************/
#ifndef INC_MS5837_I2C_H_
#define INC_MS5837_I2C_H_
/******************************************************************************/
#define MS5837_ADDRESS          0x76
#define MS5837_ADDRESS_SHIFT    MS5837_ADDRESS << 1

#define MS5837_RESET            0x1E
#define MS5837_CONVERT_D1_256   0x40
#define MS5837_CONVERT_D1_512   0x42
#define MS5837_CONVERT_D1_1024  0x44
#define MS5837_CONVERT_D1_2048  0x46
#define MS5837_CONVERT_D1_4096  0x48
#define MS5837_CONVERT_D2_256   0x50
#define MS5837_CONVERT_D2_512   0x52
#define MS5837_CONVERT_D2_1024  0x54
#define MS5837_CONVERT_D2_2048  0x56
#define MS5837_CONVERT_D2_4096  0x58
#define MS5837_ADC_READ         0x00
#define MS5837_PROM_READ_START  0xA0
#define MS5837_PROM_READ_END    0xAC

#define MS5837_PROM_SIZE        7
// #define MS5837_D1_SIZE          5
// #define MS5837_D2_SIZE          5

#define MS5837_C1               1 // Pressure Sensitivity - C1
#define MS5837_C2               2 // Pressure Offset - C2
#define MS5837_C3               3 // Temperature coefficient of pressure sensitivity - C3
#define MS5837_C4               4 // Temperature coefficient of pressure offset - C4
#define MS5837_C5               5 // Reference temperature - C5
#define MS5837_C6               6 // Temperature coefficient of the temperature - C6

#define I2C_READ                1
#define I2C_WRITE               0
#define I2C_DELAY_D1            2
#define I2C_DELAY_D2            2

#define MS5837_TRIAL            1

#define FACTORY_SETTINGS_VALUE  0x001101000000

#define TWO_1TH                 2
#define TWO_3RD                 8
#define TWO_4th                 16
#define TWO_7TH                 128
#define TWO_8TH                 256
#define TWO_13TH                8192
#define TWO_15TH                32768
#define TWO_16TH                65536
#define TWO_21ST                2097150
#define TWO_23RD                8388610
#define TWO_33RD                8589930000
#define TWO_37TH                137439000000
/******************************************************************************/
typedef struct
{
    float pressure;
    float temperature;
} MS5837_Sensor_Result;

extern uint32_t i2cTransDelay;

/******************************************************************************/
/**
 * @fn int32_t MS5837Init(I2C_HandleTypeDef*)
 * @brief
 *
 * @pre
 * @post
 * @param handle
 * @return
 */
int32_t MS5837Init(I2C_HandleTypeDef *handle);
/******************************************************************************/
/**
 * @fn int32_t MS5837Read(I2C_HandleTypeDef*, MS5837_Sensor_Result*)
 * @brief
 *
 * @pre
 * @post
 * @param handle
 * @param result
 * @return
 */
int32_t MS5837Read(I2C_HandleTypeDef *handle, MS5837_Sensor_Result *result);
/******************************************************************************/
/**
 * @fn int32_t MS5837PressureConvert(void)
 * @brief
 *
 * @pre
 * @post
 * @return
 */
int32_t MS5837PressureConvert(void);
/******************************************************************************/
#endif /* INC_MS5837_I2C_H_ */
/******************************************************************************/
/******************************************************************************/
