/**
 ******************************************************************************
 * @file           : ms5837-i2c.c
 * @brief          : TE MS5837 I2C Pressure/Temperature Sensor driver
 *  ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 Logiq Air Suspension Systems.
 * All rights reserved.</center></h2>
 *
 * Proprietary Information
 ******************************************************************************/

/******************************************************************************/
#include "ms5837-i2c.h"
//#include "cmsis_os2.h"
/******************************************************************************/
// Note: This will probably be stored in FLASH
static uint16_t MS5837_Prom[MS5837_PROM_SIZE] =
{ 0 };

static I2C_HandleTypeDef *i2c_handle = NULL;

static uint8_t d1_address = MS5837_CONVERT_D1_256;
static uint8_t d2_address = MS5837_CONVERT_D2_256;
static uint8_t adc_read = MS5837_ADC_READ;

uint32_t i2cTransDelay = 1000;

/******************************************************************************/
/**
 * @fn uint32_t swap_24(uint32_t)
 * @brief
 *
 * @pre
 * @post
 * @param src
 * @return
 */
static inline uint32_t swap_24(uint32_t src)
{
    uint32_t low_bytes = src & 0x000000FF;
    uint32_t high_bytes = src & 0x00FF0000;
    uint32_t shift_results = src & 0x0000FF00;
    shift_results = shift_results | (low_bytes << 16);
    return (shift_results | (high_bytes >> 16));
}
/******************************************************************************/
/**
 * @fn uint8_t MS5837CRC4(uint32_t[])
 * @brief
 *
 * @pre
 * @post
 * @param n_prom
 * @return
 */
static __attribute__((unused)) uint8_t MS5837CRC4(uint32_t n_prom[]) // n_prom defined as 8x unsigned int (n_prom[8])
{
    int32_t cnt; // simple counter
    uint32_t n_rem = 0; // crc remainder
    uint8_t n_bit;

    n_prom[0] = ((n_prom[0]) & 0x0FFF); // CRC byte is replaced by 0
    n_prom[7] = 0; // Subsidiary value, set to 0

    for (cnt = 0; cnt < 16; ++cnt) // operation is performed on bytes
    { // choose LSB or MSB
        if (cnt % 2 == 1)
        {
            n_rem ^= (uint16_t) ((n_prom[cnt >> 1]) & 0x00FF);
        }
        else
        {
            n_rem ^= (uint16_t) (n_prom[cnt >> 1] >> 8);
        }

        for (n_bit = 8; n_bit > 0; --n_bit)
        {
            if (n_rem & (0x8000))
            {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else
            {
                n_rem = (n_rem << 1);
            }
        }
    }
    n_rem = ((n_rem >> 12) & 0x000F); // final 4-bit remainder is CRC code
    return (n_rem ^ 0x00);
}
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
int32_t MS5837Init(I2C_HandleTypeDef *handle)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t reset = MS5837_RESET;
    uint8_t address_offset = 0;

    if (handle != NULL)
    {
        i2c_handle = handle;
    }
    else
    {
        return (MS5837_ERROR_INVALID_HANDLE);
    }

    status = HAL_I2C_IsDeviceReady(i2c_handle, MS5837_ADDRESS_SHIFT, MS5837_TRIAL, HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return (MS5837_ERROR_DEVICE_NOTREADY);
    }

    status = HAL_I2C_Master_Transmit(i2c_handle, MS5837_ADDRESS_SHIFT, &reset, sizeof(uint8_t), HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return (MS5837_ERROR_TRANSMIT_RESET);
    }

    for (int16_t index = 0; index < MS5837_PROM_SIZE; ++index)
    {
        address_offset = MS5837_PROM_READ_START | (index << 1);
        status = HAL_I2C_Master_Transmit(i2c_handle, MS5837_ADDRESS_SHIFT, &address_offset, sizeof(uint8_t), HAL_MAX_DELAY);
        if (status != HAL_OK)
        {
            return (MS5837_ERROR_TRANSMIT_PROM);
        }

        status = HAL_I2C_Master_Receive(i2c_handle, MS5837_ADDRESS_SHIFT, (uint8_t*) &MS5837_Prom[index], sizeof(uint16_t),
        HAL_MAX_DELAY);

        if (status != HAL_OK)
        {
            return (MS5837_ERROR_RECIEVE_PROM);
        }

        MS5837_Prom[index] = __REV16(MS5837_Prom[index]);
    }
    return MS5837_SUCCESS;
}
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
int32_t MS5837Read(I2C_HandleTypeDef *handle, MS5837_Sensor_Result *result)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t D_value = 0;

    // D1 and D2 need to be unsigned 32-bit integers (long 0-4294967295)
    uint32_t D1 = 0;    // Store uncompensated pressure value
    uint32_t D2 = 0;    // Store uncompensated temperature value

    // These three variables are used for the conversion steps
    // They should be signed 32-bit integer initially
    // i.e. signed long from -2147483648 to 2147483647
    int64_t dT = 0;
    int32_t TEMP = 0;

    // These values need to be signed 64 bit integers
    // (long long = int64_t)
    int64_t OFF = 0;
    int64_t SENS = 0;

    int64_t Ti = 0;
    int64_t OFFi = 0;
    int64_t SENSi = 0;

    int32_t P = 0; // First order pressure in mbar, initially as a signed long integer
    UNUSED(P);
    int32_t P2 = 0; // Second order pressure in mbar, initially as a signed long integer


    if (handle != NULL)
    {
        i2c_handle = handle;
    }
    else
    {
        return (MS5837_ERROR_INVALID_HANDLE);
    }

    if (result == NULL)
    {
        return MS5837_ERROR_INVALID_POINTER;
    }
    else
    {
        result->pressure = 0.0f;
        result->temperature = 0.0f;
    }

    // First read the D1 register for pressure
    status = HAL_I2C_Master_Transmit(i2c_handle, MS5837_ADDRESS_SHIFT, &d1_address, sizeof(uint8_t), HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return (MS5837_ERROR_TRANSMIT_READ_D1);
    }

    //osDelay(I2C_DELAY_D1);
    HAL_Delay(i2cTransDelay);

    status = HAL_I2C_Master_Transmit(i2c_handle, MS5837_ADDRESS_SHIFT, &adc_read, sizeof(uint8_t), HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return (MS5837_ERROR_READ_ADC);
    }

    HAL_Delay(i2cTransDelay);

    status = HAL_I2C_Master_Receive(i2c_handle, MS5837_ADDRESS_SHIFT, (uint8_t*) &D_value, sizeof(uint32_t), HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return (MS5837_ERROR_RECIEVE_READ_D1);
    }

    D1 = swap_24(D_value);

    HAL_Delay(i2cTransDelay);

    // Second we read the D2 register for temperature
    status = HAL_I2C_Master_Transmit(i2c_handle, MS5837_ADDRESS_SHIFT, &d2_address, sizeof(uint8_t), HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return (MS5837_ERROR_TRANSMIT_READ_D1);
    }

    //osDelay(I2C_DELAY_D2);
    HAL_Delay(i2cTransDelay);

    status = HAL_I2C_Master_Transmit(i2c_handle, MS5837_ADDRESS_SHIFT, &adc_read, sizeof(uint8_t), HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return (MS5837_ERROR_READ_ADC);
    }

    status = HAL_I2C_Master_Receive(i2c_handle, MS5837_ADDRESS_SHIFT, (uint8_t*) &D_value, sizeof(uint32_t), HAL_MAX_DELAY);
    if (status != HAL_OK)
    {
        return (MS5837_ERROR_RECIEVE_READ_D2);
    }

    D2 = swap_24(D_value);

//    // Calculate temperature
    dT = (int32_t)D2 - ((int32_t)MS5837_Prom[MS5837_C5] << 8);
    TEMP = 2000 + ((int64_t)dT * (int64_t)MS5837_Prom[MS5837_C6] >> 23);

    OFF = (MS5837_Prom[MS5837_C2] << 16) + ((MS5837_Prom[MS5837_C4] * dT) >> 7);
    SENS = ((int64_t)MS5837_Prom[MS5837_C1] << 15) + ((int64_t)(MS5837_Prom[MS5837_C3] * dT) >> 8);
    P = ((D1 * SENS >> 21) - OFF) >> 13;

    // Do 2nd order temperature compensation (see pg 12 of MS5837 data sheet)
    if (TEMP < 2000)
    {
        // Low temperature
        Ti = 3 * (dT^2) >> 33;
        OFFi = 3 * (((int64_t)TEMP - 2000) ^ 2) >> 1;
        SENSi = 5 * (((int64_t)TEMP - 2000) ^ 2) >> 3;
        if (TEMP < -1500)
        {
            OFFi = (int64_t)OFFi + 7 * (((int64_t)TEMP + 1500) ^ 2);
            SENSi = (int64_t)SENSi + 4 * (((int64_t)TEMP + 1500) * ((int64_t)TEMP + 1500));
        }
    }
    else
    {
        // High temperature
        Ti = 2 * (dT^2) >> 37;
        OFFi = 1 * (((int64_t)TEMP - 2000) ^ 2) >> 4;
        SENSi = 0;
    }

    OFFi = OFF - OFFi;
    SENSi = SENS - SENSi;
    result->temperature = ((float)TEMP - Ti) / 100;
    P2 =  ((( ((D1 * SENSi) >> 21) - OFFi) >> 13) / 10);

    result->pressure = (float)P2 * 0.0145038;

    return MS5837_SUCCESS;
}
/******************************************************************************/
/******************************************************************************/

