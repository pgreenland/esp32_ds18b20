#ifndef DS18B20_H
#define DS18B20_H

/* Standard libraries */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ESP-IDF */
#include "hal/uart_types.h"

/* 1-Wire library */
#include "one_wire.h"

/* Public definitions */
#define DS18B20_FAMILY_CODE (0x28U)

/* Public types */
typedef enum
{
	eDS18B20_Resolution_9Bit = 0U,
	eDS18B20_Resolution_10Bit,
	eDS18B20_Resolution_11Bit,
	eDS18B20_Resolution_12Bit

} eDS18B20_Resolution_t;

/* Public functions */

/**
 * @brief Set the resolution of the DS18B20 sensor
 *
 * @param eUART UART instance used for 1-Wire communication
 * @param pstROMCode Pointer to ROM code of the device to configure
 * @param eResolution Desired resolution setting
 *
 * @return TRUE if successful, FALSE otherwise
 */
bool DS18B20_SetResolution(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, eDS18B20_Resolution_t eResolution);

/**
 * @brief Issue simultaneous temperature conversion command to all DS18B20 devices on the bus
 *
 * @param eUART UART instance used for 1-Wire communication
 *
 * @return TRUE if successful, FALSE otherwise
 */
bool DS18B20_SimultaneousConvert(uart_port_t eUART);

/**
 * @brief Read temperature from DS18B20 sensor - assumes conversion has already been performed (for example via simultaneous convert)
 *
 * @param eUART UART instance used for 1-Wire communication
 * @param pstROMCode Pointer to ROM code of the device to read
 * @param pfTemperature Pointer to variable to store read temperature in degrees Celsius
 *
 * @return TRUE if successful, FALSE otherwise
 */
bool DS18B20_ReadTemp(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, float *pfTemperature);

/**
 * @brief Convert and read temperature from DS18B20 sensor
 *
 * @param eUART UART instance used for 1-Wire communication
 * @param pstROMCode Pointer to ROM code of the device to read
 * @param pfTemperature Pointer to variable to store read temperature in degrees Celsius
 *
 * @return TRUE if successful, FALSE otherwise
 */
bool DS18B20_ConvertAndReadTemp(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, float *pfTemperature);

#endif
