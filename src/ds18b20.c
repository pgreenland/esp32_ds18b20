/* ESP Logging */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

/* FreeRTOS */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Public header */
#include "ds18b20.h"

/* Private definitions - Commands and parameters */
#define DS18B20_CMD_CONVERT_T (0x44U)
#define DS18B20_CMD_READ_SCRATCHPAD (0xBEU)
#define DS18B20_CMD_WRITE_SCRATCHPAD (0x4EU)

/* Max conversion time for 12-bit resolution + 50ms */
#define DS18B20_CONVERSION_TIMEOUT_MS (800U)

/* Poll every 10ms during conversion */
#define DS18B20_CONVERSION_POLL_MS (10U)

/* Scratchpad size in bytes */
#define DS18B20_SCRATCHPAD_SIZE (9U) /* (including CRC) */

/* Temperature scaling */
#define DS18B20_DEGC_PER_BIT (0.0625f) /* LSB = 0.0625 degC */

/* Resolution bit shift */
#define DS18B20_RESOLUTION_BIT_SHIFT (5U)

/* Fixed configuration bits */
#define DS18B20_FIXED_CONFIG_BITS (0x1FU)

/* Scratchpad byte indexes */
#define DS18B20_SCRATCHPAD_INDEX_TEMP_LSB (0U)
#define DS18B20_SCRATCHPAD_INDEX_TEMP_MSB (1U)
#define DS18B20_SCRATCHPAD_INDEX_TH (2U)
#define DS18B20_SCRATCHPAD_INDEX_TL (3U)
#define DS18B20_SCRATCHPAD_INDEX_CONFIG (4U)

/* Private functions */
static bool ConvertAndWaitForCompletion(uart_port_t eUART);
static bool ReadScratchpad(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, uint8_t *pabyScratchpad);

/* Private variables - log tag */
static const char *TAG = "DS18B20";

/* Public functions */
bool DS18B20_SetResolution(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, eDS18B20_Resolution_t eResolution)
{
	bool bSuccess = true;
	uint8_t abyScratchpad[DS18B20_SCRATCHPAD_SIZE];

	/* Saturate resolution */
	if (eResolution > eDS18B20_Resolution_12Bit) eResolution = eDS18B20_Resolution_12Bit;

	/* Prepare configuration byte */
	uint8_t uiConfig = ((uint8_t)eResolution << DS18B20_RESOLUTION_BIT_SHIFT) | DS18B20_FIXED_CONFIG_BITS;

	/* Validate arguments */
	if (NULL == pstROMCode)
	{
		ESP_LOGE(TAG, "Invalid ROM code pointer");
		bSuccess = false;
	}

	if (bSuccess)
	{
		/* Read scratchpad */
		bSuccess = ReadScratchpad(eUART, pstROMCode, abyScratchpad);
	}

	if (bSuccess)
	{
		/* Reset bus */
		if (!ONE_WIRE_Reset(eUART))
		{
			/* No devices present */
			bSuccess = false;
		}
	}

	if (bSuccess)
	{
		/* Select device */
		ONE_WIRE_MatchROM(eUART, pstROMCode);

		/* Issue write scratchpad command */
		ONE_WIRE_WriteByte(eUART, DS18B20_CMD_WRITE_SCRATCHPAD);

		/* Write scratchpad bytes - restoring TH and TL, while updating config */
		ONE_WIRE_WriteByte(eUART, abyScratchpad[DS18B20_SCRATCHPAD_INDEX_TH]);
		ONE_WIRE_WriteByte(eUART, abyScratchpad[DS18B20_SCRATCHPAD_INDEX_TL]);
		ONE_WIRE_WriteByte(eUART, uiConfig);

		/* Read back scratchpad to verify */
		bSuccess = ReadScratchpad(eUART, pstROMCode, abyScratchpad);
	}

	if (bSuccess)
	{
		/* Check configuration byte is correct */
		bSuccess = (abyScratchpad[DS18B20_SCRATCHPAD_INDEX_CONFIG] == uiConfig);
	}

	return bSuccess;
}

bool DS18B20_SimultaneousConvert(uart_port_t eUART)
{
	bool bSuccess = true;

	/* Reset bus */
	if (!ONE_WIRE_Reset(eUART))
	{
		/* No devices present */
		bSuccess = false;
	}

	if (bSuccess)
	{
		/* Issue skip ROM command */
		ONE_WIRE_SkipROM(eUART);

		/* Issue conversion command and wait for completion */
		bSuccess = ConvertAndWaitForCompletion(eUART);
	}

	return bSuccess;
}

bool DS18B20_ReadTemp(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, float *pfTemperature)
{
	bool bSuccess = true;
	uint8_t abyScratchpad[DS18B20_SCRATCHPAD_SIZE];

	/* Validate arguments */
	if (	(NULL == pstROMCode)
		 || (NULL == pfTemperature)
	   )
	{
		ESP_LOGE(TAG, "Invalid arguments");
		bSuccess = false;
	}

	if (bSuccess)
	{
		/* Read scratchpad */
		bSuccess = ReadScratchpad(eUART, pstROMCode, abyScratchpad);
	}

	if (bSuccess)
	{
		/* Extract temperature (LSB first) */
		uint16_t uiTemperature = (((uint16_t)abyScratchpad[DS18B20_SCRATCHPAD_INDEX_TEMP_MSB] << 8U) | (uint16_t)abyScratchpad[DS18B20_SCRATCHPAD_INDEX_TEMP_LSB]);
		int16_t iTemperature = (int16_t)uiTemperature;
		*pfTemperature = ((float)iTemperature * DS18B20_DEGC_PER_BIT);
	}

	return bSuccess;
}

bool DS18B20_ConvertAndReadTemp(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, float *pfTemperature)
{
	bool bSuccess = true;

	/* Validate arguments */
	if (	(NULL == pstROMCode)
		 || (NULL == pfTemperature)
	   )
	{
		ESP_LOGE(TAG, "Invalid arguments");
		bSuccess = false;
	}

	if (bSuccess)
	{
		/* Reset bus */
		if (!ONE_WIRE_Reset(eUART))
		{
			/* No devices present */
			bSuccess = false;
		}
	}

	if (bSuccess)
	{
		/* Select device */
		ONE_WIRE_MatchROM(eUART, pstROMCode);

		/* Issue Convert T command */
		bSuccess = ConvertAndWaitForCompletion(eUART);
	}

	if (bSuccess)
	{
		/* Read temperature from device */
		bSuccess = DS18B20_ReadTemp(eUART, pstROMCode, pfTemperature);
	}

	return bSuccess;
}

/* Private functions */

/**
 * @brief Issue temperature conversion and wait for completion
 *
 * @param eUART UART instance used for 1-Wire communication
 */
static bool ConvertAndWaitForCompletion(uart_port_t eUART)
{
	bool bSuccess = false;

	/* Issue Convert T command */
	ONE_WIRE_WriteByte(eUART, DS18B20_CMD_CONVERT_T);

	/* Wait for conversion to complete */
	for (uint32_t uiWait = 0; uiWait < DS18B20_CONVERSION_TIMEOUT_MS; uiWait += DS18B20_CONVERSION_POLL_MS)
	{
		/* Wait a bit */
		vTaskDelay(pdMS_TO_TICKS(DS18B20_CONVERSION_POLL_MS));

		/* Read a bit */
		//bSuccess = (0U != ONE_WIRE_ReadByte(eUART)); // Using byte read for logic analyzer decoding
		bSuccess = (0U != ONE_WIRE_ReadByte(eUART));

		/* Check if conversion complete */
		if (bSuccess)
		{
			break;
		}
	}

	return bSuccess;
}

/**
 * @brief Read scratchpad data from DS18B20 device
 *
 * @param eUART UART instance used for 1-Wire communication
 * @param pstROMCode Pointer to ROM code of the device to read
 * @param pabyScratchpad Pointer to buffer to store scratchpad data (must be at least DS18B20_SCRATCHPAD_SIZE bytes)
 *
 * @return TRUE if successful, FALSE otherwise
 */
static bool ReadScratchpad(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, uint8_t *pabyScratchpad)
{
	bool bSuccess = true;

	/* Validate arguments */
	if (	(NULL == pstROMCode)
		 || (NULL == pabyScratchpad)
	   )
	{
		ESP_LOGE(TAG, "Invalid arguments");
		bSuccess = false;
	}

	if (bSuccess)
	{
		/* Reset bus */
		if (!ONE_WIRE_Reset(eUART))
		{
			/* No devices present */
			bSuccess = false;
		}
	}

	if (bSuccess)
	{
		/* Select device */
		ONE_WIRE_MatchROM(eUART, pstROMCode);

		/* Issue read scratchpad command */
		ONE_WIRE_WriteByte(eUART, DS18B20_CMD_READ_SCRATCHPAD);

		/* Read scratchpad data with CRC */
		for (uint8_t i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
		{
			pabyScratchpad[i] = ONE_WIRE_ReadByte(eUART);
		}

		/* Verify CRC */
		bSuccess = (ONE_WIRE_CRC8(pabyScratchpad, DS18B20_SCRATCHPAD_SIZE - 1U) == pabyScratchpad[DS18B20_SCRATCHPAD_SIZE - 1U]);
	}

	return bSuccess;
}
