#include "MCP3913_HAL_port.h"
#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * @brief Setea el Chip Select del SPI en LOW
 * @param adc_handle Puntero al handle del ADC
 */
void MCP3913_Port_CS_Low(const MCP3913_handle_t* adc_handle) {
  HAL_GPIO_WritePin((GPIO_TypeDef *)adc_handle->spi_cs_port, adc_handle->spi_cs_pin, GPIO_PIN_RESET); // Activo el CS
}

/*
 * @brief Setea el Chip Select del SPI en HIGH
 * @param adc_handle Puntero al handle del ADC
 */
void MCP3913_Port_CS_High(const MCP3913_handle_t* adc_handle) {
  HAL_GPIO_WritePin((GPIO_TypeDef *)adc_handle->spi_cs_port, adc_handle->spi_cs_pin, GPIO_PIN_SET); // Desactivo el CS
}

/*
 * @brief Escribe un registro de 24 bits en el ADC MCP3913.
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a escribir (0 a 31)
 * @param reg_value Puntero al valor que se desea escribir representado como 3 bytes (3 bytes x 8 bits = 24 bits). El byte[0] es el LSB y el byte[2] es el MSB del registro de 24 bits.
 */
void MCP3913_Port_Write_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, const uint8_t* reg_value) {
  MCP3913_Control_Byte_t control;

  control.read = false;
  control.reg_address = reg_address;
  control.dev_address = adc_handle->dev_address;

  MCP3913_Port_CS_Low(adc_handle); // Activo el CS

  HAL_SPI_Transmit((SPI_HandleTypeDef*)adc_handle->spi_handle, &control.byte, 1, HAL_MAX_DELAY); // Envio el CONTROL BYTE
  for(int i = 0; i < 3; i++) {
    HAL_SPI_Transmit((SPI_HandleTypeDef*)adc_handle->spi_handle, (uint8_t*)&reg_value[2-i], 1, HAL_MAX_DELAY); // Escribo registro del ADC
  }

  MCP3913_Port_CS_High(adc_handle); // Desactivo el CS
}

/*
 * @brief Lee N bytes del ADC MCP3913.
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a leer (0 a 31)
 * @param reg_value Puntero donde se guardaran los valores leidos representados como bytes.
 * @param n_bytes cantidad de bytes a leer
 */
void MCP3913_Port_Read_N_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value, uint8_t n_bytes) {
  MCP3913_Control_Byte_t control;

  control.read = true;
  control.reg_address = reg_address;
  control.dev_address = adc_handle->dev_address;

  MCP3913_Port_CS_Low(adc_handle); // Activo el CS

  HAL_SPI_Transmit((SPI_HandleTypeDef*)adc_handle->spi_handle, &control.byte, 1, HAL_MAX_DELAY); // Envio el CONTROL BYTE
  for(int i = 0; i < n_bytes; i++) {
    HAL_SPI_Receive((SPI_HandleTypeDef*)adc_handle->spi_handle, &reg_value[n_bytes-1-i], 1, HAL_MAX_DELAY); // Leo el o los registros del ADC
  }

  MCP3913_Port_CS_High(adc_handle); // Desactivo el CS
}

/*
 * @brief Lee 3 bytes del ADC MCP3913.
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a leer (0 a 31)
 * @param reg_value Puntero donde se guardaran los valores leidos representados como bytes.
 */
void MCP3913_Port_Read_3_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value) {
  MCP3913_Port_Read_N_Bytes_Reg(adc_handle, reg_address, reg_value, 3);
}

/*
 * @brief Lee 4 bytes del ADC MCP3913.
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a leer (0 a 31)
 * @param reg_value Puntero donde se guardaran los valores leidos representados como bytes.
 */
void MCP3913_Port_Read_4_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value) {
  MCP3913_Port_Read_N_Bytes_Reg(adc_handle, reg_address, reg_value, 4);
}

/*
 * @brief Lee 24 bytes del ADC MCP3913 (utilizada para leer todos los canales en una sola operacion)
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a leer (0 a 31)
 * @param reg_value Puntero donde se guardaran los valores leidos representados como bytes.
 */
void MCP3913_Port_Read_24_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value) {
  MCP3913_Port_Read_N_Bytes_Reg(adc_handle, reg_address, reg_value, 24);
}
