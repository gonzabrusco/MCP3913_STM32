#ifndef __MCP3913_HAL_PORT_H__
#define __MCP3913_HAL_PORT_H__

#include <stdint.h>
#include "MCP3913.h"

/* Funciones complementarias especificas de STM32 para el driver MCP3914 */

/*
 * @brief Escribe un registro de 24 bits en el ADC MCP3913.
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a escribir (0 a 31)
 * @param reg_value Puntero al valor que se desea escribir representado como 3 bytes (3 bytes x 8 bits = 24 bits). El byte[0] es el LSB y el byte[2] es el MSB del registro de 24 bits.
 */
void MCP3913_Port_Write_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, const uint8_t* reg_value);
/*
 * @brief Lee N bytes del ADC MCP3913.
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a leer (0 a 31)
 * @param reg_value Puntero donde se guardaran los valores leidos representados como bytes.
 * @param n_bytes cantidad de bytes a leer
 */
void MCP3913_Port_Read_N_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value, uint8_t n_bytes);
/*
 * @brief Lee 3 bytes del ADC MCP3913.
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a leer (0 a 31)
 * @param reg_value Puntero donde se guardaran los valores leidos representados como bytes.
 */
void MCP3913_Port_Read_3_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value);
/*
 * @brief Lee 4 bytes del ADC MCP3913.
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a leer (0 a 31)
 * @param reg_value Puntero donde se guardaran los valores leidos representados como bytes.
 */
void MCP3913_Port_Read_4_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value);
/*
 * @brief Lee 24 bytes del ADC MCP3913 (utilizada para leer todos los canales en una sola operacion)
 * @param adc_handle Puntero al handle del ADC
 * @param reg_address Dirección del registro a leer (0 a 31)
 * @param reg_value Puntero donde se guardaran los valores leidos representados como bytes.
 */
void MCP3913_Port_Read_24_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value);
/*
 * @brief Setea el Chip Select del SPI en LOW
 * @param adc_handle Puntero al handle del ADC
 */
void MCP3913_Port_CS_Low(const MCP3913_handle_t* adc_handle);
/*
 * @brief Setea el Chip Select del SPI en HIGH
 * @param adc_handle Puntero al handle del ADC
 */
void MCP3913_Port_CS_High(const MCP3913_handle_t* adc_handle);

#endif
