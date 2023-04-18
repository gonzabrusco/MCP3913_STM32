#ifndef __MCP3913_PORT_H__
#define __MCP3913_PORT_H__

#include <stdint.h>
#include "MCP3913.h"

/* Funciones complementarias especificas de STM32 para el driver MCP3914 */

void MCP3913_Port_Write_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, const uint8_t* reg_value);
void MCP3913_Port_Read_N_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value, uint8_t n_bytes);
void MCP3913_Port_Read_3_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value);
void MCP3913_Port_Read_4_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value);
void MCP3913_Port_Read_24_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value);
void MCP3913_Port_CS_Low(const MCP3913_handle_t* adc_handle);
void MCP3913_Port_CS_High(const MCP3913_handle_t* adc_handle);

#endif
