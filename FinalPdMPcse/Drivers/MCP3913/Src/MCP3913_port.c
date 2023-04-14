#include "MCP3914_port.h"
#include "stm32g0xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* Pone en nivel bajo el CS */
void MCP3913_Port_CS_Low(const MCP3913_handle_t* adc_handle) {
  HAL_GPIO_WritePin((GPIO_TypeDef *)adc_handle->spi_cs_port, adc_handle->spi_cs_pin, GPIO_PIN_RESET);
}

/* Pone en nivel alto el CS */
void MCP3913_Port_CS_High(const MCP3913_handle_t* adc_handle) {
  HAL_GPIO_WritePin((GPIO_TypeDef *)adc_handle->spi_cs_port, adc_handle->spi_cs_pin, GPIO_PIN_SET);
}

/*Realiza la comunicación establecida para la escritura por el datasheet del MCP3913.
Primero envía un byte de control, y luego envía los bytes que se quieren escribir en el registro.*/
void MCP3913_Port_Write_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, const uint8_t* reg_value) {
  MCP3913_Control_Byte_t control;

  control.read = false;
  control.reg_address = reg_address;
  control.dev_address = adc_handle->dev_address;

  MCP3913_Port_CS_Low(adc_handle);

  HAL_SPI_Transmit((SPI_HandleTypeDef*)adc_handle->spi_handle, &control.byte, 1, HAL_MAX_DELAY);
  for(int i = 0; i < 3; i++) {
    HAL_SPI_Transmit((SPI_HandleTypeDef*)adc_handle->spi_handle, &reg_value[2-i], 1, HAL_MAX_DELAY);
  }

  MCP3913_Port_CS_High(adc_handle);
}

/*Realiza la comunicación establecida para la lectura por el datasheet del MCP3913.
Primero envía un byte de control, y luego lee los bytes del registro.*/
void MCP3913_Port_Read_N_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value, uint8_t n_bytes) {
  MCP3913_Control_Byte_t control;

  control.read = true;
  control.reg_address = reg_address;
  control.dev_address = adc_handle->dev_address;

  MCP3913_Port_CS_Low(adc_handle);

  HAL_SPI_Transmit((SPI_HandleTypeDef*)adc_handle->spi_handle, &control.byte, 1, HAL_MAX_DELAY);
  for(int i = 0; i < n_bytes; i++) {
    HAL_SPI_Receive((SPI_HandleTypeDef*)adc_handle->spi_handle, &reg_value[n_bytes-1-i], 1, HAL_MAX_DELAY);
  }

  MCP3913_Port_CS_High(adc_handle);
}

/*Lee un registro de 3 bytes. (Todos los registros del MCP3913 son de este tipo).*/
void MCP3913_Port_Read_3_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value) {
  MCP3913_Port_Read_N_Bytes_Reg(adc_handle, reg_address, reg_value, 3);
}

/*Lee un registro de 4 bytes. (Utilizado cuando se setea el adc en la expansión de signo del ADC).*/
void MCP3913_Port_Read_4_Bytes_Reg(const MCP3913_handle_t* adc_handle, uint8_t reg_address, uint8_t* reg_value) {
  MCP3913_Port_Read_N_Bytes_Reg(adc_handle, reg_address, reg_value, 4);
}
