#include "MCP3913_LL_port.h"
#include "stm32g0xx_ll_spi.h"
#include "stm32g0xx_ll_gpio.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * @brief Setea el Chip Select del SPI en LOW
 * @param adc_handle Puntero al handle del ADC
 */
void MCP3913_Port_CS_Low(const MCP3913_handle_t* adc_handle) {
  LL_GPIO_ResetOutputPin((GPIO_TypeDef *)adc_handle->spi_cs_port, adc_handle->spi_cs_pin); // Activo el CS
}

/*
 * @brief Setea el Chip Select del SPI en HIGH
 * @param adc_handle Puntero al handle del ADC
 */
void MCP3913_Port_CS_High(const MCP3913_handle_t* adc_handle) {
  LL_GPIO_SetOutputPin((GPIO_TypeDef *)adc_handle->spi_cs_port, adc_handle->spi_cs_pin); // Activo el CS
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

  while(!LL_SPI_IsActiveFlag_TXE((SPI_TypeDef *)adc_handle->spi_handle)); // Espero si el SPI esta ocupado
  LL_SPI_TransmitData8((SPI_TypeDef *)adc_handle->spi_handle, control.byte); // Envio el CONTROL BYTE
  for(int i = 0; i < 3; i++) {
    while(!LL_SPI_IsActiveFlag_TXE((SPI_TypeDef *)adc_handle->spi_handle)); // Espero si el SPI esta ocupado
    LL_SPI_TransmitData8((SPI_TypeDef *)adc_handle->spi_handle, reg_value[2-i]); // Escribo registro del ADC
  }

  // Vacio el buffer de recepcion
  while(LL_SPI_IsActiveFlag_RXNE((SPI_TypeDef *)adc_handle->spi_handle)) LL_SPI_ReceiveData8((SPI_TypeDef *)adc_handle->spi_handle);

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

  while(!LL_SPI_IsActiveFlag_TXE((SPI_TypeDef *)adc_handle->spi_handle)); // Espero a que el buffer de salida de SPI este vacio
  LL_SPI_TransmitData8((SPI_TypeDef *)adc_handle->spi_handle, control.byte); // Envio el CONTROL BYTE

  // Vacio el buffer de recepcion
  while(LL_SPI_IsActiveFlag_RXNE((SPI_TypeDef *)adc_handle->spi_handle)) LL_SPI_ReceiveData8((SPI_TypeDef *)adc_handle->spi_handle);

  // Comienzo proceso de recepcion
  for(int i = 0; i < n_bytes; i++) {
    // Transmito dummy data
    while(!LL_SPI_IsActiveFlag_TXE((SPI_TypeDef *)adc_handle->spi_handle)); // Espero a que el buffer de salida de SPI este vacio
    LL_SPI_TransmitData8((SPI_TypeDef *)adc_handle->spi_handle, 0x00); // Envio dummy data

    while(!LL_SPI_IsActiveFlag_RXNE((SPI_TypeDef *)adc_handle->spi_handle)); // Espero si el SPI esta ocupado
    reg_value[n_bytes-1-i] = LL_SPI_ReceiveData8((SPI_TypeDef *)adc_handle->spi_handle); // Leo el o los registros del ADC
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
