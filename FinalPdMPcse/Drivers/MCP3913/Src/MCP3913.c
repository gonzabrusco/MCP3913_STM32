#include "MCP3913.h"
#include "MCP3913_port.h"

/* Carga las configuraciones por defecto a la estrucuta MCP3913_handle_t apuntada. No inicializa el ADC */
void MCP3913_Load_Default_Config(MCP3913_handle_t* adc_handle) {
  //Configuración del registro de lock
  adc_handle->lock_crc_reg.lock = MCP3913_PASSWORD;
  adc_handle->lock_crc_reg.crc_reg = 0;

  //Configuración por default del registro Config 0
  adc_handle->config0_reg.boost = BOOSTX1;
  adc_handle->config0_reg.dither = MAX;
  adc_handle->config0_reg.vref_cal = MCP3913_DEFAULT_VREF_CAL;
  adc_handle->config0_reg.osr = O256;
  adc_handle->config0_reg.pre = O256;
  adc_handle->config0_reg.en_gaincal = false;
  adc_handle->config0_reg.en_offcal = false;

  //Configuración por default del registro Config 1
  adc_handle->config1_reg.ch0_reset = false;
  adc_handle->config1_reg.ch0_shutdown = false;
  adc_handle->config1_reg.ch1_reset = false;
  adc_handle->config1_reg.ch1_shutdown = false;
  adc_handle->config1_reg.ch2_reset = false;
  adc_handle->config1_reg.ch2_shutdown = false;
  adc_handle->config1_reg.ch3_reset = false;
  adc_handle->config1_reg.ch3_shutdown = false;
  adc_handle->config1_reg.ch4_reset = false;
  adc_handle->config1_reg.ch4_shutdown = false;
  adc_handle->config1_reg.ch5_reset = false;
  adc_handle->config1_reg.ch5_shutdown = false;
  adc_handle->config1_reg.clk_ext = false;
  adc_handle->config1_reg.vref_ext = false;

  //Configuración por default del registro de status
  adc_handle->statuscom_reg.dr_high = false;
  adc_handle->statuscom_reg.dr_link = false;
  adc_handle->statuscom_reg.en_crc_com = false;
  adc_handle->statuscom_reg.en_int = false;
  adc_handle->statuscom_reg.read_reg_incr = TYPE;
  adc_handle->statuscom_reg.width_crc = CRC_16_BITS;
  adc_handle->statuscom_reg.width_data = WIDTH_32_BITS_SE;
  adc_handle->statuscom_reg.write_reg_incr = false;
  adc_handle->statuscom_reg.ch0_not_ready = true;
  adc_handle->statuscom_reg.ch1_not_ready = true;
  adc_handle->statuscom_reg.ch2_not_ready = true;
  adc_handle->statuscom_reg.ch3_not_ready = true;
  adc_handle->statuscom_reg.ch4_not_ready = true;
  adc_handle->statuscom_reg.ch5_not_ready = true;

  //Configuración por default del registro de ganancias
  adc_handle->gain_reg.ch0 = GAINX1;
  adc_handle->gain_reg.ch1 = GAINX1;
  adc_handle->gain_reg.ch2 = GAINX1;
  adc_handle->gain_reg.ch3 = GAINX1;
  adc_handle->gain_reg.ch4 = GAINX1;
  adc_handle->gain_reg.ch5 = GAINX1;

  // Configuracion de los registros de calibracion. Todos en cero por default.
  adc_handle->offcal_ch0_reg.value = 0;
  adc_handle->gaincal_ch0_reg.value = 0;
  adc_handle->offcal_ch1_reg.value = 0;
  adc_handle->gaincal_ch1_reg.value = 0;
  adc_handle->offcal_ch2_reg.value = 0;
  adc_handle->gaincal_ch2_reg.value = 0;
  adc_handle->offcal_ch3_reg.value = 0;
  adc_handle->gaincal_ch3_reg.value = 0;
  adc_handle->offcal_ch4_reg.value = 0;
  adc_handle->gaincal_ch4_reg.value = 0;
  adc_handle->offcal_ch5_reg.value = 0;
  adc_handle->gaincal_ch5_reg.value = 0;
}

/* Inicializa el ADC con las configuraciones pasadas */
void MCP3913_Init(const MCP3913_handle_t* adc_handle) {

  // Desbloqueo los registros del ADC escribiendo el password en LOCK
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_LOCK_CRC_REG_ADD, adc_handle->lock_crc_reg.byte);

  MCP3913_Port_Write_Reg(adc_handle, MCP3913_PHASE0_REG_ADD, adc_handle->phase0_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_PHASE1_REG_ADD, adc_handle->phase1_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_GAIN_REG_ADD, adc_handle->gain_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_STATUSCOM_REG_ADD, adc_handle->statuscom_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_CONFIG0_REG_ADD, adc_handle->config0_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_CONFIG1_REG_ADD, adc_handle->config1_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_OFFCAL_CH0_REG_ADD, adc_handle->offcal_ch0_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_GAINCAL_CH0_REG_ADD, adc_handle->gaincal_ch0_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_OFFCAL_CH1_REG_ADD, adc_handle->offcal_ch1_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_GAINCAL_CH1_REG_ADD, adc_handle->gaincal_ch1_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_OFFCAL_CH2_REG_ADD, adc_handle->offcal_ch2_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_GAINCAL_CH2_REG_ADD, adc_handle->gaincal_ch2_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_OFFCAL_CH3_REG_ADD, adc_handle->offcal_ch3_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_GAINCAL_CH3_REG_ADD, adc_handle->gaincal_ch3_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_OFFCAL_CH4_REG_ADD, adc_handle->offcal_ch4_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_GAINCAL_CH4_REG_ADD, adc_handle->gaincal_ch4_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_OFFCAL_CH5_REG_ADD, adc_handle->offcal_ch5_reg.byte);
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_GAINCAL_CH5_REG_ADD, adc_handle->gaincal_ch5_reg.byte);

  // Bloqueo los registros del ADC borrando el password en LOCK.
  // Creo una variable auxiliar para no modificar la estructura pasada por el usuario.
  MCP3913_Lock_CRC_Reg_t lock_crc_reg = adc_handle->lock_crc_reg;
  lock_crc_reg.lock = 0;
  MCP3913_Port_Write_Reg(adc_handle, MCP3913_LOCK_CRC_REG_ADD, lock_crc_reg.byte);
}

/* Lee la medicion del canal solicitado */
int32_t MCP3913_Read_Channel(const MCP3913_handle_t* adc_handle, uint8_t channel) {
  MCP3913_Channel_Reg_t channel_read;
  MCP3913_StatusCom_Reg_t statuscom_reg;

  channel_read.value = 0;

  do {
    // Leo el estado del ADC
    MCP3913_Port_Read_3_Bytes_Reg(adc_handle, MCP3913_STATUSCOM_REG_ADD, statuscom_reg.byte);
  } while(statuscom_reg.channel_byte & (1 << channel)); //Se espera hasta que el bit numero 'ch' del byte de canales esté en 0

  MCP3913_Port_Read_4_Bytes_Reg(adc_handle, channel, channel_read.byte);

  return channel_read.value;
}



