#ifndef __MCP3913_H__
#define __MCP3913_H__

#include <stdint.h>
#include <stdbool.h>

/* Driver para el ADC MCP3913*/

#define MCP3913_DEFAULT_DEV_ADDRESS 0b01 //Address por default hardcodeado del MCP3913, al inicio de cada transmisión SPI se debe enviar el address del ADC.
#define MCP3913_PASSWORD 0xA5 //Palabra clave que debe escribirse en el registro lock_crc (0x1F) del ADC para que ecuche los comandos de escritura.
#define MCP3913_DEFAULT_VREF_CAL 0x50 //Valor de calibración para el desvío por temperatura que Microchip recomienda poner

// Mapa de Memoria MCP3914
#define MCP3913_CHANNEL0_REG_ADD      0x00 // Channel 0 ADC Data <23:0>, MSB first
#define MCP3913_CHANNEL1_REG_ADD      0x01 // Channel 1 ADC Data <23:0>, MSB first
#define MCP3913_CHANNEL2_REG_ADD      0x02 // Channel 2 ADC Data <23:0>, MSB first
#define MCP3913_CHANNEL3_REG_ADD      0x03 // Channel 3 ADC Data <23:0>, MSB first
#define MCP3913_CHANNEL4_REG_ADD      0x04 // Channel 4 ADC Data <23:0>, MSB first
#define MCP3913_CHANNEL5_REG_ADD      0x05 // Channel 5 ADC Data <23:0>, MSB first
#define MCP3913_MOD_REG_ADD           0x08 // Delta-Sigma Modulators Output Value
#define MCP3913_PHASE0_REG_ADD        0x09 // Phase Delay Configuration Register - Channel pairs 4/5
#define MCP3913_PHASE1_REG_ADD        0x0A // Phase Delay Configuration Register - Channel pairs 0/1 and 2/3
#define MCP3913_GAIN_REG_ADD          0x0B // Gain Configuration Register
#define MCP3913_STATUSCOM_REG_ADD     0x0C // Status and Communication Register
#define MCP3913_CONFIG0_REG_ADD       0x0D // Configuration Register
#define MCP3913_CONFIG1_REG_ADD       0x0E // Configuration Register
#define MCP3913_OFFCAL_CH0_REG_ADD    0x0F // Offset Correction Register - Channel 0
#define MCP3913_GAINCAL_CH0_REG_ADD   0x10 // Offset Correction Register - Channel 0
#define MCP3913_OFFCAL_CH1_REG_ADD    0x11 // Offset Correction Register - Channel 1
#define MCP3913_GAINCAL_CH1_REG_ADD   0x12 // Offset Correction Register - Channel 1
#define MCP3913_OFFCAL_CH2_REG_ADD    0x13 // Offset Correction Register - Channel 2
#define MCP3913_GAINCAL_CH2_REG_ADD   0x14 // Offset Correction Register - Channel 2
#define MCP3913_OFFCAL_CH3_REG_ADD    0x15 // Offset Correction Register - Channel 3
#define MCP3913_GAINCAL_CH3_REG_ADD   0x16 // Offset Correction Register - Channel 3
#define MCP3913_OFFCAL_CH4_REG_ADD    0x17 // Offset Correction Register - Channel 4
#define MCP3913_GAINCAL_CH4_REG_ADD   0x18 // Offset Correction Register - Channel 4
#define MCP3913_OFFCAL_CH5_REG_ADD    0x19 // Offset Correction Register - Channel 5
#define MCP3913_GAINCAL_CH5_REG_ADD   0x1A // Offset Correction Register - Channel 5
#define MCP3913_LOCK_CRC_REG_ADD      0x1F // Security Register (Password and CRC-16 on Register Map)

/*Ganancia de los PGA:
GAINX1 = Ganacia x1 (default)
GAINX2 = Ganacia x2
GAINX4 = Ganacia x4
GAINX8 = Ganacia x8
GAINX16 = Ganacia x16
GAINX32 = Ganacia x32
*/
typedef enum { GAINX1 = 0, GAINX2 = 1, GAINX4 = 2, GAINX8 = 3, GAINX16 = 4, GAINX32 = 5 } MCP3913_Gain_t;

/*Multiplicador de la corriente de polarización del circuito del ADC, a mayor corriente hay mayor consumo pero permite que el ADC opere a mayor frecuencia.
Multiplicadores:
BOOSTX05 = Multiplica la corriente de polarización de todos los canales x0.5
BOOSTX066 = Multiplica la corriente de polarización de todos los canales x0.66
BOOSTX1 = Multiplica la corriente de polarización de todos los canales x1.0 (default)
BOOSTX2 = Multiplica la corriente de polarización de todos los canales x2.0
*/
typedef enum { BOOSTX05 = 0, BOOSTX066 = 1, BOOSTX1 = 2, BOOSTX2 = 3 } MCP3913_Boost_t;

/*Ratio de oversampling para los modulardores delta-sigma:
O32 = 32 bits de oversampling
O64 = 64 bits de oversampling
O128 = 128 bits de oversampling
O256 = 256 bits de oversampling (default)
O512 = 512 bits de oversampling
O1024 = 1024 bits de oversampling
O2048 = 2048 bits de oversampling
O4096 = 4096 bits de oversampling
*/
typedef enum { O32 = 0, O64 = 1, O128 = 2, O256 = 3, O512 = 4, O1024 = 5, O2048 = 6, O4096 = 7 } MCP3913_Oversampling_t;

/*Divisores del clock maestro que setean el clock de los ADCs:
MCLK1 = Clock de los ADCs seteados en MCLK (default)
MCLK2 = Clock de los ADCs seteados en MCLK/2
MCLK4 = Clock de los ADCs seteados en MCLK/4
MCLK8 = Clock de los ADCs seteados en MCLK/8
*/
typedef enum { MCLK1 = 0, MCLK2 = 1, MCLK4 = 2, MCLK8 = 3} MCP3913_Prescale_t;

/*Método de loopeo de registros cuando se hace lectura continua:
REGISTER = Se loopea en el mismo registro, continuamente se lee la misma posición.
GROUP = Se loopea por lo que el fabricante del ADC define como "GROUP", en general suelen ser pares de registros contiguos (ej: se loopea entre el canal 0 y canal 1)
TYPE = Se loopea por lo que es configuración, o lo que es canal, es útil para seteo rápido de configuraciones y lectura de todos los canales del ADC.
ALL = Se loopea todo el mapa de registros completo.
*/
typedef enum { REGISTER = 0, GROUP = 1, TYPE = 2, ALL = 3 } MCP3913_Looping_t;

/*Los moduladores sigma-delta tienen el problema de que cuando vos tenés DC en la entrada, pueden generar mediciones periódicas en la salida.
Esto es malo para los medidores porque la frecuencia de la onda periódica que genera suele estar alrededor de los 50Hz/60Hz.
El dither es un mecanismo por el cual el ADC agrega ruido a la salida para descorrelacionar este problema y que este ruido disperse este problemas a todas las frecuencias:
OFF = Dither apagado
MIN = Dither encendido al nivel mínimo
MED = Dither encendido al nivel medio
MAX = Dither encendido al nivel máximo
*/
typedef enum { OFF = 0, MIN = 1, MED = 2, MAX = 3 } MCP3913_Dither_t;

/*Cantidad de bits del CRC:
CRC_16_BITS = CRC de 16 bits
CRC_32_BITS = CRC de 32 bits
*/
typedef enum { CRC_16_BITS = 0, CRC_32_BITS = 1 } MCP3913_Width_CRC_t;

/*
Formato de datos en la lectura del ADC:
BITS_16 = 16-bit (with rounding)
BITS_24 = 24-bit (default)
BITS_32_ZP = 32-bit with zeros padding
BITS_32_SE = 32-bit with sign extension
*/
typedef enum { WIDTH_16_BITS = 0, WIDTH_24_BITS = 1, WIDTH_32_BITS_ZP = 2, WIDTH_32_BITS_SE = 3 } MCP3913_Width_Data_t;

/* Phase0 Register:
phasec = Retardo de fase entre los canales ch4 y ch5
*/
typedef union{
  struct __attribute__ ((packed)) {
    uint16_t phasec :12;
    uint16_t :12; //msb
  };
  uint8_t byte[3]; // 0 = lsB 2 = msB
} MCP3913_Phase0_Reg_t;

 /* Phase Register:
phasea = Retardo de fase entre los canales ch0 y ch1
phaseb = Retardo de fase entre los canales ch2 y ch3
*/
typedef union {
  struct __attribute__ ((packed)) {
  uint16_t phasea :12;
  uint16_t phaseb :12; //msb
  };
  uint8_t byte[3]; // 0 = lsB 2 = msB
} MCP3913_Phase1_Reg_t;

/*Gain Register:
ch0: Ganancia del canal 0
ch1: Ganancia del canal 1
ch2: Ganancia del canal 2
ch3: Ganancia del canal 3
ch4: Ganancia del canal 4
ch5: Ganancia del canal 5
*/
typedef union{
  struct __attribute__ ((packed)) {
    MCP3913_Gain_t ch0 :3;
    MCP3913_Gain_t ch1 :3;
    MCP3913_Gain_t ch2 :3;
    MCP3913_Gain_t ch3 :3;
    MCP3913_Gain_t ch4 :3;
    MCP3913_Gain_t ch5 :3;
    uint8_t :6; //msb
  };
  uint8_t byte[3]; // 0 = lsB 2 = msB
} MCP3913_Gain_Reg_t;

/* Status register:
ch0_not_ready = devuelve true cuando el canal 0 no se encuentra listo devolver muestras.
ch1_not_ready = devuelve true cuando el canal 1 no se encuentra listo devolver muestras.
ch2_not_ready = devuelve true cuando el canal 2 no se encuentra listo devolver muestras.
ch3_not_ready = devuelve true cuando el canal 3 no se encuentra listo devolver muestras.
ch4_not_ready = devuelve true cuando el canal 4 no se encuentra listo devolver muestras.
ch5_not_ready = devuelve true cuando el canal 5 no se encuentra listo devolver muestras.
en_int = {
  true = Se activa el flag de interrupción para verificación de CRC. Si hay error de CRC, DR se pone en 0 y se mantiene en 0 hasta que se escriba la palabra clave 0xA5 en el registro LockCRCReg.lock.
  false = Se desactiva el flag de interrupción, DR no se pone en 0 en caso de error. (default)
}
en_crc_com = {
  true = Se agrega un checksum CRC-16 al final de cada mensaje (los mensajes van a ser más largos).
  false = Deshabilitado. (default)
}
width_data = Formato de datos del ADC.
width_crc = Tamaño de la cantidad de bits del crc.
dr_link = {
  true = Se genera un pulso low en el pin DR cuando todos los canales están listos para ser leídos.
  false = Cada canal envía un pulso low en DR independientemente del resto.
}
dr_high = {
  false = El pin DR está en alta impedancia cuando no hay lecturas disponibles.
  true = El pin DR está en high cuando no hay lecturas disponibles.
}
write_reg_incr = {
  true = Autoincrementa el contador de registros para loopear en las escrituras continuas.
  false = No autoincrementa, se queda escribiendo el mismo registro en escrituras continuas.
}
read_reg_incr = Setea los distintos tipos de loopeos de lectura
*/
typedef union{
  struct __attribute__ ((packed)) {
    uint8_t channel_byte :6;
    uint32_t :18; //msb
  };

  struct __attribute__ ((packed)) {
    bool ch0_not_ready :1;
    bool ch1_not_ready :1;
    bool ch2_not_ready :1;
    bool ch3_not_ready :1;
    bool ch4_not_ready :1;
    bool ch5_not_ready :1;
    uint8_t :6;
    uint8_t reserved :2; // Estos dos bits deben ser SIEMPRE cero. Sino no funciona el ADC
    bool en_int :1;
    bool en_crc_com :1;
    MCP3913_Width_Data_t width_data :2;
    MCP3913_Width_CRC_t width_crc :1;
    bool dr_link :1;
    bool dr_high :1;
    bool write_reg_incr :1;
    MCP3913_Looping_t read_reg_incr :2; //msb
  };
  uint8_t byte[3]; // 0 = lsB 2 = msB
} MCP3913_StatusCom_Reg_t;

/*Config 0
vref_cal = Coeficiente de calibración por temperatura, el valor que se recomienda setear está guardado en MCP3913_DEFAULT_VREF_CAL.
osr = Oversampling ratio
pre = Prescaler del analog master clock
boost = Selección de corriente de polarización
dither = Dithering para cancelación de tonos de DC.
en_gaincal = Habilita la calibración de ganancia en todos los canales. Esto agrega un delay de 24DMCLK en todos los canales.
en_offcal = Habilita la calibración de offset en todos los canales.
*/
typedef union{
  struct __attribute__ ((packed)) {
    uint8_t vref_cal :8;
    uint8_t :5;
    MCP3913_Oversampling_t osr :3;
    MCP3913_Prescale_t pre :2;
    MCP3913_Boost_t boost :2;
    MCP3913_Dither_t dither :2;
    bool en_gaincal :1;
    bool en_offcal :1; //msb
  };
  uint8_t byte[3]; // 0 = lsB 2 = msB
} MCP3913_Config0_Reg_t;


/*Config 1
clK_ext = {
  true = clock externo.
  false = clock generado por el cristal.
}
vref_ext = {
  true = tensión de referencia externa.
  false = tensión de referencia interna.
}
ch0_shutdown = Pone al canal 0 en modo shutdown
ch1_shutdown = Pone al canal 1 en modo shutdown
ch2_shutdown = Pone al canal 2 en modo shutdown
ch3_shutdown = Pone al canal 3 en modo shutdown
ch4_shutdown = Pone al canal 4 en modo shutdown
ch5_shutdown = Pone al canal 5 en modo shutdown
ch0_reset = Pone al canal 0 en modo soft reset
ch1_reset = Pone al canal 1 en modo soft reset
ch2_reset = Pone al canal 2 en modo soft reset
ch3_reset = Pone al canal 3 en modo soft reset
ch4_reset = Pone al canal 4 en modo soft reset
ch5_reset = Pone al canal 5 en modo soft reset
*/
typedef union {
  struct __attribute__ ((packed)) {
    uint8_t :6;
    bool clk_ext :1;
    bool vref_ext :1;
    bool ch0_shutdown :1;
    bool ch1_shutdown :1;
    bool ch2_shutdown :1;
    bool ch3_shutdown :1;
    bool ch4_shutdown :1;
    bool ch5_shutdown :1;
    uint8_t :2;
    bool ch0_reset :1;
    bool ch1_reset :1;
    bool ch2_reset :1;
    bool ch3_reset :1;
    bool ch4_reset :1;
    bool ch5_reset :1;
    uint8_t :2; //msb

  };
  uint8_t byte[3]; // 0 = lsB 2 = msB
} MCP3913_Config1_Reg_t;

/*Lock/CRC
crc_reg = Devuelve el cálculo del crc.
lock = Se le debe escribir el valor definido en MCP3913_PASSWORD para que permita la escritura en el resto de los registros
*/
typedef union {
  struct __attribute__ ((packed)) {
    uint16_t crc_reg :16;
    uint8_t lock :8; //msb
  };
  uint8_t byte[3]; // 0 = lsB 2 = msB
} MCP3913_Lock_CRC_Reg_t;

/* Calibración de offset*/
typedef union {
  struct __attribute__ ((packed)) {
      uint32_t value :24;
  };
  uint8_t byte[3]; // 0 = lsB 2 = msB
} MCP3913_OffCal_Reg_t;

/* Calibración de ganancia*/
typedef union {
  struct __attribute__ ((packed)) {
      uint32_t value :24;
  };
  uint8_t byte[3];
} MCP3913_GainCal_Reg_t;

/*Estructura del byte de control:
Address del ADC [7:6] | Registro [5:1] | En 1 indica lectura, 0 escritura [0]
*/
typedef union {
  struct __attribute__ ((packed)) {
    bool read :1;
    uint8_t reg_address :5;
    uint8_t dev_address :2; // msb
  };
  uint8_t byte;
} MCP3913_Control_Byte_t;

/*Registro de modulación, posee las salidas de los moduladores de cada canal:
comp_ch0 = Salidas de los comparadores del canal 0
comp_ch1 = Salidas de los comparadores del canal 1
comp_ch2 = Salidas de los comparadores del canal 2
comp_ch3 = Salidas de los comparadores del canal 3
comp_ch4 = Salidas de los comparadores del canal 4
comp_ch5 = Salidas de los comparadores del canal 5
*/
typedef union{
  struct __attribute__ ((packed)) {
    uint8_t comp_ch0 :4;
    uint8_t comp_ch1 :4;
    uint8_t comp_ch2 :4;
    uint8_t comp_ch3 :4;
    uint8_t comp_ch4 :4;
    uint8_t comp_ch5 :4; // msb
  };
  uint8_t byte[3]; // 0 = lsB 2 = msB
} MCP3913_Mod_Reg_t;

/*Valor lectura de canal de ADC
*/
typedef union {
  uint8_t byte[4];
  int32_t value;
} MCP3913_Channel_Reg_t;

/**
  * @brief Estructura de configuración del MCP3913
  */
typedef struct __MCP3913_handle_t {
    void * spi_handle;
    void * spi_cs_port;
    uint16_t spi_cs_pin;
    uint8_t dev_address;
    MCP3913_Phase0_Reg_t phase0_reg;
    MCP3913_Phase1_Reg_t phase1_reg;
    MCP3913_Gain_Reg_t gain_reg;
    MCP3913_StatusCom_Reg_t statuscom_reg;
    MCP3913_Config0_Reg_t config0_reg;
    MCP3913_Config1_Reg_t config1_reg;
    MCP3913_Lock_CRC_Reg_t lock_crc_reg;
    MCP3913_OffCal_Reg_t offcal_ch0_reg;
    MCP3913_GainCal_Reg_t gaincal_ch0_reg;
    MCP3913_OffCal_Reg_t offcal_ch1_reg;
    MCP3913_GainCal_Reg_t gaincal_ch1_reg;
    MCP3913_OffCal_Reg_t offcal_ch2_reg;
    MCP3913_GainCal_Reg_t gaincal_ch2_reg;
    MCP3913_OffCal_Reg_t offcal_ch3_reg;
    MCP3913_GainCal_Reg_t gaincal_ch3_reg;
    MCP3913_OffCal_Reg_t offcal_ch4_reg;
    MCP3913_GainCal_Reg_t gaincal_ch4_reg;
    MCP3913_OffCal_Reg_t offcal_ch5_reg;
    MCP3913_GainCal_Reg_t gaincal_ch5_reg;
} MCP3913_handle_t;

/* Carga las configuraciones por defecto a la estrucuta MCP3913_handle_t apuntada. No inicializa el ADC */
void MCP3913_Load_Default_Config(MCP3913_handle_t* adc_handle);
/* Inicializa el ADC */
void MCP3913_Init_ADC(const MCP3913_handle_t* adc_handle);
/* Lee la medicion del canal solicitado. Si se desea leer más de un canal, es más eficiente usar MCP3913_Read_All_Channels */
void MCP3913_Read_Channel(const MCP3913_handle_t* adc_handle, uint8_t channel, int32_t * value);
/* Lee la medicion de todos los canales en una sola operación. Esto es más rápido porque porque cuando consulta el registro STATUSCOM y los DRSTATUS bits estan en cero, automaticamente lee todos los canales en una sola operación. */
void MCP3913_Read_All_Channels(const MCP3913_handle_t* adc_handle, int32_t * values);

#endif
