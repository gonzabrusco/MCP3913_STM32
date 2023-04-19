# Driver para el ADC MCP3913 usando un STM32

El integrado MCP3913 es un ADC con 6 canales diferenciales con 24 bits de resolución, cada uno con su PGA configurable, bloque de compensación de fase, filtro sinc y etapa de corrección de offset y ganancia (también llamada calibración).

La comunicación con el MCP3913 es por SPI y la configuración del mismo es mediante la escritura de registros de 24 bits.

La lectura de los canales de ADC puede ser de a un canal a la vez (se inicia una comunicacion SPI nueva por cada lectura) o leyendo de forma continua todos los canales (se obtienen todos los canales en una sola comunicacion SPI).

El driver presente en este repositorio provee funciones para configurar e inicializar el ADC, como así también para adquirir las muestras de un canal por vez o de todos los canales de una sola vez. 
