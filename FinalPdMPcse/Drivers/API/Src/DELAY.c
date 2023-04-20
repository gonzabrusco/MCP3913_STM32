
#include "DELAY.h"
#include "stm32g0xx_hal.h"  		/* <- HAL include */

/*
 * @brief Inicializa un delay
 * @param [in] delay: puntero a la variable que almacena el delay
 * @param [in] duration: la duracion del delay
 */
void DELAY_Init( delay_t * delay, tick_t duration ) {
	if(delay == NULL) return; // Chequeo de parametros
	delay->duration = duration; // Guardo duracion
	delay->running = false; // Inicializa frenado
}

/*
 * @brief Lee un delay
 * @param [in] delay: puntero a la variable que almacena el delay
 * @param [in] duration: la duracion del delay
 * @return true si se cumplio el delay, false en caso contrario
 */
bool_t DELAY_Read( delay_t * delay ) {
	if(delay == NULL) return false; // Chequeo de parametros
	if(!delay->running) {
		// No esta ejecutandose, tomo marca de tiempo y lo marco como corriendo
		delay->start_time = HAL_GetTick();
		delay->running = true;
	}
	else if(HAL_GetTick() - delay->start_time >= delay->duration) {
		// Se cumplio el tiempo, marco el delay como frenado y retorno avisando que termino el tiempo.
		delay->running = false;
		return true;
	}
	return false;
}

/*
 * @brief Modifica la duracion de un delay
 * @param [in] delay: puntero a la variable que almacena el delay
 * @param [in] duration: la duracion del delay
 */
void DELAY_Write( delay_t * delay, tick_t duration ) {
	if(delay == NULL) return; // Chequeo de parametros
	delay->duration = duration; // Actualizo la duracion
}
