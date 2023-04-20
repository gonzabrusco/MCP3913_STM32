
#ifndef _API_DELAY_H_
#define _API_DELAY_H_

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t tick_t;
typedef bool bool_t;
typedef struct{
   tick_t start_time;
   tick_t duration;
   bool_t running;
} delay_t;

/*
 * @brief Inicializa un delay
 * @param [in] delay: puntero a la variable que almacena el delay
 * @param [in] duration: la duracion del delay
 */
void DELAY_Init( delay_t * delay, tick_t duration );
/*
 * @brief Lee un delay
 * @param [in] delay: puntero a la variable que almacena el delay
 * @param [in] duration: la duracion del delay
 * @return true si se cumplio el delay, false en caso contrario
 */
bool_t DELAY_Read( delay_t * delay );
/*
 * @brief Modifica la duracion de un delay
 * @param [in] delay: puntero a la variable que almacena el delay
 * @param [in] duration: la duracion del delay
 */
void DELAY_Write( delay_t * delay, tick_t duration );

#endif /* API_API_DELAY_H_ */
