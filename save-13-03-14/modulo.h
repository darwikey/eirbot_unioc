#ifndef MODULO_H
#define MODULO_H

#include <stdint.h>

/**
 * Process nb mod modulant for int8_t
 * with respect of nb > -modulant and nb < INT8_T_MAX - modulant
 **/
int8_t modulo_fast_8(int8_t nb, int8_t modulant);

/**
 * Process nb mod modulant for int8_t
 * Safe way, Slow way
 **/
int8_t modulo_safe_8(int8_t nb, int8_t modulant);

/**
 * Process nb mod modulant for int16_t
 * with respect of nb > -modulant and nb < INT16_T_MAX - modulant
 **/
int16_t modulo_fast_16(int16_t nb, int16_t modulant);

/**
 * Process nb mod modulant for int16_t
 * Safe way, Slow way
 **/
int16_t modulo_safe_16(int16_t nb, int16_t modulant);

/**
 * Process nb mod modulant for int32_t
 * with respect of nb > -modulant and nb < INT32_T_MAX - modulant
 **/
int32_t modulo_fast_32(int32_t nb, int32_t modulant);

/**
 * Process nb mod modulant for int32_t
 * Safe way, Slow way
 **/
int32_t modulo_safe_32(int32_t nb, int32_t modulant);

#endif /* MODULO_H */

