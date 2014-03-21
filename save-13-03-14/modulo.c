#include <stdint.h>
#include "modulo.h"

int8_t modulo_fast_8(int8_t nb, int8_t modulant)
{
  return (nb + modulant) % modulant;
}

int8_t modulo_safe_8(int8_t nb, int8_t modulant)
{
  while(nb < 0)
    nb += modulant;
  return nb % modulant;
}

int16_t modulo_fast_16(int16_t nb, int16_t modulant)
{
  return (nb + modulant) % modulant;
}

int16_t modulo_safe_16(int16_t nb, int16_t modulant)
{
  while(nb < 0)
    nb += modulant;
  return nb % modulant;
}

int32_t modulo_fast_32(int32_t nb, int32_t modulant)
{
  return (nb + modulant) % modulant;
}

int32_t modulo_safe_32(int32_t nb, int32_t modulant)
{
  while(nb < 0)
    nb += modulant;
  return nb % modulant;
}

