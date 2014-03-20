#ifdef TEST

#ifndef TEST_H
#define TEST_H

// Compatibilite de type
#define uint32_t unsigned long
#define int32_t  long
#define uint16_t unsigned short
#define int16_t  short
#define uint8_t unsigned char
#define int8_t signed char


// Dans position_manager.h
struct pos {
  int16_t x;
  int16_t y;
  int16_t a;
};


// STD LIB
#define NULL ((void*)0)

#include <stdio.h>
#define PRINT_DBG(...)			\
  printf(__VA_ARGS__)

#endif//TEST_H

#else
#define PRINT_DBG
#endif
