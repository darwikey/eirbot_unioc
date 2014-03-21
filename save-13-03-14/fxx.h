#ifndef FXX_H
#define FXX_H


#define DOUBLE

#ifdef F32
#include <f32.h>
typedef f32 fxx;
#elif defined DOUBLE
typedef double fxx;
#else
#error "No fxx configuration"
#endif


fxx fxx_mul(fxx a,fxx b);
fxx fxx_div(fxx a,fxx b);
fxx fxx_inv(fxx a);
fxx fxx_from_double(double a);
double fxx_to_double(fxx a);

#endif //FXX_H
