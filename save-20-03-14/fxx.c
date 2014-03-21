#include "fxx.h"

#ifdef F32

inline fxx fxx_mul(fxx a,fxx b)
{
  return f32_mul(a,b);
}
inline fxx fxx_div(fxx a,fxx b)
{
  return f32_div(a,b);
}
inline fxx fxx_inv(fxx a)
{
  return f32_inv(a);
}

inline fxx fxx_from_double(double a)
{
  return f32_from_double(a);
}

inline double fxx_to_double(fxx a)
{
  return f32_to_double(a);
}

#elif defined DOUBLE

inline fxx fxx_mul(fxx a,fxx b)
{
  return a * b;
}
inline fxx fxx_div(fxx a,fxx b)
{
  return a/b;
}
inline fxx fxx_inv(fxx a)
{
  return 1/a;
}

inline fxx fxx_from_double(fxx a)
{
  return a;
}

inline fxx fxx_to_double(fxx a)
{
  return a;
}

#endif
