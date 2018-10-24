/* A simple LUT logf.  */

#define NW 64
#define LN2 0.6931471805599453f
#define LOG10_E 0.4342944819032518f

/* The table is generated with a python3 snipet:
import math

nw = 64
delta = 1/nw
ltable = [math.log(1+i*delta) for i in range(0,nw)]
print("static float stable[NW] = {")
for i in range(0,nw):
    print(" ", '{:#.8f}'.format(ltable[i]), end=',')
    if ((i+1)%4 == 0):
        print()

print("};")
*/

static const float ltable[NW] = {
  0.00000000,  0.01550419,  0.03077166,  0.04580954,
  0.06062462,  0.07522342,  0.08961216,  0.10379679,
  0.11778304,  0.13157636,  0.14518201,  0.15860503,
  0.17185026,  0.18492234,  0.19782574,  0.21056477,
  0.22314355,  0.23556607,  0.24783616,  0.25995752,
  0.27193372,  0.28376817,  0.29546421,  0.30702504,
  0.31845373,  0.32975329,  0.34092659,  0.35197642,
  0.36290549,  0.37371641,  0.38441170,  0.39499381,
  0.40546511,  0.41582790,  0.42608440,  0.43623677,
  0.44628710,  0.45623743,  0.46608973,  0.47584590,
  0.48550782,  0.49507727,  0.50455601,  0.51394575,
  0.52324814,  0.53246480,  0.54159728,  0.55064712,
  0.55961579,  0.56850474,  0.57731537,  0.58604905,
  0.59470711,  0.60329085,  0.61180154,  0.62024041,
  0.62860866,  0.63690746,  0.64513796,  0.65330127,
  0.66139848,  0.66943065,  0.67739882,  0.68530400,
};

/* log kernel function with LUT
   Assume that 1 <= x < 2.  */
static float
kernel_lut_logf (float x)
{
  int k;
  float c, r, r2, r3, d;
  if (x < 1 || x >= 2)
    {
      while (1) ;
    }
  if (x < 1.0644944589178593f)
    {
      float y = x - 1;
      float y2 = y * y;
      return y - (1/2)*y2 + (1/3)*y2*y - (1/4)*y2*y2;
    }
  k = (int)((x - 1)*NW);
  if (x - k/NW > 1/(2*NW))
    k = k + 1;
  c = 1 + k/NW;
  r = (x - c)/(x + c);
  r2 = r * r;
  r3 = r2 * r;
  d = 2 * (r + (1/3)*r3 + (1/5)*r3*r2);
  return ltable[k] + d;
}

float
logf (float x)
{
  int e;
  union { float x; unsigned int r;} u;
  u.x = x;
  e = ((u.r & 0x7f800000) >> 23) - 127;
  if ((u.r & ~0xff800000) == 0)
    return -3.402823466e+38f;
  // 2 * mantissa
  u.r = (u.r & ~0xff800000) | 0x3ff80000;
  x = u.x;
  return kernel_lut_logf (x) + (e - 1) * LN2;
}

float
log10f (float x)
{
  return logf (x) * LOG10_E;
}

#if 0
#include <math.h>

int main ()
{
  int i;
  for (i = 0; i < 4096; i++)
    {
      float l;
      float x = 1 + i/4096;
      l = lut_logf (x);
      printf ("%f\n", (double)l - log(x));
    }
  return 0;
}
#endif
