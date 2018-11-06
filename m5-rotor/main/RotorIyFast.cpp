// Copyright (C) 2018 kaz Kojima
//
// This file is part of RotorIy program.  This program is free
// software; you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the
// Free Software Foundation; either version 3, or (at your option)
// any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program;
// see the file COPYING.

#include <cstdint>
#include <cstdio>
//#include <cmath>

#define ENABLE_3RD_ORDER_INTERPOLATION 0

#include "RotorIyFast.h"

RotorIy::RotorIy ()
{
  m_Ix = m_Iy = m_Iz = 0.0;
  m_S0 = 1.0;
  m_Si = m_Sj = m_Sk = 0.0;
#if ENABLE_3RD_ORDER_INTERPOLATION
  m_omega0x = m_omega0y = m_omega0z = 0.0;
  m_omega1x = m_omega1y = m_omega1z = 0.0;
#endif
  m_gain = 0.01;
}

RotorIy::RotorIy (float gain, float dt, float epsilon):
  m_gain(gain), m_dt(dt), m_epsilon(epsilon)
{
  m_S0 = 1.0;
  m_Si = m_Sj = m_Sk = 0.0;
  m_Ix = m_Iy = m_Iz = 0.0;
#if ENABLE_3RD_ORDER_INTERPOLATION
  m_omega0x = m_omega0y = m_omega0z = 0.0;
  m_omega1x = m_omega1y = m_omega1z = 0.0;
#endif
}

RotorIy::~RotorIy ()
{
}

// Additional fast finite float math functions.
extern "C" float Sinf(float x);
extern "C" float Cosf(float x);

static const float ep = 1.192093e-07;
static const float root_ep = 3.452670e-04;
static const float root_root_ep = 1.858136e-02;

static float
invSqrt (float x)
{
  union { int32_t i; float f; } u;
  float halfx = 0.5f * x;
  float y = x;
  u.f = y;
  u.i = 0x5f3759df - (u.i>>1);
  y = u.f;
  y = y * (1.5f - (halfx * y * y));
  return y;
}

inline static float
Sqrt (float x)
{
  return __builtin_sqrtf (x);
}

static float
Atan2 (float y, float x)
{
  float absx = x >= 0 ? x : -x;
  float absy = y >= 0 ? y : -y;
  float absmin = absx >= absy ? absy : absx;
  float absmax = absx >= absy ? absx : absy;
  float a = absmin/absmax;
  float s = a*a;
  // 7th order Taylor approximation
  float r = ((-0.0464964749*s + 0.15931422)*s - 0.327622764)*s*a + a;
  if (absy > absx)
    r = 1.57079637 - r;
  if (x < 0)
    r = 3.14159274 - r;
  if (y < 0)
    r = -r;
  return r;
}

static float
Sincf(float x)
{
  if (x < 0)
    x = -x;
  if (x >= root_root_ep)
    return Sinf(x)/x;

  float sc = 1.0;
  if (x >= ep)
    {
      float x2 = x*x;
      sc -= x2/6;
      if (x >= root_ep)
	sc += x2*x2/120;
    }

  return sc;
}
  
#if ENABLE_3RD_ORDER_INTERPOLATION
// inline version of commutator product. Only correct for Euclidean bivectors.
// return (1/2.0) (x*y - y*x);
inline static void
comm (const float a, const float b, const float c,
      const float d, const float e, const float f,
      float& x, float& y, float& z)
{
  x = -b*f+c*e;
  y = a*f-c*d;
  z = -a*e+b*d;
}
#endif

static const float sq_gravity_mss = (9.80665f*9.80665f);

void
RotorIy::UpdateIMU (float gx, float gy, float gz,
		    float ax, float ay, float az,
		    float& c, float& si, float& sj, float& sk)
{
  float omegax, omegay, omegaz;

  m_Ix = (1 - m_epsilon)*m_Ix + m_epsilon*gx;
  m_Iy = (1 - m_epsilon)*m_Iy + m_epsilon*gy;
  m_Iz = (1 - m_epsilon)*m_Iz + m_epsilon*gz;
  omegax = gx - m_Ix;
  omegay = gy - m_Iy;
  omegaz = gz - m_Iz;

  if (m_gain > 0)
    {
      // m_V = applyVersor (S, -e3);
      float vx = 2*m_S0*m_Sj - 2*m_Si*m_Sk;
      float vy = -2*m_S0*m_Si - 2*m_Sj*m_Sk;
      float vz = -m_S0*m_S0 + m_Si*m_Si + m_Sj*m_Sj - m_Sk*m_Sk;
      float nm2 = (ax+vx)*(ax+vx) + (ay+vy)*(ay+vy) + (az+vz)*(az+vz);
      float nm3 = ax*ax + ay*ay + az*az;
      // Don't fuse if y+v is too short or if |y| is far from |G|.
      //printf("%f %f %f\n", 0.8*sq_gravity_mss, nm3, 1.2*sq_gravity_mss);
      if (nm2 > m_norm_threshold
	  && 0.8*sq_gravity_mss < nm3 && nm3 < 1.2*sq_gravity_mss)
	{
	  // u = (1.0/nm)*(y+v);
	  float inm = invSqrt (nm2);
	  float ux = inm*(ax+vx);
	  float uy = inm*(ay+vy);
	  float uz = inm*(az+vz);
	  //printf("%f %f %f %f\n", inm, vx, vy, vz);
	  // P = u*v;
	  float p0, pi, pj, pk;
	  p0 = ux*vx + uy*vy + uz*vz;
	  pi = uy*vz - uz*vy;
	  pj = uz*vx - ux*vz;
	  pk = ux*vy - uy*vx;
	  // Y = -2.0*log (P);
	  // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
	  nm2 = pi*pi + pj*pj + pk*pk;
	  float nm = Sqrt (nm2);
	  if (nm > ep)
	    {
	      float ac = m_gain*(-2.0)*invSqrt (nm2)*Atan2 (p0, nm);
	      //printf("%f %f %f %f %f %f\n", omegax, omegay, omegaz, ac*pi, ac*pj, ac*pk);
	      omegax += ac*pi;
	      omegay += ac*pj;
	      omegaz += ac*pk;
	    }
	}
    }

#if ENABLE_3RD_ORDER_INTERPOLATION
  // 3rd order approximation by Candy and Lasenby.
  // _omega0 = \omega(-T), _omega1 = \omega(0), omega = \omega(T)
  float cx, cy, cz;
  comm (omegax - m_omega0x, omegay - m_omega0y, omegaz - m_omega0z,
	m_omega1x, m_omega1y, m_omega1z,
	cx, cy, cz);

  float dbx, dby, dbz;
  dbx = ((1.0/12)*(-m_omega0x + 8.0*m_omega1x + 5.0*omegax) + (1.0/24)*m_dt*cx);
  dby = ((1.0/12)*(-m_omega0y + 8.0*m_omega1y + 5.0*omegay) + (1.0/24)*m_dt*cy);
  dbz = ((1.0/12)*(-m_omega0z + 8.0*m_omega1z + 5.0*omegaz) + (1.0/24)*m_dt*cz);
  omegax = dbx;
  omegay = dby;
  omegaz = dbz;
  
  m_omega0x = m_omega1x;
  m_omega0y = m_omega1y;
  m_omega0z = m_omega1z;
  m_omega1x = omegax;
  m_omega1y = omegay;
  m_omega1z = omegaz;
#endif

  float delta = 0.5*m_dt*Sqrt(omegax*omegax + omegay*omegay + omegaz*omegaz);

  float dc = Cosf(delta);
  float dsc = -0.5*m_dt*Sincf(delta);
  float dsi = dsc*omegax;
  float dsj = dsc*omegay;
  float dsk = dsc*omegaz;
  //printf("dS %f\n", dc*dc + dsi*dsi + dsj*dsj + dsk*dsk);
  c = dc*m_S0 - dsi*m_Si - dsj*m_Sj - dsk*m_Sk;
  si = dsi*m_S0 + dc*m_Si - dsk*m_Sj + dsj*m_Sk;
  sj = dsj*m_S0 + dsk*m_Si + dc*m_Sj - dsi*m_Sk;
  sk = dsk*m_S0 - dsj*m_Si + dsi*m_Sj + dc*m_Sk;
  // Memowise the result
  m_S0 = c;
  m_Si = si;
  m_Sj = sj;
  m_Sk = sk;
  m_dS0 = dc;
  m_dSi = dsi;
  m_dSj = dsj;
  m_dSk = dsk;
}

// Fontijne-Dost algorism of 3D rotor reconstruction

inline static void
crossm (float x0, float x1, float x2, float y0, float y1, float y2,
	float& vi, float& vj, float& vk)
{
  vi = x1*y2 - x2*y1;
  vj = x2*y0 - x0*y2;
  vk = x0*y1 - x1*y0;
}

inline static void
ReconstructRotor0 (float x0, float x1, float x2,
		   float y0, float y1, float y2,
		   float xp0, float xp1, float xp2,
		   float yp0, float yp1, float yp2,
		   float& v0, float& vi, float& vj, float& vk)
{
  // (yp + y).(xp - x) + (yp - y)^(xp - x)
  v0 = (yp0 + y0)*(xp0 - x0) + (yp1 + y1)*(xp1 - x1) + (yp2 + y2)*(xp2 - x2);
  crossm (yp0 - y0, yp1 - y1, yp2 - y2, xp0 - x0, xp1 - x1, xp2 - x2,
	  vi, vj, vk);
}

static const float rotor_ep = 1.0e-6;

static void
ReconstructRotor (float x0, float x1, float x2,
		  float y0, float y1, float y2,
		  float xp0, float xp1, float xp2,
		  float yp0, float yp1, float yp2,
		  float& v0, float& vi, float& vj, float& vk)
{
  // V = (yp + y).(xp - x) + (yp - y)^(xp - x)
  ReconstructRotor0 (x0, x1, x2, y0, y1, y2,
		     xp0, xp1, xp2, yp0, yp1, yp2,
		     v0, vi, vj, vk);
  // if (|V| > epsilon) return V
  float nm = v0*v0 + vi*vi + vj*vj + vk*vk;
  if (nm > rotor_ep)
    return;

  // Singularity fix.
  // z = (x^y)*, zp = (xp^yp)*
  float z0, z1, z2, zp0, zp1, zp2;
  crossm (x0, x1, x2, y0, y1, y2, z0, z1, z2);
  crossm (xp0, xp1, xp2, yp0, yp1, yp2, zp0, zp1, zp2);
  // Vxz = (zp + z).(xp - x) + (zp - z)^(xp - x)
  ReconstructRotor0 (x0, x1, x2, z0, z1, z2,
		     xp0, xp1, xp2, zp0, zp1, zp2,
		     v0, vi, vj, vk);
  // Vzy = (yp + y).(zp - z) + (yp - y)^(zp - z)
  float u0, ui, uj, uk;
  ReconstructRotor0 (z0, z1, z2, y0, y1, y2,
		     zp0, zp1, zp2, yp0, yp1, yp2,
		     u0, ui, uj, uk);
  // Select larger one.
  float nxz = v0*v0 + vi*vi + vj*vj + vk*vk;
  float nzy = u0*u0 + ui*ui + uj*uj + uk*uk;
  if (nxz > nzy && nxz > rotor_ep)
    return;
  else if (nxz <= nzy && nzy > rotor_ep)
    {
      v0 = u0;
      vi = ui;
      vj = uj;
      vk = uk;
      return;
    }
  // If both norm smaller than epsilon, return 1
  v0 = 1;
  vi = vj = vk = 0;
}

void
RotorIy::UpdateIMU (float gx, float gy, float gz,
		    float ax, float ay, float az,
		    float mx, float my, float mz,
		    float& c, float& si, float& sj, float& sk)
{
  float omegax, omegay, omegaz;

  m_Ix = (1 - m_epsilon)*m_Ix + m_epsilon*gx;
  m_Iy = (1 - m_epsilon)*m_Iy + m_epsilon*gy;
  m_Iz = (1 - m_epsilon)*m_Iz + m_epsilon*gz;
  omegax = gx - m_Ix;
  omegay = gy - m_Iy;
  omegaz = gz - m_Iz;

  enum { NOFUSE, FUSE_ACC, FUSE_ACC_MAG, } fuse_type;
  if (m_gain == 0)
    fuse_type = NOFUSE;
  else
    {
      // (va,va)vm - (va,vm)va
      float nma = ax*ax + ay*ay + az*az;
      float cma = ax*mx + ay*my + az*mz;
      float tx = nma*mx - cma*ax;
      float ty = nma*my - cma*ay;
      float tz = nma*mz - cma*az;
      float nmt = tx*tx + ty*ty + tz*tz;
      //printf("%f %f\n", nmt, sq_gravity_mss * 100 * 100);
      if (nmt > sq_gravity_mss * 100 * 100
	  && nma > 0.8*sq_gravity_mss)
	{
	  float inmt = invSqrt (nmt);
	  // Holizontal north unit vector.
	  mx = inmt*tx;
	  my = inmt*ty;
	  mz = inmt*tz;
	  //printf("H %f %f %f\n", mx, my, mz);
	  float inma = invSqrt (nma);
	  ax = inma*ax;
	  ay = inma*ay;
	  az = inma*az;
	  fuse_type = FUSE_ACC_MAG;
	}
      else
	fuse_type = FUSE_ACC;
    }
  if (fuse_type == FUSE_ACC)
    {
      // v = applyVersor (S, -e3);
      float vx = 2*m_S0*m_Sj - 2*m_Si*m_Sk;
      float vy = -2*m_S0*m_Si - 2*m_Sj*m_Sk;
      float vz = -m_S0*m_S0 + m_Si*m_Si + m_Sj*m_Sj - m_Sk*m_Sk;
      float nma = ax*ax + ay*ay + az*az;
      float nm2 = (ax+vx)*(ax+vx) + (ay+vy)*(ay+vy) + (az+vz)*(az+vz);
      // Don't fuse if y+v is too short or if |y| is far from |G|.
      //printf("%f %f %f\n", 0.8*sq_gravity_mss, nma, 1.2*sq_gravity_mss);
      if (nm2 > m_norm_threshold
	  && 0.8*sq_gravity_mss < nma && nma < 1.2*sq_gravity_mss)
	{
	  // u = (1.0/nm)*(y+v);
	  float inm = invSqrt (nm2);
	  float ux = inm*(ax+vx);
	  float uy = inm*(ay+vy);
	  float uz = inm*(az+vz);
	  //printf("%f %f %f %f\n", inm, vx, vy, vz);
	  // P = u*v;
	  float p0, pi, pj, pk;
	  p0 = ux*vx + uy*vy + uz*vz;
	  pi = uy*vz - uz*vy;
	  pj = uz*vx - ux*vz;
	  pk = ux*vy - uy*vx;
	  // Y = -2.0*log (P);
	  // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
	  nm2 = pi*pi + pj*pj + pk*pk;
	  float nm = Sqrt (nm2);
	  if (nm > ep)
	    {
	      float ac = m_gain*(-2.0)*invSqrt (nm2)*Atan2 (p0, nm);
	      omegax += ac*pi;
	      omegay += ac*pj;
	      omegaz += ac*pk;
	    }
	}
    }
  else if (fuse_type == FUSE_ACC_MAG)
    {
      // v = applyVersor (S, -e3);
      float vx = 2*m_S0*m_Sj - 2*m_Si*m_Sk;
      float vy = -2*m_S0*m_Si - 2*m_Sj*m_Sk;
      float vz = -m_S0*m_S0 + m_Si*m_Si + m_Sj*m_Sj - m_Sk*m_Sk;
      // u = applyVersor (S, e1);
      float ux = m_S0*m_S0 + m_Si*m_Si - m_Sj*m_Sj - m_Sk*m_Sk;
      float uy = 2*m_Si*m_Sj - 2*m_S0*m_Sk;
      float uz = 2*m_S0*m_Sj + 2*m_Si*m_Sk;
      // Notice that y(ACC) and m(MAG) are normalized already.
      // Compute a rotor P which satisfies Pv = y and Pu = m
      float p0, pi, pj, pk;
      ReconstructRotor (vx, vy, vz, ux, uy, uz,
			ax, ay, az, mx, my, mz,
			p0, pi, pj, pk);
      //printf("P %f %f %f %f\n", p0, pi, pj, pk);
      // Y = -2.0*log (P);
      // log(P) = ([pi, pj, pk]/|[pi, pj, pk]|)*atan2(p0, |[pi, pj, pk]|)
      float nm2 = pi*pi + pj*pj + pk*pk;
      float nm = Sqrt (nm2);
      if (nm > ep)
	{
	  float ac = m_gain*(-2.0)*invSqrt (nm2)*Atan2 (p0, nm);
	  omegax += ac*pi;
	  omegay += ac*pj;
	  omegaz += ac*pk;
	  //printf("O %f %f %f\n", omegax, omegay, omegaz);
	}
    }

#if ENABLE_3RD_ORDER_INTERPOLATION
  // 3rd order approximation by Candy and Lasenby.
  // _omega0 = \omega(-T), _omega1 = \omega(0), omega = \omega(T)
  float cx, cy, cz;
  comm (omegax - m_omega0x, omegay - m_omega0y, omegaz - m_omega0z,
	m_omega1x, m_omega1y, m_omega1z,
	cx, cy, cz);

  float dbx, dby, dbz;
  dbx = ((1.0/12)*(-m_omega0x + 8.0*m_omega1x + 5.0*omegax) + (1.0/24)*m_dt*cx);
  dby = ((1.0/12)*(-m_omega0y + 8.0*m_omega1y + 5.0*omegay) + (1.0/24)*m_dt*cy);
  dbz = ((1.0/12)*(-m_omega0z + 8.0*m_omega1z + 5.0*omegaz) + (1.0/24)*m_dt*cz);
  omegax = dbx;
  omegay = dby;
  omegaz = dbz;
  
  m_omega0x = m_omega1x;
  m_omega0y = m_omega1y;
  m_omega0z = m_omega1z;
  m_omega1x = omegax;
  m_omega1y = omegay;
  m_omega1z = omegaz;
#endif

  float delta = 0.5*m_dt*Sqrt(omegax*omegax + omegay*omegay + omegaz*omegaz);

  float dc = Cosf(delta);
  float dsc = -0.5*m_dt*Sincf(delta);
  float dsi = dsc*omegax;
  float dsj = dsc*omegay;
  float dsk = dsc*omegaz;
  c = dc*m_S0 - dsi*m_Si - dsj*m_Sj - dsk*m_Sk;
  si = dsi*m_S0 + dc*m_Si - dsk*m_Sj + dsj*m_Sk;
  sj = dsj*m_S0 + dsk*m_Si + dc*m_Sj - dsi*m_Sk;
  sk = dsk*m_S0 - dsj*m_Si + dsi*m_Sj + dc*m_Sk;
  //printf("%f %f %f %f\n", dsk*m_S0, -dsj*m_Si, dsi*m_Sj, dc*m_Sk);
  // Memowise the result
  m_S0 = c;
  m_Si = si;
  m_Sj = sj;
  m_Sk = sk;
  m_dS0 = dc;
  m_dSi = dsi;
  m_dSj = dsj;
  m_dSk = dsk;
}

void
RotorIy::SetGain (float gain)
{
  if (gain >= 0 && gain < 10)
    m_gain = gain;
}

void
RotorIy::Show (void)
{
  printf ("S=%2.6f %2.6f %2.6f %2.6f\n", m_S0, m_Si, m_Sj, m_Sk);
  printf ("dS=%2.6f %2.6f %2.6f %2.6f\n", m_dS0, m_dSi, m_dSj, m_dSk);
}
