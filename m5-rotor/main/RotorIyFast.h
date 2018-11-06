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

class RotorIy
{
 public:
  RotorIy ();
  RotorIy (float gain, float dt, float epsilon);
  virtual ~RotorIy ();
  // Update with IMU value and return the current estimation.
  void UpdateIMU (float ax, float ay, float az, float gx, float gy, float gz,
		  float& c, float& si, float& sj, float& sk);
  void UpdateIMU (float ax, float ay, float az, float gx, float gy, float gz,
		  float mx, float my, float mz,
		  float& c, float& si, float& sj, float& sk);
  // Set gain.
  void SetGain (float gain);
  void Show ();
 private:
  // Estimated rotor.
  float m_S0, m_Si, m_Sj, m_Sk;
  float m_dS0, m_dSi, m_dSj, m_dSk;
  // Memowised omegas.
  float m_omega0x, m_omega0y, m_omega0z;
  float m_omega1x, m_omega1y, m_omega1z;
  // Integrated gyro bivector which shows its drift.
  float m_Ix, m_Iy, m_Iz;
  // Fuse gain.
  float m_gain;
  // Constants
  const float m_dt = 1.0e-3;
  const float m_epsilon = 1.0e-6;
  // Is 1.0(~0.1*GRAVITY_MSS) ok?
  const float m_norm_threshold = 1.0;
};
