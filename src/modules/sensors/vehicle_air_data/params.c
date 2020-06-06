/****************************************************************************
 *
 *   Copyright (c) 2012-2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * QNH for barometer
 *
 * @min 500
 * @max 1500
 * @group Sensors
 * @unit hPa
 *
 * @reboot_required true
 *
 */
PARAM_DEFINE_FLOAT(SENS_BARO_QNH, 1013.25f);

/**
 * Airspeed compensation for barometer altitude.
 *
 * Can be used to compensate for changes in barometric
 * altitude caused by changes in airspeed.
 *
 * How to set this value:
 * BARO_AS_COMP = -(change in pressure when airspeed increases from 0 to V)/V^2
 *
 * @min -1
 * @max 1
 * @group Sensors
 * @unit 1/m/s^2
 */
PARAM_DEFINE_FLOAT(BARO_AS_COMP, 0.0f);

/**
  * QNH change rate for barometer.
  *
  * Too high a value will lead to EKF errors and possibly crash the vehicle.
  * Test by arming the plane, changing the QNH by
  * eg. 10 hPa and logging the global position altitude for 10 minutes.
  * The altitude should slowly change while being valid for all the time.
  * In case of any altitude anomalities, halve the value. A good
  * starting point is 0.1, which means about 0.8m/min rate of change to the
  * pressure altimeter. Higher values are faster but more prone to EKF errors.
  *
  *
  * @min 0
  * @max 0.5
  * @group Sensors
  * @unit hPa/min
  */
 PARAM_DEFINE_FLOAT(SENS_QNH_RATE, 0.0f);
