/****************************************************************************
 *
 *   Copyright (c) 2012-2017 PX4 Development Team. All rights reserved.
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

/*
 * @file landingslope.cpp
 *
 * @author Thomas Gubler <thomasgubler@gmail.com>
 */

#include "Landingslope.hpp"

#include <mathlib/mathlib.h>

void
Landingslope::update(float landing_slope_angle_rad_new,
		     float flare_relative_alt_new,
		     float motor_lim_relative_alt_new,
		     float H1_virt_new)
{
	_landing_slope_angle_rad = landing_slope_angle_rad_new;
	_flare_relative_alt = flare_relative_alt_new;
	_motor_lim_relative_alt = motor_lim_relative_alt_new;
	_H1_virt = H1_virt_new;

	calculateSlopeValues();
}

void
Landingslope::calculateSlopeValues()
{
	_H0 =  _flare_relative_alt + _H1_virt;
	_d1 = _flare_relative_alt / tanf(_landing_slope_angle_rad);
	_flare_constant = (_H0 * _d1) / _flare_relative_alt;
	_flare_length = -logf(_H1_virt / _H0) * _flare_constant;
	_horizontal_slope_displacement = (_flare_length - _d1);

	//...But since we are not using a predefined flare path (temporary fix)
	_horizontal_slope_displacement = 0.0f;
	_flare_length = _flare_relative_alt / tanf(_landing_slope_angle_rad);
}

float
Landingslope::getLandingSlopeRelativeAltitude(float wp_landing_distance)
{
	return Landingslope::getLandingSlopeRelativeAltitude(wp_landing_distance, _horizontal_slope_displacement,
			_landing_slope_angle_rad);
}

float
Landingslope::getFlareCurveRelativeAltitude(float wp_landing_distance)
{
	/* If airplane is in front of waypoint return flare curve altitude, else return waypoint altitude */
	if (wp_landing_distance > 0) {
		return _H0 * expf(-math::max(0.0f, _flare_length - wp_landing_distance) / _flare_constant) - _H1_virt;

	}

	return 0.0f;
}

/**
 *
 * @return Relative altitude of point on landing slope at distance to landing waypoint=wp_landing_distance
 */
float Landingslope::getLandingSlopeRelativeAltitude(float wp_landing_distance, float horizontal_slope_displacement,
		float landing_slope_angle_rad)
{
	// flare_relative_alt is negative
	return (wp_landing_distance - horizontal_slope_displacement) * tanf(landing_slope_angle_rad);
}

/**
 *
 * @return Absolute altitude of point on landing slope at distance to landing waypoint=wp_landing_distance
 */
float Landingslope::getLandingSlopeAbsoluteAltitude(float wp_landing_distance, float wp_landing_altitude,
		float horizontal_slope_displacement, float landing_slope_angle_rad)
{
	return getLandingSlopeRelativeAltitude(wp_landing_distance, horizontal_slope_displacement,
					       landing_slope_angle_rad) + wp_landing_altitude;
}

/**
 *
 * @return distance to landing waypoint of point on landing slope at altitude=slope_altitude
 */
float Landingslope::getLandingSlopeWPDistance(float slope_altitude, float wp_landing_altitude,
		float horizontal_slope_displacement, float landing_slope_angle_rad)
{
	return (slope_altitude - wp_landing_altitude) / tanf(landing_slope_angle_rad) + horizontal_slope_displacement;
}

/**
 *
 * @return the remaining flare curve length at a given altitude.
 */
float
Landingslope::getFlareCurveLengthAtAltiude(float alt)
{
	/* If airplane is in front of waypoint return distance of remaining flare, else return 0 */
	if (alt > 0 && _H1_virt > 0) {
		return _flare_length - _flare_constant * logf(_H0 / (_H1_virt + alt));

	}

	return 0.0f;

}
