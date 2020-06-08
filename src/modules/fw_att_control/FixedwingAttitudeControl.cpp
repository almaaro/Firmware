/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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

#include "FixedwingAttitudeControl.hpp"

#include <vtol_att_control/vtol_type.h>

using namespace time_literals;
using math::constrain;
using math::gradual;
using math::radians;

FixedwingAttitudeControl::FixedwingAttitudeControl(bool vtol) :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::attitude_ctrl),
	_actuators_0_pub(vtol ? ORB_ID(actuator_controls_virtual_fw) : ORB_ID(actuator_controls_0)),
	_attitude_sp_pub(vtol ? ORB_ID(fw_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle"))
{
	// check if VTOL first
	if (vtol) {
		int32_t vt_type = -1;

		if (param_get(param_find("VT_TYPE"), &vt_type) == PX4_OK) {
			_is_tailsitter = (static_cast<vtol_type>(vt_type) == vtol_type::TAILSITTER);
		}
	}

	/* fetch initial parameter values */
	parameters_update();

	// set initial maximum body rate setpoints
	_roll_ctrl.set_max_rate(radians(_param_fw_acro_x_max.get()));
	_pitch_ctrl.set_max_rate_pos(radians(_param_fw_acro_y_max.get()));
	_pitch_ctrl.set_max_rate_neg(radians(_param_fw_acro_y_max.get()));
	_yaw_ctrl.set_max_rate(radians(_param_fw_acro_z_max.get()));
}

FixedwingAttitudeControl::~FixedwingAttitudeControl()
{
	perf_free(_loop_perf);
}

bool
FixedwingAttitudeControl::init()
{
	if (!_att_sub.registerCallback()) {
		PX4_ERR("vehicle attitude callback registration failed!");
		return false;
	}

	return true;
}

int
FixedwingAttitudeControl::parameters_update()
{
	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_param_fw_p_tc.get());
	_pitch_ctrl.set_k_p(_param_fw_pr_p.get());
	_pitch_ctrl.set_k_i(_param_fw_pr_i.get());
	_pitch_ctrl.set_k_ff(_param_fw_pr_ff.get());
	_pitch_ctrl.set_integrator_max(_param_fw_pr_imax.get());

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_param_fw_r_tc.get());
	_roll_ctrl.set_k_p(_param_fw_rr_p.get());
	_roll_ctrl.set_k_i(_param_fw_rr_i.get());
	_roll_ctrl.set_k_ff(_param_fw_rr_ff.get());
	_roll_ctrl.set_integrator_max(_param_fw_rr_imax.get());

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_param_fw_yr_p.get());
	_yaw_ctrl.set_k_i(_param_fw_yr_i.get());
	_yaw_ctrl.set_k_ff(_param_fw_yr_ff.get());
	_yaw_ctrl.set_integrator_max(_param_fw_yr_imax.get());

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_param_fw_wr_p.get());
	_wheel_ctrl.set_k_i(_param_fw_wr_i.get());
	_wheel_ctrl.set_k_ff(_param_fw_wr_ff.get());
	_wheel_ctrl.set_integrator_max(_param_fw_wr_imax.get());
	_wheel_ctrl.set_max_rate(radians(_param_fw_w_rmax.get()));

	return PX4_OK;
}

void
FixedwingAttitudeControl::vehicle_control_mode_poll()
{
	_vcontrol_mode_sub.update(&_vcontrol_mode);

	if (_vehicle_status.is_vtol) {
		const bool is_hovering = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
					 && !_vehicle_status.in_transition_mode;
		const bool is_tailsitter_transition = _vehicle_status.in_transition_mode && _is_tailsitter;

		if (is_hovering || is_tailsitter_transition) {
			_vcontrol_mode.flag_control_attitude_enabled = false;
			_vcontrol_mode.flag_control_manual_enabled = false;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_manual_poll()
{
	const bool is_tailsitter_transition = _is_tailsitter && _vehicle_status.in_transition_mode;
	const bool is_fixed_wing = _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING;

	if (_vcontrol_mode.flag_control_manual_enabled && (!is_tailsitter_transition || is_fixed_wing)) {

		// Always copy the new manual setpoint, even if it wasn't updated, to fill the _actuators with valid values
		if (_manual_sub.copy(&_manual)) {

			// Check if we are in rattitude mode and the pilot is above the threshold on pitch
			if (_vcontrol_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual.y) > _param_fw_ratt_th.get() || fabsf(_manual.x) > _param_fw_ratt_th.get()) {
					_vcontrol_mode.flag_control_attitude_enabled = false;
				}
			}

			if (!_vcontrol_mode.flag_control_climb_rate_enabled &&
			    !_vcontrol_mode.flag_control_offboard_enabled) {

				if (_vcontrol_mode.flag_control_attitude_enabled) {
					// STABILIZED mode generate the attitude setpoint from manual user inputs

					_att_sp.roll_body = _manual.y * radians(_param_fw_man_r_max.get()) + radians(_param_fw_rsp_off.get());
					_att_sp.roll_body = constrain(_att_sp.roll_body,
								      -radians(_param_fw_man_r_max.get()), radians(_param_fw_man_r_max.get()));

					_att_sp.pitch_body = -_manual.x * radians(_param_fw_man_p_max.get());
					_att_sp.pitch_body = constrain(_att_sp.pitch_body,
								       -radians(_param_fw_man_p_max.get()), radians(_param_fw_man_p_max.get()));

					_att_sp.yaw_body = 0.0f;
					_att_sp.thrust_body[0] = _manual.z;

					Quatf q(Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body));
					q.copyTo(_att_sp.q_d);

					_att_sp.timestamp = hrt_absolute_time();

					_attitude_sp_pub.publish(_att_sp);

				} else if (_vcontrol_mode.flag_control_rates_enabled &&
					   !_vcontrol_mode.flag_control_attitude_enabled) {

					// RATE mode we need to generate the rate setpoint from manual user inputs
					_rates_sp.timestamp = hrt_absolute_time();
					_rates_sp.roll = _manual.y * radians(_param_fw_acro_x_max.get());
					_rates_sp.pitch = -_manual.x * radians(_param_fw_acro_y_max.get());
					_rates_sp.yaw = _manual.r * radians(_param_fw_acro_z_max.get());
					_rates_sp.thrust_body[0] = _manual.z;

					_rate_sp_pub.publish(_rates_sp);

				} else {
					/* manual/direct control */
					_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y * _param_fw_man_r_sc.get() + _param_trim_roll.get();
					_actuators.control[actuator_controls_s::INDEX_PITCH] = -_manual.x * _param_fw_man_p_sc.get() + _param_trim_pitch.get();
					_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r * _param_fw_man_y_sc.get() + _param_trim_yaw.get();
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
				}
			}
		}
	}
}

void
FixedwingAttitudeControl::vehicle_attitude_setpoint_poll()
{
	if (_att_sp_sub.update(&_att_sp)) {
		_rates_sp.thrust_body[0] = _att_sp.thrust_body[0];
		_rates_sp.thrust_body[1] = _att_sp.thrust_body[1];
		_rates_sp.thrust_body[2] = _att_sp.thrust_body[2];
	}
}

void
FixedwingAttitudeControl::vehicle_rates_setpoint_poll()
{
	if (_rates_sp_sub.update(&_rates_sp)) {
		if (_is_tailsitter) {
			float tmp = _rates_sp.roll;
			_rates_sp.roll = -_rates_sp.yaw;
			_rates_sp.yaw = tmp;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_land_detected_poll()
{
	if (_vehicle_land_detected_sub.updated()) {
		vehicle_land_detected_s vehicle_land_detected {};

		if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
			_landed = vehicle_land_detected.landed;
		}
	}
}

void
FixedwingAttitudeControl::vehicle_motor_airstream_poll()
{
	if (_vehicle_motor_airstream_sub.update(&_vehicle_motor_airstream)) {
		if (hrt_elapsed_time(&_vehicle_motor_airstream.timestamp) < 1_s
		    && _vehicle_motor_airstream.required_as_elev > 0.0f) {

			_motor_airstream_valid = true;

		} else {
			_motor_airstream_valid = false;
		}

	}
}

float FixedwingAttitudeControl::get_airspeed_and_update_scaling()
{
	_airspeed_validated_sub.update();
	const bool airspeed_valid = PX4_ISFINITE(_airspeed_validated_sub.get().equivalent_airspeed_m_s)
				    && (hrt_elapsed_time(&_airspeed_validated_sub.get().timestamp) < 1_s);

	// if no airspeed measurement is available out best guess is to use the trim airspeed
	float airspeed = _param_fw_airspd_trim.get();

	if ((_param_fw_arsp_mode.get() == 0) && airspeed_valid) {
		/* prevent numerical drama by requiring 0.5 m/s minimal speed */
		airspeed = math::max(0.5f, _airspeed_validated_sub.get().equivalent_airspeed_m_s);

	} else {
		// VTOL: if we have no airspeed available and we are in hover mode then assume the lowest airspeed possible
		// this assumption is good as long as the vehicle is not hovering in a headwind which is much larger
		// than the minimum airspeed
		if (_vehicle_status.is_vtol && _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
		    && !_vehicle_status.in_transition_mode) {
			airspeed = _param_fw_airspd_min.get();
		}
	}

	// Adjust the minimum airspeed to the flap setting
	_airspeed_min_adj = _flaps_applied * _param_fw_airspd_min_flps.get() + (1.0f - _flaps_applied) *
			    _param_fw_airspd_min.get();

	/*
	 * For scaling our actuators using anything less than the min (close to stall)
	 * speed doesn't make any sense - its the strongest reasonable deflection we
	 * want to do in flight and its the baseline a human pilot would choose.
	 *
	 * Forcing the scaling to this value allows reasonable handheld tests.
	 */
	float airspeed_constrained = constrain(airspeed, _airspeed_min_adj, _param_fw_airspd_max.get());

	_airspeed_scaling = (_param_fw_arsp_scale_en.get()) ? (_param_fw_airspd_trim.get() / airspeed_constrained) : 1.0f;

	//The elevator needs a different scaler as the motor airstream may hit it.
	airspeed_constrained = math::max(_vehicle_motor_airstream.required_as_elev, _param_fw_airspd_min.get());

	if (_motor_airstream_valid) {
		_airspeed_scaling_elevator = sqrtf(_vehicle_motor_airstream.as_elev_trim_as_level_sq) / airspeed_constrained;

	} else {
		_airspeed_scaling_elevator = _airspeed_scaling;
	}



	return airspeed;
}

void FixedwingAttitudeControl::Run()
{
	if (should_exit()) {
		_att_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	if (_att_sub.update(&_att)) {

		// only update parameters if they changed
		bool params_updated = _parameter_update_sub.updated();

		// check for parameter updates
		if (params_updated) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
			parameters_update();
		}

		/* only run controller if attitude changed */
		static uint64_t last_run = 0;
		float deltaT = constrain((hrt_elapsed_time(&last_run) / 1e6f), 0.01f, 0.1f);
		last_run = hrt_absolute_time();

		/* get current rotation matrix and euler angles from control state quaternions */
		matrix::Dcmf R = matrix::Quatf(_att.q);

		vehicle_angular_velocity_s angular_velocity{};
		_vehicle_rates_sub.copy(&angular_velocity);
		float rollspeed = angular_velocity.xyz[0];
		float pitchspeed = angular_velocity.xyz[1];
		float yawspeed = angular_velocity.xyz[2];

		if (_is_tailsitter) {
			/* vehicle is a tailsitter, we need to modify the estimated attitude for fw mode
			 *
			 * Since the VTOL airframe is initialized as a multicopter we need to
			 * modify the estimated attitude for the fixed wing operation.
			 * Since the neutral position of the vehicle in fixed wing mode is -90 degrees rotated around
			 * the pitch axis compared to the neutral position of the vehicle in multicopter mode
			 * we need to swap the roll and the yaw axis (1st and 3rd column) in the rotation matrix.
			 * Additionally, in order to get the correct sign of the pitch, we need to multiply
			 * the new x axis of the rotation matrix with -1
			 *
			 * original:			modified:
			 *
			 * Rxx  Ryx  Rzx		-Rzx  Ryx  Rxx
			 * Rxy	Ryy  Rzy		-Rzy  Ryy  Rxy
			 * Rxz	Ryz  Rzz		-Rzz  Ryz  Rxz
			 * */
			matrix::Dcmf R_adapted = R;		//modified rotation matrix

			/* move z to x */
			R_adapted(0, 0) = R(0, 2);
			R_adapted(1, 0) = R(1, 2);
			R_adapted(2, 0) = R(2, 2);

			/* move x to z */
			R_adapted(0, 2) = R(0, 0);
			R_adapted(1, 2) = R(1, 0);
			R_adapted(2, 2) = R(2, 0);

			/* change direction of pitch (convert to right handed system) */
			R_adapted(0, 0) = -R_adapted(0, 0);
			R_adapted(1, 0) = -R_adapted(1, 0);
			R_adapted(2, 0) = -R_adapted(2, 0);

			/* fill in new attitude data */
			R = R_adapted;

			/* lastly, roll- and yawspeed have to be swaped */
			float helper = rollspeed;
			rollspeed = -yawspeed;
			yawspeed = helper;
		}

		const matrix::Eulerf euler_angles(R);

		vehicle_attitude_setpoint_poll();

		// vehicle status update must be before the vehicle_control_mode_poll(), otherwise rate sp are not published during whole transition
		_vehicle_status_sub.update(&_vehicle_status);

		vehicle_control_mode_poll();
		vehicle_manual_poll();
		vehicle_land_detected_poll();

		// the position controller will not emit attitude setpoints in some modes
		// we need to make sure that this flag is reset
		_att_sp.fw_control_yaw = _att_sp.fw_control_yaw && _vcontrol_mode.flag_control_auto_enabled;

		bool wheel_control = false;

		// TODO: manual wheel_control on ground?
		if (_param_fw_w_en.get() && _att_sp.fw_control_yaw) {
			wheel_control = true;
		}

		/* lock integrator until control is started */
		bool lock_integrator = !_vcontrol_mode.flag_control_rates_enabled
				       || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && ! _vehicle_status.in_transition_mode);

		/* if we are in rotary wing mode, do nothing */
		if (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING && !_vehicle_status.is_vtol) {
			perf_end(_loop_perf);
			return;
		}

		control_flaps(deltaT);

		/* decide if in stabilized or full manual control */
		if (_vcontrol_mode.flag_control_rates_enabled) {

			const float airspeed = get_airspeed_and_update_scaling();

			/* reset integrals where needed */
			if (_att_sp.roll_reset_integral) {
				_roll_ctrl.reset_integrator();
			}

			if (_att_sp.pitch_reset_integral) {
				_pitch_ctrl.reset_integrator();
			}

			if (_att_sp.yaw_reset_integral) {
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}

			/* Reset integrators if the aircraft is on ground
			 * or a multicopter (but not transitioning VTOL)
			 */
			if (_landed
			    || (_vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING
				&& !_vehicle_status.in_transition_mode)) {

				_roll_ctrl.reset_integrator();
				_pitch_ctrl.reset_integrator();
				_yaw_ctrl.reset_integrator();
				_wheel_ctrl.reset_integrator();
			}

			/* Prepare data for attitude controllers */
			struct ECL_ControlData control_input = {};
			control_input.roll = euler_angles.phi();
			control_input.pitch = euler_angles.theta();
			control_input.yaw = euler_angles.psi();
			control_input.body_x_rate = rollspeed;
			control_input.body_y_rate = pitchspeed;
			control_input.body_z_rate = yawspeed;
			control_input.roll_setpoint = _att_sp.roll_body;
			control_input.pitch_setpoint = _att_sp.pitch_body;
			control_input.yaw_setpoint = _att_sp.yaw_body;
			control_input.airspeed_min = _airspeed_min_adj;
			control_input.airspeed_max = _param_fw_airspd_max.get();
			control_input.airspeed = airspeed;
			control_input.scaler = _airspeed_scaling;
			control_input.lock_integrator = lock_integrator;

			if (wheel_control) {
				_local_pos_sub.update(&_local_pos);

				/* Use min airspeed to calculate ground speed scaling region.
				* Don't scale below gspd_scaling_trim
				*/
				float groundspeed = sqrtf(_local_pos.vx * _local_pos.vx + _local_pos.vy * _local_pos.vy);
				float gspd_scaling_trim = (_param_fw_airspd_min.get() * 0.6f);
				float groundspeed_scaler = gspd_scaling_trim / ((groundspeed < gspd_scaling_trim) ? gspd_scaling_trim : groundspeed);

				control_input.groundspeed = groundspeed;
				control_input.groundspeed_scaler = groundspeed_scaler;
			}

			/* reset body angular rate limits on mode change */
			if ((_vcontrol_mode.flag_control_attitude_enabled != _flag_control_attitude_enabled_last) || params_updated) {
				if (_vcontrol_mode.flag_control_attitude_enabled
				    || _vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING) {
					_roll_ctrl.set_max_rate(radians(_param_fw_r_rmax.get()));
					_pitch_ctrl.set_max_rate_pos(radians(_param_fw_p_rmax_pos.get()));
					_pitch_ctrl.set_max_rate_neg(radians(_param_fw_p_rmax_neg.get()));
					_yaw_ctrl.set_max_rate(radians(_param_fw_y_rmax.get()));

				} else {
					_roll_ctrl.set_max_rate(radians(_param_fw_acro_x_max.get()));
					_pitch_ctrl.set_max_rate_pos(radians(_param_fw_acro_y_max.get()));
					_pitch_ctrl.set_max_rate_neg(radians(_param_fw_acro_y_max.get()));
					_yaw_ctrl.set_max_rate(radians(_param_fw_acro_z_max.get()));
				}
			}

			_flag_control_attitude_enabled_last = _vcontrol_mode.flag_control_attitude_enabled;

			/* bi-linear interpolation over airspeed for actuator trim scheduling */
			float trim_roll = _param_trim_roll.get();
			float trim_pitch = _param_trim_pitch.get();
			float trim_pitch_flaps = trim_pitch;
			float trim_yaw = _param_trim_yaw.get();

			float as_land = _param_fw_airspd_min_flps.get() * _param_fw_lnd_airspd_sc.get();
			float as_diff_max_trim = math::max(_param_fw_airspd_max.get()-_param_fw_airspd_trim.get(), 1.0f);
			float as_diff_trim_min = math::max(_param_fw_airspd_trim.get()-_param_fw_airspd_min.get(), 1.0f);
			float as_diff_min_land = math::max(_param_fw_airspd_min.get()-as_land, 0.5f);
			float thr_ratio_lo = constrain((_actuators.control[actuator_controls_s::INDEX_THROTTLE]-_param_fw_thr_min.get())/(_param_fw_thr_cruise.get()-_param_fw_thr_min.get()), 0.0f, 1.0f);
			float thr_ratio_hi = constrain((_actuators.control[actuator_controls_s::INDEX_THROTTLE]-_param_fw_thr_cruise.get())/(_param_fw_thr_max.get()-_param_fw_thr_cruise.get()), 0.0f, 1.0f);
			float as_ratio_lo = constrain((airspeed -_param_fw_airspd_min.get())/as_diff_trim_min, 0.0f, 1.0f);
			float as_ratio_hi = constrain((airspeed -_param_fw_airspd_trim.get())/as_diff_max_trim, 0.0f, 1.0f);
			float as_ratio_land = constrain((airspeed-as_land)/as_diff_min_land, 0.0f, 1.0f);

			//Clean flight config trims
			if(_actuators.control[actuator_controls_s::INDEX_THROTTLE] > _param_fw_thr_cruise.get()){
				if (airspeed > _param_fw_airspd_trim.get()){
					trim_pitch = find_trim_from_4_coordinates(thr_ratio_hi, as_ratio_hi, _param_trm_p_vc_tc.get(), _param_trm_p_vh_tc.get(), _param_trm_p_vc_th.get(), _param_trm_p_vh_th.get());
				}else {
					trim_pitch = find_trim_from_4_coordinates(thr_ratio_hi, as_ratio_lo, _param_trm_p_vm_tc.get(), _param_trm_p_vc_tc.get(), _param_trm_p_vm_th.get(), _param_trm_p_vc_th.get());
				}
			} else{
				if (airspeed > _param_fw_airspd_trim.get()){
					trim_pitch = find_trim_from_4_coordinates(thr_ratio_lo, as_ratio_hi, _param_trm_p_vc_tm.get(), _param_trm_p_vh_tm.get(), _param_trm_p_vc_tc.get(), _param_trm_p_vh_tc.get());
				}else {
					trim_pitch = find_trim_from_4_coordinates(thr_ratio_lo, as_ratio_lo, _param_trm_p_vm_tm.get(), _param_trm_p_vc_tm.get(), _param_trm_p_vm_tc.get(), _param_trm_p_vc_tc.get());
				}
			}

			//Flaps trims. If as_land is very close to as_min, don't use it.
			if(_actuators.control[actuator_controls_s::INDEX_THROTTLE] > _param_fw_thr_cruise.get()){
				if (airspeed >_param_fw_airspd_min.get() || as_diff_min_land < 1.0f){
					trim_pitch_flaps = find_trim_from_4_coordinates(thr_ratio_hi, as_ratio_lo, _param_trm_pf_vc_tc.get(), _param_trm_pf_vh_tc.get(), _param_trm_pf_vc_th.get(), _param_trm_pf_vh_th.get());
				}else {
					trim_pitch_flaps = find_trim_from_4_coordinates(thr_ratio_hi, as_ratio_land, _param_trm_pf_vm_tc.get(), _param_trm_pf_vc_tc.get(), _param_trm_pf_vm_th.get(), _param_trm_pf_vc_th.get());
				}
			} else{
				if (airspeed > _param_fw_airspd_min.get() || as_diff_min_land < 1.0f){
					trim_pitch_flaps = find_trim_from_4_coordinates(thr_ratio_lo, as_ratio_lo, _param_trm_pf_vc_tm.get(), _param_trm_pf_vh_tm.get(), _param_trm_pf_vc_tc.get(), _param_trm_pf_vh_tc.get());
				}else {
					trim_pitch_flaps = find_trim_from_4_coordinates(thr_ratio_lo, as_ratio_land, _param_trm_pf_vm_tm.get(), _param_trm_pf_vc_tm.get(), _param_trm_pf_vm_tc.get(), _param_trm_pf_vc_tc.get());
				}
			}

			if (airspeed < _param_fw_airspd_trim.get()) {
				trim_roll += math::gradual(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
							   _param_fw_dtrim_r_vmin.get(),
							   0.0f);

				trim_yaw += math::gradual(airspeed, _param_fw_airspd_min.get(), _param_fw_airspd_trim.get(),
							  _param_fw_dtrim_y_vmin.get(),
							  0.0f);

			} else {
				trim_roll += math::gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
							   _param_fw_dtrim_r_vmax.get());
				trim_yaw += math::gradual(airspeed, _param_fw_airspd_trim.get(), _param_fw_airspd_max.get(), 0.0f,
							  _param_fw_dtrim_y_vmax.get());
			}

			/* add trim increment if flaps are deployed  */
			trim_roll += _flaps_applied * _param_fw_dtrim_r_flps.get();
			trim_pitch = (1.0f -_flaps_applied) * trim_pitch + _flaps_applied * trim_pitch_flaps;

			/* Run attitude controllers */
			if (_vcontrol_mode.flag_control_attitude_enabled) {
				if (PX4_ISFINITE(_att_sp.roll_body) && PX4_ISFINITE(_att_sp.pitch_body)) {
					_roll_ctrl.control_attitude(control_input);
					_pitch_ctrl.control_attitude(control_input);

					if (wheel_control) {
						_wheel_ctrl.control_attitude(control_input);
						_yaw_ctrl.reset_integrator();

					} else {
						// runs last, because is depending on output of roll and pitch attitude
						_yaw_ctrl.control_attitude(control_input);
						_wheel_ctrl.reset_integrator();
					}

					/* Update input data for rate controllers */
					control_input.roll_rate_setpoint = _roll_ctrl.get_desired_rate();
					control_input.pitch_rate_setpoint = _pitch_ctrl.get_desired_rate();
					control_input.yaw_rate_setpoint = _yaw_ctrl.get_desired_rate();

					/* Run attitude RATE controllers which need the desired attitudes from above, add trim */
					float roll_u = _roll_ctrl.control_euler_rate(control_input);
					_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

					if (!PX4_ISFINITE(roll_u)) {
						_roll_ctrl.reset_integrator();
					}

					// The elevator has a different scaler
					control_input.scaler = _airspeed_scaling_elevator;
					float pitch_u = _pitch_ctrl.control_euler_rate(control_input);
					_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;
					control_input.scaler = _airspeed_scaling;

					if (!PX4_ISFINITE(pitch_u)) {
						_pitch_ctrl.reset_integrator();
					}

					float yaw_u = 0.0f;

					if (wheel_control) {
						yaw_u = _wheel_ctrl.control_bodyrate(control_input);

					} else {
						yaw_u = _yaw_ctrl.control_euler_rate(control_input);
					}

					_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

					/* add in manual rudder control in manual modes */
					if (_vcontrol_mode.flag_control_manual_enabled) {
						_actuators.control[actuator_controls_s::INDEX_YAW] += _manual.r;
					}

					if (!PX4_ISFINITE(yaw_u)) {
						_yaw_ctrl.reset_integrator();
						_wheel_ctrl.reset_integrator();
					}

					/* throttle passed through if it is finite and if no engine failure was detected */
					_actuators.control[actuator_controls_s::INDEX_THROTTLE] = (PX4_ISFINITE(_att_sp.thrust_body[0])
							&& !_vehicle_status.engine_failure) ? _att_sp.thrust_body[0] : 0.0f;

					/* scale effort by battery status */
					if (_param_fw_bat_scale_en.get() &&
					    _actuators.control[actuator_controls_s::INDEX_THROTTLE] > 0.1f) {

						if (_battery_status_sub.updated()) {
							battery_status_s battery_status{};

							if (_battery_status_sub.copy(&battery_status)) {
								if (battery_status.scale > 0.0f) {
									_battery_scale = battery_status.scale;
								}
							}
						}

						_actuators.control[actuator_controls_s::INDEX_THROTTLE] *= _battery_scale;
					}

				}

				/*
				 * Lazily publish the rate setpoint (for analysis, the actuators are published below)
				 * only once available
				 */
				_rates_sp.roll = _roll_ctrl.get_desired_bodyrate();
				_rates_sp.pitch = _pitch_ctrl.get_desired_bodyrate();
				_rates_sp.yaw = _yaw_ctrl.get_desired_bodyrate();

				_rates_sp.timestamp = hrt_absolute_time();

			} else {
				vehicle_rates_setpoint_poll();

				_roll_ctrl.set_bodyrate_setpoint(_rates_sp.roll);
				_yaw_ctrl.set_bodyrate_setpoint(_rates_sp.yaw);
				_pitch_ctrl.set_bodyrate_setpoint(_rates_sp.pitch);

				float roll_u = _roll_ctrl.control_bodyrate(control_input);
				_actuators.control[actuator_controls_s::INDEX_ROLL] = (PX4_ISFINITE(roll_u)) ? roll_u + trim_roll : trim_roll;

				float yaw_u = _yaw_ctrl.control_bodyrate(control_input);
				_actuators.control[actuator_controls_s::INDEX_YAW] = (PX4_ISFINITE(yaw_u)) ? yaw_u + trim_yaw : trim_yaw;

				// The elevator has a different scaler
				control_input.scaler = _airspeed_scaling_elevator;
				float pitch_u = _pitch_ctrl.control_bodyrate(control_input);
				_actuators.control[actuator_controls_s::INDEX_PITCH] = (PX4_ISFINITE(pitch_u)) ? pitch_u + trim_pitch : trim_pitch;
				control_input.scaler = _airspeed_scaling;

				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = PX4_ISFINITE(_rates_sp.thrust_body[0]) ?
						_rates_sp.thrust_body[0] : 0.0f;
			}

			rate_ctrl_status_s rate_ctrl_status{};
			rate_ctrl_status.timestamp = hrt_absolute_time();
			rate_ctrl_status.rollspeed_integ = _roll_ctrl.get_integrator();
			rate_ctrl_status.pitchspeed_integ = _pitch_ctrl.get_integrator();

			if (wheel_control) {
				rate_ctrl_status.additional_integ1 = _wheel_ctrl.get_integrator();

			} else {
				rate_ctrl_status.yawspeed_integ = _yaw_ctrl.get_integrator();
			}

			_rate_ctrl_status_pub.publish(rate_ctrl_status);
		}

		// Add feed-forward from roll control output to yaw control output
		// This can be used to counteract the adverse yaw effect when rolling the plane
		_actuators.control[actuator_controls_s::INDEX_YAW] += _param_fw_rll_to_yaw_ff.get()
				* constrain(_actuators.control[actuator_controls_s::INDEX_ROLL], -1.0f, 1.0f);

		_actuators.control[actuator_controls_s::INDEX_FLAPS] = _flaps_applied;
		_actuators.control[5] = _manual.aux1;
		_actuators.control[actuator_controls_s::INDEX_AIRBRAKES] = _flaperons_applied;
		// FIXME: this should use _vcontrol_mode.landing_gear_pos in the future
		_actuators.control[7] = _manual.aux3;

		/* lazily publish the setpoint only once available */
		_actuators.timestamp = hrt_absolute_time();
		_actuators.timestamp_sample = _att.timestamp;

		/* Only publish if any of the proper modes are enabled */
		if (_vcontrol_mode.flag_control_rates_enabled ||
		    _vcontrol_mode.flag_control_attitude_enabled ||
		    _vcontrol_mode.flag_control_manual_enabled) {
			_actuators_0_pub.publish(_actuators);
		}
	}

	perf_end(_loop_perf);
}

void FixedwingAttitudeControl::control_flaps(const float dt)
{
	/* default flaps to center */
	float flap_control = 0.0f;

	/* map flaps by default to manual if valid */
	if (PX4_ISFINITE(_manual.flaps) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_param_fw_flaps_scl.get()) > 0.01f) {
		flap_control = 0.5f * (_manual.flaps + 1.0f) * _param_fw_flaps_scl.get();

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_param_fw_flaps_scl.get()) > 0.01f) {

		switch (_att_sp.apply_flaps) {
		case vehicle_attitude_setpoint_s::FLAPS_OFF:
			flap_control = 0.0f; // no flaps
			break;

		case vehicle_attitude_setpoint_s::FLAPS_LAND:
			flap_control = 1.0f * _param_fw_flaps_scl.get() * _param_fw_flaps_lnd_scl.get();
			break;

		case vehicle_attitude_setpoint_s::FLAPS_TAKEOFF:
			flap_control = 1.0f * _param_fw_flaps_scl.get() * _param_fw_flaps_to_scl.get();
			break;
		}
	}

	float flaps_dt = dt / _param_fw_flaps_rate.get();

	// move the actual control value continuous with time, full flap travel in
	if (fabsf(_flaps_applied - flap_control) > 0.01f) {
		_flaps_applied += (_flaps_applied - flap_control) < 0 ? flaps_dt : -flaps_dt;

	} else {
		_flaps_applied = flap_control;
	}

	/* default flaperon to center */
	float flaperon_control = 0.0f;

	/* map flaperons by default to manual if valid */
	if (PX4_ISFINITE(_manual.aux2) && _vcontrol_mode.flag_control_manual_enabled
	    && fabsf(_param_fw_flaperon_scl.get()) > 0.01f) {

		flaperon_control = 0.5f * (_manual.aux2 + 1.0f) * _param_fw_flaperon_scl.get();

	} else if (_vcontrol_mode.flag_control_auto_enabled
		   && fabsf(_param_fw_flaperon_scl.get()) > 0.01f) {

		if (_att_sp.apply_flaps == vehicle_attitude_setpoint_s::FLAPS_LAND) {
			flaperon_control = _param_fw_flaperon_scl.get();

		} else {
			flaperon_control = 0.0f;
		}
	}

	// move the actual control value continuous with time, full flap travel in 1sec
	if (fabsf(_flaperons_applied - flaperon_control) > 0.01f) {
		_flaperons_applied += (_flaperons_applied - flaperon_control) < 0 ? flaps_dt : -flaps_dt;

	} else {
		_flaperons_applied = flaperon_control;
	}
}

float FixedwingAttitudeControl::find_trim_from_4_coordinates(float thr_ratio, float as_ratio, float trim_lo_as_lo_thr, float trim_hi_as_lo_thr, float trim_lo_as_hi_thr, float trim_hi_as_hi_thr)
{
	// find average
	float avg = (trim_lo_as_lo_thr + trim_hi_as_lo_thr + trim_lo_as_hi_thr + trim_hi_as_hi_thr) / 4.0f;
	float trim = 0;

	//find which plane to calculate
	if (as_ratio > thr_ratio){
		if (as_ratio < 1.0f - thr_ratio){
			//lower
			trim = math::find_z_from_plane(as_ratio, thr_ratio, 0,0, trim_lo_as_lo_thr, 1,0, trim_hi_as_lo_thr, 0.5f, 0.5f, avg);
		} else{
			//right
			trim = math::find_z_from_plane(as_ratio, thr_ratio, 1,0, trim_hi_as_lo_thr, 1,1, trim_hi_as_hi_thr, 0.5f, 0.5f, avg);
		}
	} else {
		if (thr_ratio < 1.0f - as_ratio){
			//left
			trim = math::find_z_from_plane(as_ratio, thr_ratio, 0,0, trim_lo_as_lo_thr, 0,1, trim_lo_as_hi_thr, 0.5f, 0.5f, avg);
		} else{
			//upper
			trim = math::find_z_from_plane(as_ratio, thr_ratio, 1,1, trim_hi_as_hi_thr, 0,1, trim_lo_as_hi_thr, 0.5f, 0.5f, avg);
		}
	}
	return trim;
}

int FixedwingAttitudeControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	FixedwingAttitudeControl *instance = new FixedwingAttitudeControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int FixedwingAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FixedwingAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
fw_att_control is the fixed wing attitude controller.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("fw_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int fw_att_control_main(int argc, char *argv[])
{
	return FixedwingAttitudeControl::main(argc, argv);
}
