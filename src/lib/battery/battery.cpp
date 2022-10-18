/****************************************************************************
 *
 *   Copyright (c) 2019-2021 PX4 Development Team. All rights reserved.
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
 * @file battery.cpp
 *
 * Library calls for battery functionality.
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Timothy Scott <timothy@auterion.com>
 */

#include "battery.h"
#include <mathlib/mathlib.h>
#include <cstring>
#include <px4_platform_common/defines.h>

using namespace time_literals;

Battery::Battery(int index, ModuleParams *parent, const int sample_interval_us, const uint8_t source) :
	ModuleParams(parent),
	_index(index < 1 || index > 9 ? 1 : index),
	_source(source)
{
	const float expected_filter_dt = static_cast<float>(sample_interval_us) / 1_s;
	_voltage_filter_v.setParameters(expected_filter_dt, 1.f);
	_current_filter_a.setParameters(expected_filter_dt, .5f);
	_current_average_filter_a.setParameters(expected_filter_dt, 50.f);
	_throttle_filter.setParameters(expected_filter_dt, 1.f);

	if (index > 9 || index < 1) {
		PX4_ERR("Battery index must be between 1 and 9 (inclusive). Received %d. Defaulting to 1.", index);
	}

	// 16 chars for parameter name + null terminator
	char param_name[17];

	snprintf(param_name, sizeof(param_name), "BAT%d_V_EMPTY", _index);
	_param_handles.v_empty = param_find(param_name);

	if (_param_handles.v_empty == PARAM_INVALID) {
		PX4_ERR("Could not find parameter with name %s", param_name);
	}

	snprintf(param_name, sizeof(param_name), "BAT%d_V_CHARGED", _index);
	_param_handles.v_charged = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_N_CELLS", _index);
	_param_handles.n_cells = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_CAPACITY", _index);
	_param_handles.capacity = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_V_LOAD_DROP", _index);
	_param_handles.v_load_drop = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_R_INTERNAL", _index);
	_param_handles.r_internal = param_find(param_name);

	snprintf(param_name, sizeof(param_name), "BAT%d_SOURCE", _index);
	_param_handles.source = param_find(param_name);

	_param_handles.low_thr = param_find("BAT_LOW_THR");
	_param_handles.crit_thr = param_find("BAT_CRIT_THR");
	_param_handles.emergen_thr = param_find("BAT_EMERGEN_THR");

	_param_handles.bat_avrg_current = param_find("BAT_AVRG_CURRENT");

	updateParams();
}

void Battery::updateVoltage(const float voltage_v)
{
	_voltage_v = voltage_v;
	_voltage_filter_v.update(voltage_v);
}

void Battery::updateCurrent(const float current_a)
{
	_current_a = current_a;
	_current_filter_a.update(current_a);
}

/*ASTROX BMS code, for update battery status data from bms bridge board*/
void Battery::updateSOC(const float soc, const float temp, const uint8_t warning, bool connected)
{
	/* SOC 는 반드시 소수점으로 표현해야만 한다. 또한 스캐일링을 해야만 한다. 기본적으로 10을 나눈 값으로 동작된다.
		아래 3개의 데이터는 다른 드라이버에서도 사용되는 변수이다.

		이는 모두 SOC를 기반으로 계산되어 업데이트가 되도록 구성된다.
		vs_bat 드라이버에서는 이미 계산된 데이터가 전달되므로, 아래처럼
		변수로부터 바로 업데이트 되게끔하여 계산 부분을 스킵하도록하였다.
	*/
	_state_of_charge = soc;
	_warning = warning;
	_connected = connected;
	/*온도 데이터는 다른 드라이버에서는 기본적으로 사용하지 않으므로 추가하였음.*/
	_temperature = temp;
}

/*배터리 데이터를 업데이트 하는 함수이다.*/
void Battery::updateBatteryStatus(const hrt_abstime &timestamp)
{
	if (!_battery_initialized) {
		_voltage_filter_v.reset(_voltage_v);
		_current_filter_a.reset(_current_a);
	}

	sumDischarged(timestamp, _current_a);
	estimateStateOfCharge(_voltage_filter_v.getState(), _current_filter_a.getState());
	/*vs_bat 드라이버의 index 가 4 이므로, 4일 경우에는 이부분을 스킵하도록 설정*/
	if(_index != 4)
	{
		computeScale();

		if (_connected && _battery_initialized)
		{
			_warning = determineWarning(_state_of_charge);
		}
	}

	if (_voltage_filter_v.getState() > 2.1f) {
		_battery_initialized = true;

	} else {
		_connected = false;
	}
}

battery_status_s Battery::getBatteryStatus()
{
	battery_status_s battery_status{};
	battery_status.voltage_v = _voltage_v;
	/* vs-bat 드라이버에서 제공되는 전압 데이터를 2개의 셀 데이터로 넘기기 위한 작업이다.
	   PX4에서 기본적으로 제공하는 셀의 최대 갯수가 16개 이므로, 24셀의 데이터를 표현하는것은 기본적으로 불가능하다.
	   cell_count 변수를 통해 배터리 셀의 갯수를 표현하고, 이 cell_count 를 기반으로 배터리 계산이 이루어 진다.

		ex) 배터리 전압 계산방법 ..
	   	for( int i = 0 ; i< cell_count; i++ )
		 {
			배터리 전압 += voltage_cell_v[i];
		 }

	vs_bat 드라이버에서는 셀 갯수를 2개로 지정하고, 2개의 셀 데이터에 배터리 전압을 2로 나눈 데이터를 집어 넣도록 구성하였다.
	배터리의 전압 계산은 voltage_cell_v[] 배열 데이터의 합으로 구성되기 때문에 이러한 구조를 취했다.
	cell_count = BAT4_N_CELLS = 2;  BAT4_N_CELLS 파라메터는
	drivers/power_monitor/vs_bat/vs_bat_param.c 파일에서 확인이 가능하다.
	*/
	if(_index == 4 ) {
		battery_status.voltage_cell_v[0] = _voltage_v/2;
		battery_status.voltage_cell_v[1] = _voltage_v - battery_status.voltage_cell_v[0];
	}

	battery_status.voltage_filtered_v = _voltage_filter_v.getState();
	battery_status.current_a = _current_a;
	battery_status.current_filtered_a = _current_filter_a.getState();
	battery_status.current_average_a = _current_average_filter_a.getState();
	battery_status.discharged_mah = _discharged_mah;
	battery_status.remaining = _state_of_charge;
	battery_status.scale = _scale;
	battery_status.time_remaining_s = computeRemainingTime(_current_a);
	battery_status.temperature = _temperature;
	battery_status.cell_count = _params.n_cells;
	battery_status.connected = _connected;
	battery_status.source = _source;
	battery_status.priority = _priority;
	battery_status.capacity = _params.capacity > 0.f ? static_cast<uint16_t>(_params.capacity) : 0;
	battery_status.id = static_cast<uint8_t>(_index);
	battery_status.warning = _warning;
	battery_status.timestamp = hrt_absolute_time();
	return battery_status;
}

void Battery::publishBatteryStatus(const battery_status_s &battery_status)
{
	if (_source == _params.source) {
		_battery_status_pub.publish(battery_status);
	}
}

void Battery::updateAndPublishBatteryStatus(const hrt_abstime &timestamp)
{
	updateBatteryStatus(timestamp);
	publishBatteryStatus(getBatteryStatus());
}

void Battery::sumDischarged(const hrt_abstime &timestamp, float current_a)
{
	// Not a valid measurement
	if (current_a < 0.f) {
		// Because the measurement was invalid we need to stop integration
		// and re-initialize with the next valid measurement
		_last_timestamp = 0;
		return;
	}

	// Ignore first update because we don't know dt.
	if (_last_timestamp != 0) {
		const float dt = (timestamp - _last_timestamp) / 1e6;
		// mAh since last loop: (current[A] * 1000 = [mA]) * (dt[s] / 3600 = [h])
		_discharged_mah_loop = (current_a * 1e3f) * (dt / 3600.f);
		_discharged_mah += _discharged_mah_loop;
	}

	_last_timestamp = timestamp;
}

void Battery::estimateStateOfCharge(const float voltage_v, const float current_a)
{
	// remaining battery capacity based on voltage
	float cell_voltage= {0.0f};

	cell_voltage = voltage_v / _params.n_cells;


	// correct battery voltage locally for load drop to avoid estimation fluctuations
	if (_params.r_internal >= 0.f)
	{
		cell_voltage += _params.r_internal * current_a;

	} else {
		actuator_controls_s actuator_controls{};
		_actuator_controls_0_sub.copy(&actuator_controls);
		const float throttle = actuator_controls.control[actuator_controls_s::INDEX_THROTTLE];
		_throttle_filter.update(throttle);

		if (!_battery_initialized) {
			_throttle_filter.reset(throttle);
		}

		// assume linear relation between throttle and voltage drop
		cell_voltage += throttle * _params.v_load_drop;
	}
	/* ASTROX BMS code, BMS 배터리 드라이버의 index를 4로 지정하였기 때문에,
	인덱스가 4인경우에는 SOC 추정계산 코드를 넘어가도록 함. 이 외에는 정상적으로 동작.*/
	if(_index != 4 ){
		_state_of_charge_volt_based = math::interpolate(cell_voltage, _params.v_empty, _params.v_charged, 0.f, 1.f);

	// choose which quantity we're using for final reporting
	if (_params.capacity > 0.f && _battery_initialized) {
		// if battery capacity is known, fuse voltage measurement with used capacity
		// The lower the voltage the more adjust the estimate with it to avoid deep discharge
		const float weight_v = 3e-4f * (1 - _state_of_charge_volt_based);
		_state_of_charge = (1 - weight_v) * _state_of_charge + weight_v * _state_of_charge_volt_based;
		// directly apply current capacity slope calculated using current
		_state_of_charge -= _discharged_mah_loop / _params.capacity;
		_state_of_charge = math::max(_state_of_charge, 0.f);

		const float state_of_charge_current_based = math::max(1.f - _discharged_mah / _params.capacity, 0.f);
		_state_of_charge = math::min(state_of_charge_current_based, _state_of_charge);

	} else {
		_state_of_charge = _state_of_charge_volt_based;
		}
	}
}

uint8_t Battery::determineWarning(float state_of_charge)
{
	if (state_of_charge < _params.emergen_thr) {
		return battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (state_of_charge < _params.crit_thr) {
		return battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (state_of_charge < _params.low_thr) {
		return battery_status_s::BATTERY_WARNING_LOW;

	} else {
		return battery_status_s::BATTERY_WARNING_NONE;
	}
}

void Battery::computeScale()
{
	const float voltage_range = (_params.v_charged - _params.v_empty);

	// reusing capacity calculation to get single cell voltage before drop
	const float bat_v = _params.v_empty + (voltage_range * _state_of_charge_volt_based);

	_scale = _params.v_charged / bat_v;

	if (_scale > 1.3f) { // Allow at most 30% compensation
		_scale = 1.3f;

	} else if (!PX4_ISFINITE(_scale) || _scale < 1.f) { // Shouldn't ever be more than the power at full battery
		_scale = 1.f;
	}
}

float Battery::computeRemainingTime(float current_a)
{
	float time_remaining_s = NAN;

	if (_vehicle_status_sub.updated()) {
		vehicle_status_s vehicle_status;

		if (_vehicle_status_sub.copy(&vehicle_status)) {
			_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		}
	}

	if (!PX4_ISFINITE(_current_average_filter_a.getState()) || _current_average_filter_a.getState() < FLT_EPSILON) {
		_current_average_filter_a.reset(_params.bat_avrg_current);
	}

	if (_armed && PX4_ISFINITE(current_a)) {
		// only update with positive numbers
		_current_average_filter_a.update(fmaxf(current_a, 0.f));
	}

	// Remaining time estimation only possible with capacity
	if (_params.capacity > 0.f) {
		const float remaining_capacity_mah = _state_of_charge * _params.capacity;
		const float current_ma = fmaxf(_current_average_filter_a.getState() * 1e3f, FLT_EPSILON);
		time_remaining_s = remaining_capacity_mah / current_ma * 3600.f;
	}

	return time_remaining_s;
}

void Battery::updateParams()
{
	param_get(_param_handles.v_empty, &_params.v_empty);
	param_get(_param_handles.v_charged, &_params.v_charged);
	param_get(_param_handles.n_cells, &_params.n_cells);
	param_get(_param_handles.capacity, &_params.capacity);
	param_get(_param_handles.v_load_drop, &_params.v_load_drop);
	param_get(_param_handles.r_internal, &_params.r_internal);
	param_get(_param_handles.source, &_params.source);
	param_get(_param_handles.low_thr, &_params.low_thr);
	param_get(_param_handles.crit_thr, &_params.crit_thr);
	param_get(_param_handles.emergen_thr, &_params.emergen_thr);
	param_get(_param_handles.bat_avrg_current, &_params.bat_avrg_current);

	ModuleParams::updateParams();

	_first_parameter_update = false;
}
