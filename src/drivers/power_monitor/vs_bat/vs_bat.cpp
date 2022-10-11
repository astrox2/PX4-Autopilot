/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * Driver for the I2C attached VS_BAT
 */

#include "vs_bat.h"


VS_BAT::VS_BAT(const I2CSPIDriverConfig &config, int battery_index) :
	I2C(config),
	ModuleParams(nullptr),
	I2CSPIDriver(config),
	_sample_perf(perf_alloc(PC_ELAPSED, "vs_bat_read")),
	_comms_errors(perf_alloc(PC_COUNT, "vs_bat_com_err")),
	_collection_errors(perf_alloc(PC_COUNT, "vs_bat_collection_err")),
	_battery(battery_index, this, VS_BAT_SAMPLE_INTERVAL_US, battery_status_s::BATTERY_SOURCE_POWER_MODULE)
{
		/*PAV_BAT_WARN 이라는 파라메터를 이 드라이버에서 사용하기 위해 _low_warn 변수에다가 저장하여 사용.*/
		param_t ph = param_find("PAV_BAT_WARN");
		int32_t value = param_get(ph,&value);
		if(ph != PARAM_INVALID && param_get(ph,&value) == PX4_OK) {
			_low_warn = (uint16_t)value;
			PX4_INFO("PAV warn volt :%d",_low_warn);
		}

		ph = param_find("PAV_BAT_CRITICAL");
		value = param_get(ph,&value);
		if(ph != PARAM_INVALID && param_get(ph,&value) == PX4_OK) {
			_low_critical = (uint16_t)value;
			PX4_INFO("PAV critical volt :%d",_low_critical);
		}
		/*CAN 패킷 구조체 초기화를 위한 작업. */
		_can_packet={0};

		// We need to publish immediately, to guarantee that the first instance of the driver publishes to uORB instance 0
		_battery.setConnected(false);
		_battery.updateVoltage(0.f);
		_battery.updateCurrent(0.f);
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());
}

VS_BAT::~VS_BAT()
{
	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_collection_errors);
}

int VS_BAT::read(uint8_t address, int16_t &data)
{
	// read desired little-endian value via I2C
	uint16_t received_bytes;
	const int ret = transfer(&address, 1, (uint8_t *)&received_bytes, sizeof(received_bytes));

	if (ret == PX4_OK) {
		data = swap16(received_bytes);

	} else {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}

	return ret;
}

int VS_BAT::read(uint8_t address, BAT_CAN &packet)
{

	// read desired little-endian value via I2C
	if(address == VS_BAT_REG_PACKET) {
		unsigned char data[10];
		const int ret = transfer(&address, 1, (uint8_t*)&data, sizeof(data));
		if (ret == PX4_OK) {
			packet.ucUniq=data[0];
			packet.ucSOC=data[1];
			packet.ucVolLO=data[2];
			packet.ucVolHI=data[3];
			packet.ucCurLo=data[4];
			packet.ucCurHI=data[5];
			packet.ucTEMPMax=data[6];
			packet.ucTEMPMin=data[7];
			packet.ucTEMPAve=data[8];
			packet.ucTEMPAux=data[9];

		} else {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}

		return ret;

	}
	else if(address == VS_BAT_ERR_PACKET){
		unsigned char data[6];
		const int ret = transfer(&address, 1, (unsigned char *)&data, sizeof(data));
		if (ret == PX4_OK) {
			packet.w_module.buf = data[0];
			packet.w_pack.buf = data[1];
			packet.w_slave.buf = data[2];
			packet.f_act1.buf = data[3];
			packet.f_act2.buf = data[4];
			packet.f_etc.buf = data[5];

		} else {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}

		return ret;
	}

	return PX4_OK;
}

int VS_BAT::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	return transfer(data, sizeof(data), nullptr, 0);
}

int VS_BAT::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != PX4_OK) {
		return ret;
	}

	// Start reading data from bms bridge board.
	ret = write(VS_BAT_BASEADDR, (uint16_t)VS_BAT_REG_PACKET);

	start();
	_sensor_ok = true;

	_initialized = ret == PX4_OK;
	return ret;
}

int VS_BAT::force_init()
{
	int ret = init();

	start();

	return ret;
}

// int VS_BAT::probe()
// {
// 	uint16_t value{0};

// 	if (read(VS_BAT_BASE_ADDR, value) != PX4_OK || value != VS_BAT_BASE_ADDR) {
// 		PX4_DEBUG("probe mfgid %d", value);
// 		return -1;
// 	}

// 	return PX4_OK;
// }


/* 실질적으로 드라이버가 운용될 떄 실행되는 메인 함수와 같음.*/
int VS_BAT::collect()
{
	perf_begin(_sample_perf);

	// read from the sensor
	// Note: If the power module is connected backwards, then the values of _current will be negative but otherwise valid.
	bool success{true};
	// int16_t bus_voltage{0};
	// int16_t current{0};

	success = success && (read(VS_BAT_REG_PACKET, _can_packet) == PX4_OK);
	success = success && (read(VS_BAT_ERR_PACKET,_can_packet) == PX4_OK);

	/*i2c 통신으로 부터 얻어온 데이터를 변수에 업데이트.*/
	_bus_voltage = (uint16_t)(_can_packet.ucVolHI << 8 | _can_packet.ucVolLO); 	/* 전압 */
	_current = (uint16_t)(_can_packet.ucCurHI << 8 | _can_packet.ucCurLo);		/* 전류 */
	_temperature = (uint16_t)(_can_packet.ucTEMPAve);				/* 온도 */
	_soc = (uint16_t)(_can_packet.ucSOC);						/* SOC, 충전률 */

	_battery_status = battery_status_s::BATTERY_WARNING_NONE; /* 배터리의 기본 모드를 nomal로 설정 */


	/*
		에러 및 경고 메세지를 15초 마다 한번씩만 실행하도록 코드를 구성
		mavlink 메세지는 128가 넘지 않아야 하므로 아래의 모든 메세지는 128바이트가 넘지 않도록 계산하였다.
	*/
	cnt ++;
	if (cnt > VS_BAT_STREAM_TIME) {
		if(_can_packet.w_module.buf > 0 || _can_packet.w_pack.buf > 0 || _can_packet.w_slave.buf > 0 ) {
			_mavlink_log_s.timestamp = hrt_absolute_time(); /*mavlink 메세지를 보내기 위해선 항상 TimeStamp가 추가가 되어야 함으로 이를 얻어옴*/
			_battery_status = battery_status_s::BATTERY_WARNING_NONE;
			memset(buf, 0, sizeof(buf));
			strcat(buf,"BMS WARNING! ");							 	// 13 byte

			if(_can_packet.w_module.bit._dc_otp > 0) strcat(buf,"C_OV_T |");			// 9 byte
			// if(_can_packet.w_module.bit._ovp) 	buf, "OV_V | ";
			// if(_can_packet.w_module.bit._uvp) 	buf,"BMS C_UNDER VOLTAGE |");
			// if(_can_packet.w_module.bit._dc_otp)	buf,"BMS C_OVER TEMPERATURE |");
			// if(_can_packet.w_module.bit._dc_utp)	buf,"BMS C_UNDER TEMPERATURE |");
			if(_can_packet.w_module.bit._diff) 	strcat(buf,"CELL_DIFF |");			// 11 byte
			// if(_can_packet.w_module.bit.pack_ovp)	buf,"  P_OVER VOLTAGE |");

			if(_can_packet.w_pack.bit._c_ocp)	strcat(buf,"P_OV_C |");			// 9 byte
			if(_can_packet.w_pack.bit._otp) 	strcat(buf,"P_UN_T |");			// 9 byte
			if(_can_packet.w_pack.bit._utp) 	strcat(buf,"P_OV_T |");			// 9 byte
			if(_can_packet.w_pack.bit._dc_ocp) 	strcat(buf,"P_UN_C |");			// 9 byte
			if(_can_packet.w_pack.bit._slave_commu) strcat(buf,"P_SLV_OVH |");		// 11 byte
			if(_can_packet.w_pack.bit._low_soc) 	strcat(buf,"SOC_L ");
			strcat(buf,"\n\r"); /*개행을 붙혀여만 그 즉시 메세지가 전송됨.*/
			strcpy(_mavlink_log_s.text ,buf);
			PX4_WARN("%s ",_mavlink_log_s.text);
			_mavlink_log_pub.publish(_mavlink_log_s);
		}

		if(_can_packet.f_act1.buf > 0 || _can_packet.f_act2.buf > 0 || _can_packet.f_etc.buf > 0)	{

			_mavlink_log_s.timestamp = hrt_absolute_time();
			_battery_status = battery_status_s::BATTERY_WARNING_CRITICAL;
			strcat(buf,"BMS ERROR! CELL :");						// 17 byte
			if(_can_packet.f_act1.bit._module_cell_ovp)	strcat(buf," OV_V |");		// 7 byte * 3 = 21 byte
			if(_can_packet.f_act1.bit._module_cell_uvp)	strcat(buf," UV_V |");
			if(_can_packet.f_act1.bit._module_cell_otp)	strcat(buf," OV_T |");
			// if(_can_packet.f_act1.bit._module_cell_utp)	strcat(buf," UN_T |");
			// if(_can_packet.f_act2.bit._cell_ocp)		strcat(buf," OV_V |");
			strcat(buf, "PACK : ");								// 7 byte
			// if(_can_packet.f_act1.bit._module_pack_otp)	strcat(buf," OV_V |");		// 7 *4  = 28 byte
			if(_can_packet.f_act1.bit._module_pack_utp)	strcat(buf," UN_T |");
			if(_can_packet.f_act1.bit._pack_ovp)		strcat(buf," OV_V |");
			if(_can_packet.f_act2.bit._pack_otp)		strcat(buf," OV_T |");
			// if(_can_packet.f_act2.bit._pack_utp)		strcat(buf," UN_T |");
			if(_can_packet.f_act2.bit._pack_uvp)		strcat(buf," UN_V |");
			if(_can_packet.f_act1.bit._modules_diff)	strcat(buf," V_DIFF |");	// 8 byte
			if(_can_packet.f_act2.bit._low_soc)		strcat(buf," SOC_L |");		// 7 byte
			if(_can_packet.f_act2.bit._slave_commu)		strcat(buf," SLV_CON |");	// 9 byte
			if(_can_packet.f_etc.bit._aux_otp)		strcat(buf, " P_CON_OVH |");	// 10 byte
			strcat(buf,"\n\r");
			strcpy(_mavlink_log_s.text ,buf);
			PX4_WARN("%s ",_mavlink_log_s.text);
			_mavlink_log_pub.publish(_mavlink_log_s);
		}
		cnt = 0;
	}

	/* 위에서 얻은 경고 및 에러 상태에 따라 배터리의 모드를 변경함. 현재 이 코드에서는 동작하지 않도록 설정함.*/
	if(_can_packet.f_act1.buf  == 0 && _can_packet.f_act2.buf  == 0 && _can_packet.f_etc.buf  == 0 &&
	   _can_packet.w_module.buf == 0 && _can_packet.w_pack.buf == 0 && _can_packet.w_slave.buf == 0 && _bus_voltage >_low_warn) {
		_battery_status = battery_status_s::BATTERY_WARNING_NONE;
	}

	/* 배터리 전압에 따라 배터리의 모드를 변경
		BATTERY_WARNING_LOW 	 : 배터리의 전압이 낮다는 경고메세지를 보내기 위해 배터리의 상태를 LOW 상태로 변경
		BATTERY_WARNING_CRITICAL : 전압이 지정한 파라메터보다 낮을 경우에 비행모드를 return to home 모드로 변경하게끔 배터리의 상태를 Critical로 변경.
	*/
	if(_bus_voltage <= _low_warn && _bus_voltage > _low_critical) {_battery_status = battery_status_s::BATTERY_WARNING_LOW;}
	else if (_bus_voltage < _low_critical) {_battery_status = battery_status_s::BATTERY_WARNING_CRITICAL;}



	if (!success) {
		PX4_DEBUG("error reading from sensor");
		_bus_voltage = _power = _current = _shunt = 0;
	}
	// _actuators_sub.copy(&_actuator_controls); // 이는 1.12.3 버전에서는 사용되었으나, 1.13.0 버전에서는 battery.cpp로 옮겨감.


	/* stable 1.13.0 버전에서 업데이트 된 battery 드라이버 코드 부분이다. 이 내용들은 lib/battery/battery.cpp 에서 확인이 가능하다.
	    아래의 코드는 uORB 메세지를 업데이트하면서 mavlink 메세지로 전달하기 위한 class인 battery를 기반으로 동작한다.
	*/
	_battery.setConnected(success);					// 배터리의 연결 상태 결정, true : 연결됨 , false : 연결되지 않음
	_battery.updateVoltage(static_cast<float>(_bus_voltage*0.1f));  // 배터리 전압의 업데이트.
	_battery.updateCurrent(static_cast<float>(_current*0.1f));	// 배터리 전류의 업데이트.

	/*이는 기존의 battery.cpp 에서 없어서 추가한 코드이다. SOC 를 계산하기 위한 코드를 스킵하기 위해 작성되었다.*/
	_battery.updateSOC(static_cast<float>(_soc),static_cast<float>(_temperature),_battery_status,true);

	_battery.updateAndPublishBatteryStatus(hrt_absolute_time());/* urob 메세지를 업데이트 */

	perf_end(_sample_perf);

	if (success) {
		return PX4_OK;

	} else {
		return PX4_ERROR;
	}
}

void VS_BAT::start()
{
	ScheduleClear();

	/* reset the report ring and state machine */
	_collect_phase = false;

	_measure_interval = VS_BAT_CONVERSION_INTERVAL;

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void VS_BAT::RunImpl()
{
	if (_initialized) {
		if (_collect_phase) {
			/* perform collection */
			if (collect() != PX4_OK) {
				perf_count(_collection_errors);
				/* if error restart the measurement state machine */
				start();
				return;
			}

			/* next phase is measurement */
			_collect_phase = true;

			if (_measure_interval > VS_BAT_CONVERSION_INTERVAL) {
				/* schedule a fresh cycle call when we are ready to measure again */
				ScheduleDelayed(_measure_interval - VS_BAT_CONVERSION_INTERVAL);
				return;
			}
		}

		/* next phase is collection */
		_collect_phase = true;

		/* schedule a fresh cycle call when the measurement is done */
		ScheduleDelayed(VS_BAT_CONVERSION_INTERVAL);

	} else {
		_battery.setConnected(false);
		_battery.updateVoltage(0.f);
		_battery.updateCurrent(0.f);
		_battery.updateSOC(0.0f,0.0f,0,false);
		_battery.updateAndPublishBatteryStatus(hrt_absolute_time());

		if (init() != PX4_OK) {
			ScheduleDelayed(VS_BAT_INIT_RETRY_INTERVAL_US);
		}
	}
}

void VS_BAT::print_status()
{
	I2CSPIDriverBase::print_status();
	PX4_INFO("re assembled int , voltage : %d, current : %d, temp : %d , SOC: %d",_bus_voltage, _current, _temperature, _soc);

	if (_initialized) {
		perf_print_counter(_sample_perf);
		perf_print_counter(_comms_errors);

		printf("poll interval:  %u \n", _measure_interval);

	} else {
		PX4_INFO("Device not initialized. Retrying every %d ms until battery is plugged in.",
			 VS_BAT_INIT_RETRY_INTERVAL_US / 1000);
	}
}
