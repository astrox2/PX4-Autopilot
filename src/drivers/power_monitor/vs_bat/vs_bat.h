/****************************************************************************
 *
 *   Copyright (C) 2021 PX4 Development Team. All rights reserved.
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


#pragma once


#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <battery/battery.h>
#include <drivers/drv_hrt.h>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/i2c_spi_buses.h>
/*ASTROX BMS code , for warning message */
#include <uORB/topics/mavlink_log.h>
#include <string.h>
#include <px4_platform_common/i2c_spi_buses.h>

using namespace time_literals;

/* Configuration Constants */
#define VS_BAT_BASEADDR 	                    0x41 /* 7-bit address. 8-bit address is 0x41 */
// If initialization is forced (with the -f flag on the command line), but it fails, the drive will try again to
// connect to the VS_BAT every this many microseconds
#define VS_BAT_INIT_RETRY_INTERVAL_US			500000
/* 100 ms * 150 = 15s */

#define VS_BAT_SAMPLE_FREQUENCY_HZ            10
#define VS_BAT_STREAM_TIME VS_BAT_SAMPLE_FREQUENCY_HZ * 150
#define VS_BAT_SAMPLE_INTERVAL_US             (1_s / VS_BAT_SAMPLE_FREQUENCY_HZ)
#define VS_BAT_CONVERSION_INTERVAL            (VS_BAT_SAMPLE_INTERVAL_US - 7)

/* VS_BAT Registers addresses */
#define VS_BAT_REG_PACKET		     (0x0A)			//new registry for packet
#define VS_BAT_ERR_PACKET		     (0x0B)


#define MAX_CURRENT                           164.0f    /* 164 Amps */
#define VS_BAT_DN_MAX                         32768.0f  /* 2^15 */
#define VS_BAT_CONST                          0.00512f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define VS_BAT_SHUNT                          0.0005f   /* Shunt is 500 uOhm */
#define VS_BAT_VSCALE                         0.00125f  /* LSB of voltage is 1.25 mV  */

#define swap16(w)                       __builtin_bswap16((w))


/*Astrox BMS code, BMS 통신모듈으로 부터 얻어온 비트 단위의 데이터를 처리하기 위한 구조체.*/
typedef union {
  struct {
    uint8_t _ovp :1;      // cell over volt
    uint8_t _uvp :1;      // cell under volt
    uint8_t _dc_otp :1;   // cell over temperature
    uint8_t _dc_utp :1;   // cell under temperature
    uint8_t _c_ovp :1;
    uint8_t _c_uvp :1;
    uint8_t _diff :1;     // occurred cell voltage level differencies
    uint8_t pack_ovp :1;   // pack over voltage
  }bit;
  uint8_t buf;
}warn_module;

typedef union {
  struct {
    uint8_t _uvp:1;     // pack under volrage
    uint8_t _otp:1;     // pack over temperature
    uint8_t _utp:1;     // pack under temperature

	/* overcurrent protection ? */
    uint8_t _c_ocp:1;   // connect pack over current => connect overcurrent protection ?
    uint8_t _dc_ocp:1;  // disconnect pack over current => disconnect overcurrent protection ?

    uint8_t _low_soc:1; // low SOC
    uint8_t _contactor:1;
    uint8_t _slave_commu:1;
  }bit;
  uint8_t buf;
}warn_pack;

typedef union {
  struct {
    uint8_t _slave_contactor:1; // slave contactor

    uint8_t _int_otp:1;         // ??

    uint8_t _ic_otp:1;          // ic over temperature
    uint8_t _swtich_otp:1;      // Switch over temperature

    /* NTC :  Negative Temperature Coefficient Thermistor */
    uint8_t _ntc_discon:1;      // NTC disconnected

    uint8_t reserve:2;         // not use

    uint8_t _aux_otp:1;          // over temperature at BMS output line
  }bit;
  uint8_t buf;
}warn_slave;

typedef union {
  struct {
    uint8_t _module_cell_ovp:1; // cell over voltage protection activate.
    uint8_t _module_cell_uvp:1; // cell under voltage protection activate.
    uint8_t _module_pack_otp:1; // pack over temperature protection activate.
    uint8_t _module_pack_utp:1; // pack under temperature protection activate.
    uint8_t _module_cell_otp:1; // cell over temperature protection activate.
    uint8_t _module_cell_utp:1; // cell under temperature protection activate.
    uint8_t _modules_diff:1;    // module voltage level differencies protection activate.
    uint8_t _pack_ovp:1;         // pack over voltage protection activate.
  }bit;
    uint8_t buf;
}fault_act1;

typedef union {
  struct {
    uint8_t _pack_uvp:1;      // pack over voltage protection activate.
    uint8_t _pack_otp:1;      // pack over temperature prorection activate.
    uint8_t _pack_utp:1;      // pack under temperature protection activate.
    uint8_t _cell_ocp:1;      // cell over current protection activate.
    uint8_t _dc_ocp:1;        //
    uint8_t _low_soc:1;       // detected low SOC. fault.
    uint8_t _contactor:1;     // detected contactor connection error.
    uint8_t _slave_commu:1;   // communication with slaves are failed.
  }bit;
    uint8_t buf;
}fault_act2;

typedef union {
  struct {
    uint8_t _slave_contactor:1; // connection from slave be disconnected!
    uint8_t _int_otp:1;         // over temperature fault be activated from pack.
    uint8_t _ic_otp:1;          // over temperature fault be activated from IC.
    uint8_t _switch_otp:1;      // over temperature fault be activated from switch.
    uint8_t _ntc_discon:1;
    uint8_t reserve:2;
    uint8_t _aux_otp:1;         // over temperature fault be activated from BMS Output.
  }bit;
  uint8_t buf;
}fault_etc;

struct BAT_CAN
{
	uint8_t ucUniq;
	uint8_t ucSOC;
	uint8_t ucVolHI;
	uint8_t ucVolLO;
	uint8_t ucCurHI;
	uint8_t ucCurLo;
	uint8_t ucTEMPMax;
	uint8_t ucTEMPMin;
	uint8_t ucTEMPAve;
	uint8_t ucTEMPAux;
	warn_module w_module;
	warn_pack w_pack;
	warn_slave w_slave;
	fault_act1 f_act1;
	fault_act2 f_act2;
	fault_etc f_etc;
};


class VS_BAT : public device::I2C, public ModuleParams, public I2CSPIDriver<VS_BAT>
{
public:
	VS_BAT(const I2CSPIDriverConfig &config, int battery_index);
	virtual ~VS_BAT();

	static I2CSPIDriverBase *instantiate(const I2CSPIDriverConfig &config, int runtime_instance);
	static void print_usage();

	void RunImpl();

	int init() override;

	/**
	 * Tries to call the init() function. If it fails, then it will schedule to retry again in
	 * VS_BAT_INIT_RETRY_INTERVAL_US microseconds. It will keep retrying at this interval until initialization succeeds.
	 *
	 * @return PX4_OK if initialization succeeded on the first try. Negative value otherwise.
	 */
	int force_init();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_status() override;

// protected:
// 	// int probe() override;

private:
	bool 		_sensor_ok{false};
	unsigned int 	_measure_interval{0};
	bool 		_collect_phase{false};
	bool 		_initialized{false};

	perf_counter_t 	_sample_perf;
	perf_counter_t 	_comms_errors;
	perf_counter_t 	_collection_errors;

	// Configuration state, computed from params
	float 		_max_current{MAX_CURRENT};
	float 		_rshunt{VS_BAT_SHUNT};
	float 		_current_lsb;
	int16_t 	_range;

	/* for mediate printing time */
	int64_t         _power{0};

	int16_t         _shunt{0};
	int16_t         _cal{0};
	bool            _mode_triggered{false};

	/*ASTROX BMS code
		1. Vspace 에서 제공된 i2c 데이터 정보를 기반으로 하는 uORB 메세지는 Battery_status이다.
		   배터리를 사용하는 모든 드라이버를 고유의 인덱스를 갖는데, 
		   이 드라이버의 인덱스를 3번(4-1로 계산) 표현하기 위함이다.
		2. BAT_CAN 은 BMS에서 제공되는 에러 및 경고 메세지를 처리하기 위한 구조체이다.
		3. _low_warn, _low_critical 은 파라메터에서 관리가 가능하도록 만들어진 변수이며, 
		각각 1차 경고, 2차 critical 레벨을 지정하기 위한 파라메터이다.
	*/
		// for battery failsafe mode.
		uint16_t 	  _low_warn{0};
		uint16_t 	  _low_critical{0};
		int16_t 	  _battery_status;

		/* i2c packet data from bridge board.
		   uORB 에서 계산되는 데이터를 제외하고, i2c 통신으로부터 얻는 데이터로 변경하기위함.
		   vs_bat 드라이버에서 사용되는 메인 변수는 아래와 같다.
		   이 데이터 들은 i2C로 부터 얻어오는 데이터 들이다.*/
		int16_t		_bus_voltage{0};	// 전압
		int16_t 	_soc{0};		// State of charge 라고하는 충전율 을 의미함.
		int16_t         _current{0};		// 전류
		int16_t         _temperature{0};	// 온도
		BAT_CAN		_can_packet;		/* 에러 및 경고 메세지 처리를 위한 구조체*/

		/* mavlink 메세지중에 text_status 라는 것을 이용해 경고 및 에러 메세지를 보내기 위함.*/
		mavlink_log_s _mavlink_log_s;
		uORB::Publication<mavlink_log_s> _mavlink_log_pub{ORB_ID(mavlink_log)};

		/* for mediate printing time */
		uint8_t 	cnt;
		char 		buf[127];

		Battery	_battery;
		/*이 코드는 기존의 코드와 구조가 다르며, BAT_CAN 구조체에 통신데이터를 저장하기 위한 멤버함수이다.*/
		int read(uint8_t address, BAT_CAN &packet);

	int read(uint8_t address, int16_t &data);
	int write(uint8_t address, uint16_t data);


	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void start();

	int collect();

};
