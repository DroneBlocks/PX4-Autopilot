/****************************************************************************
 *
 *   Copyright (c) 2026 DroneBlocks. All rights reserved.
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
 * @file uwb_dwm3001.hpp
 *
 * Driver for a Qorvo DWM3001C acting as a UWB tag (FiRa TWR initiator),
 * speaking the DW3_QM33 SDK CLI over serial.
 *
 * The module emits one JSON block per ranging round:
 *
 *   {"Block":42,"results":[{"Addr":"0x0001","Status":"Ok","D_cm":175.1}, ...]}
 *
 * This driver drives the session (`stop` then `initf`), parses each block,
 * applies the linear antenna-delay correction, de-slants the ranges using the
 * vehicle's height above ground, trilaterates, and publishes:
 *
 *   sensor_uwb               one message per anchor range (raw + calibrated)
 *   vehicle_visual_odometry  the 2D fix, for EKF2 with EKF2_EV_CTRL = 1
 *
 * Z is deliberately NOT provided: altitude comes from the rangefinder. Set
 * EKF2_EV_CTRL = 1 (bit 0, horizontal position only). Sensor lever arm is
 * handled by EKF2_EV_POS_X/Y/Z, not here.
 */

#pragma once

#include <termios.h>
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>

#include <perf/perf_counter.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_uwb.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>

using namespace time_literals;

#define UWB_DWM3001_DEFAULT_PORT "/dev/ttyS3"

static constexpr int   DWM_MAX_ANCHORS = 6;
static constexpr int   DWM_LINE_MAX    = 512;
static constexpr float DWM_MIN_DET     = 1e-9f;

struct dwm_range_s {
	uint16_t addr;      // responder address (0x0001 -> 1)
	float    raw_cm;    // uncalibrated D_cm straight off the module
	float    range_m;   // calibrated, and de-slanted when enabled
	bool     valid;     // module reported Status == "Ok" and survived de-slanting
};

struct dwm_block_s {
	uint32_t     block;
	int          count;
	dwm_range_s  ranges[DWM_MAX_ANCHORS];
};

class UWB_DWM3001 : public ModuleBase<UWB_DWM3001>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	UWB_DWM3001(const char *port);
	~UWB_DWM3001() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void parameters_update();

	/** Open and configure the serial port. Returns true on success. */
	bool openPort();

	/** Send `stop` then `initf` to start a ranging session. */
	void startSession();

	/** Drain the port into _line and dispatch complete lines. */
	void readSerial();

	/** Parse one JSON block. Returns false if the line is not a range block. */
	bool parseBlock(const char *line, dwm_block_s &out) const;

	/** Apply calibration + de-slanting, publish sensor_uwb, solve and publish EV. */
	void handleBlock(dwm_block_s &blk);

	/**
	 * 2D least-squares trilateration. Ported verbatim in behaviour from
	 * droneblocks-uwb positioning/trilateration.py solve_2d().
	 * Coordinates are whatever frame the anchors were surveyed in.
	 */
	static bool solve2d(const float ax[], const float ay[], const float r[],
			    int n, float &x, float &y);

	/** Height of the vehicle above the floor, metres. NAN if unavailable. */
	float heightAgl();

	// Publications
	uORB::PublicationMulti<sensor_uwb_s>  _sensor_uwb_pub{ORB_ID(sensor_uwb)};
	uORB::Publication<vehicle_odometry_s> _visual_odometry_pub{ORB_ID(vehicle_visual_odometry)};

	// Subscriptions
	uORB::Subscription         _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DWM_PORT_CFG>)   _param_port_cfg,
		(ParamInt<px4::params::DWM_ANC_CNT>)    _param_anc_cnt,
		(ParamInt<px4::params::DWM_DESLANT>)    _param_deslant,
		(ParamFloat<px4::params::DWM_RNG_SCALE>) _param_rng_scale,
		(ParamFloat<px4::params::DWM_RNG_OFF>)  _param_rng_off,
		(ParamFloat<px4::params::DWM_POS_NOISE>) _param_pos_noise,
		(ParamFloat<px4::params::DWM_YAW_OFF>)  _param_yaw_off,
		(ParamFloat<px4::params::DWM_A0_X>) _param_a0_x,
		(ParamFloat<px4::params::DWM_A0_Y>) _param_a0_y,
		(ParamFloat<px4::params::DWM_A0_Z>) _param_a0_z,
		(ParamFloat<px4::params::DWM_A1_X>) _param_a1_x,
		(ParamFloat<px4::params::DWM_A1_Y>) _param_a1_y,
		(ParamFloat<px4::params::DWM_A1_Z>) _param_a1_z,
		(ParamFloat<px4::params::DWM_A2_X>) _param_a2_x,
		(ParamFloat<px4::params::DWM_A2_Y>) _param_a2_y,
		(ParamFloat<px4::params::DWM_A2_Z>) _param_a2_z,
		(ParamFloat<px4::params::DWM_A3_X>) _param_a3_x,
		(ParamFloat<px4::params::DWM_A3_Y>) _param_a3_y,
		(ParamFloat<px4::params::DWM_A3_Z>) _param_a3_z,
		(ParamFloat<px4::params::DWM_A4_X>) _param_a4_x,
		(ParamFloat<px4::params::DWM_A4_Y>) _param_a4_y,
		(ParamFloat<px4::params::DWM_A4_Z>) _param_a4_z,
		(ParamFloat<px4::params::DWM_A5_X>) _param_a5_x,
		(ParamFloat<px4::params::DWM_A5_Y>) _param_a5_y,
		(ParamFloat<px4::params::DWM_A5_Z>) _param_a5_z
	)

	/** Fill _anchor_* from the DWM_Ai_* params. Anchor i has address i+1. */
	void loadAnchors();

	float _anchor_x[DWM_MAX_ANCHORS] {};
	float _anchor_y[DWM_MAX_ANCHORS] {};
	float _anchor_z[DWM_MAX_ANCHORS] {};
	int   _anchor_count{0};

	perf_counter_t _block_perf;
	perf_counter_t _fix_perf;
	perf_counter_t _parse_err_perf;

	char _port[32] {};
	int  _uart{-1};

	char _line[DWM_LINE_MAX] {};
	int  _line_len{0};

	bool        _session_started{false};
	hrt_abstime _last_fix{0};
	hrt_abstime _last_block{0};
	uint32_t    _fix_count{0};
	uint32_t    _block_count{0};
};
