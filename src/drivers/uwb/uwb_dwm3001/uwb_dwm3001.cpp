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

#include "uwb_dwm3001.hpp"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <mathlib/mathlib.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#define DWM_BAUD B115200

UWB_DWM3001::UWB_DWM3001(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_block_perf(perf_alloc(PC_COUNT, MODULE_NAME": blocks")),
	_fix_perf(perf_alloc(PC_COUNT, MODULE_NAME": fixes")),
	_parse_err_perf(perf_alloc(PC_COUNT, MODULE_NAME": parse errors"))
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';
}

UWB_DWM3001::~UWB_DWM3001()
{
	if (_uart >= 0) {
		// Leave the module idle rather than mid-session, otherwise the next
		// start has to fight an already-running session.
		const char *stop = "stop\r\n";
		(void)::write(_uart, stop, strlen(stop));
		::close(_uart);
	}

	perf_free(_block_perf);
	perf_free(_fix_perf);
	perf_free(_parse_err_perf);
}

bool UWB_DWM3001::init()
{
	loadAnchors();

	if (_anchor_count < 3) {
		PX4_ERR("DWM_ANC_CNT = %d, need at least 3 anchors for a 2D fix", _anchor_count);
		return false;
	}

	ScheduleOnInterval(10_ms);
	return true;
}

void UWB_DWM3001::loadAnchors()
{
	const float xs[DWM_MAX_ANCHORS] = {
		_param_a0_x.get(), _param_a1_x.get(), _param_a2_x.get(),
		_param_a3_x.get(), _param_a4_x.get(), _param_a5_x.get()
	};
	const float ys[DWM_MAX_ANCHORS] = {
		_param_a0_y.get(), _param_a1_y.get(), _param_a2_y.get(),
		_param_a3_y.get(), _param_a4_y.get(), _param_a5_y.get()
	};
	const float zs[DWM_MAX_ANCHORS] = {
		_param_a0_z.get(), _param_a1_z.get(), _param_a2_z.get(),
		_param_a3_z.get(), _param_a4_z.get(), _param_a5_z.get()
	};

	int n = _param_anc_cnt.get();

	if (n > DWM_MAX_ANCHORS) { n = DWM_MAX_ANCHORS; }

	if (n < 0) { n = 0; }

	for (int i = 0; i < n; i++) {
		_anchor_x[i] = xs[i];
		_anchor_y[i] = ys[i];
		_anchor_z[i] = zs[i];
	}

	_anchor_count = n;
}

void UWB_DWM3001::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		loadAnchors();
	}
}

bool UWB_DWM3001::openPort()
{
	_uart = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart < 0) {
		PX4_ERR("open %s failed (%i)", _port, errno);
		return false;
	}

	struct termios uart_config;
	tcgetattr(_uart, &uart_config);

	// Raw mode. The module speaks line-delimited ASCII, so we must not let the
	// tty layer rewrite CR/LF or do any canonical processing.
	uart_config.c_oflag &= ~ONLCR;
	uart_config.c_iflag &= ~(ICRNL | INLCR | IGNCR | IXON | IXOFF);
	uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	if (cfsetispeed(&uart_config, DWM_BAUD) < 0 || cfsetospeed(&uart_config, DWM_BAUD) < 0) {
		PX4_ERR("failed to set baud");
		::close(_uart);
		_uart = -1;
		return false;
	}

	if (tcsetattr(_uart, TCSANOW, &uart_config) < 0) {
		PX4_ERR("tcsetattr failed");
		::close(_uart);
		_uart = -1;
		return false;
	}

	return true;
}

void UWB_DWM3001::startSession()
{
	// A board already mid-session rejects `initf`, so always stop first.
	const char *stop = "stop\r\n";
	(void)::write(_uart, stop, strlen(stop));

	// Command shape is verbatim from droneblocks-uwb ranging/live_map.py.
	// Anchor i is addressed as responder i+1.
	char cmd[128];
	int n = snprintf(cmd, sizeof(cmd),
			 "initf 4 2400 200 25 2 42 01:02:03:04:05:06:07:08 1 0 0");

	for (int i = 0; i < _anchor_count && n > 0 && n < (int)sizeof(cmd) - 8; i++) {
		n += snprintf(cmd + n, sizeof(cmd) - n, " %d", i + 1);
	}

	if (n > 0 && n < (int)sizeof(cmd) - 3) {
		n += snprintf(cmd + n, sizeof(cmd) - n, "\r\n");
		(void)::write(_uart, cmd, n);
		_session_started = true;
		PX4_INFO("ranging session started on %s, %d anchors", _port, _anchor_count);

	} else {
		PX4_ERR("initf command truncated");
	}
}

void UWB_DWM3001::readSerial()
{
	uint8_t buf[128];
	int nread = ::read(_uart, buf, sizeof(buf));

	if (nread <= 0) {
		return;
	}

	for (int i = 0; i < nread; i++) {
		const char c = (char)buf[i];

		if (c == '\n' || c == '\r') {
			if (_line_len > 0) {
				_line[_line_len] = '\0';

				dwm_block_s blk{};

				if (parseBlock(_line, blk)) {
					handleBlock(blk);
				}

				_line_len = 0;
			}

		} else if (_line_len < DWM_LINE_MAX - 1) {
			_line[_line_len++] = c;

		} else {
			// Overlong line: drop it rather than emit a half-parsed block.
			perf_count(_parse_err_perf);
			_line_len = 0;
		}
	}
}

bool UWB_DWM3001::parseBlock(const char *line, dwm_block_s &out) const
{
	if (line[0] != '{') {
		return false;
	}

	const char *results = strstr(line, "\"results\"");

	if (results == nullptr) {
		return false;
	}

	out.count = 0;
	out.block = 0;

	const char *b = strstr(line, "\"Block\"");

	if (b != nullptr) {
		const char *colon = strchr(b, ':');

		if (colon != nullptr) {
			out.block = (uint32_t)strtoul(colon + 1, nullptr, 10);
		}
	}

	const char *p = results;

	while ((p = strstr(p, "\"Addr\"")) != nullptr && out.count < DWM_MAX_ANCHORS) {
		const char *obj_end = strchr(p, '}');

		if (obj_end == nullptr) {
			break;
		}

		dwm_range_s r{};
		r.raw_cm = NAN;
		r.range_m = NAN;
		r.valid = false;

		// "Addr":"0x0001"  -- strtoul with base 0 consumes the 0x form and
		// stops at the closing quote.
		const char *colon = strchr(p, ':');

		if (colon != nullptr && colon < obj_end) {
			const char *quote = strchr(colon, '"');
			const char *num = (quote != nullptr && quote < obj_end) ? quote + 1 : colon + 1;
			r.addr = (uint16_t)strtoul(num, nullptr, 0);
		}

		const char *st = strstr(p, "\"Status\"");
		bool status_ok = false;

		if (st != nullptr && st < obj_end) {
			const char *okp = strstr(st, "\"Ok\"");
			status_ok = (okp != nullptr && okp < obj_end);
		}

		const char *d = strstr(p, "\"D_cm\"");

		if (d != nullptr && d < obj_end) {
			const char *dc = strchr(d, ':');

			if (dc != nullptr && dc < obj_end) {
				r.raw_cm = strtof(dc + 1, nullptr);
			}
		}

		r.valid = status_ok && PX4_ISFINITE(r.raw_cm);
		out.ranges[out.count++] = r;

		p = obj_end + 1;
	}

	return out.count > 0;
}

float UWB_DWM3001::heightAgl()
{
	vehicle_local_position_s lpos;

	if (_local_pos_sub.copy(&lpos) && lpos.dist_bottom_valid && PX4_ISFINITE(lpos.dist_bottom)) {
		return lpos.dist_bottom;
	}

	return NAN;
}

void UWB_DWM3001::handleBlock(dwm_block_s &blk)
{
	perf_count(_block_perf);
	_block_count++;
	_last_block = hrt_absolute_time();

	const float scale = _param_rng_scale.get();
	const float offset_cm = _param_rng_off.get();
	const bool deslant = (_param_deslant.get() != 0);
	const float agl = deslant ? heightAgl() : NAN;

	const hrt_abstime now = hrt_absolute_time();

	float ax[DWM_MAX_ANCHORS];
	float ay[DWM_MAX_ANCHORS];
	float rr[DWM_MAX_ANCHORS];
	int   nsolve = 0;

	for (int i = 0; i < blk.count; i++) {
		dwm_range_s &r = blk.ranges[i];

		// Anchor i is addressed as responder i+1.
		const int idx = (int)r.addr - 1;
		const bool known = (idx >= 0 && idx < _anchor_count);

		if (r.valid) {
			float slant = (scale * r.raw_cm + offset_cm) * 0.01f;

			// The 2D solver treats every range as a horizontal distance. It is
			// not: it is a slant range. With anchors above the vehicle the bias
			// is far larger than the radio noise, so de-slant when we have a
			// trustworthy height. r_h = sqrt(r_slant^2 - dz^2).
			if (deslant && known && PX4_ISFINITE(agl)) {
				const float dz = _anchor_z[idx] - agl;

				if (slant > fabsf(dz)) {
					slant = sqrtf(slant * slant - dz * dz);

				} else {
					// Geometrically impossible: the range is shorter than the
					// height difference. Drop it rather than feed a bad fix.
					r.valid = false;
				}
			}

			if (r.valid) {
				r.range_m = slant;

				if (known && nsolve < DWM_MAX_ANCHORS) {
					ax[nsolve] = _anchor_x[idx];
					ay[nsolve] = _anchor_y[idx];
					rr[nsolve] = slant;
					nsolve++;
				}
			}
		}

		sensor_uwb_s msg{};
		msg.timestamp = now;
		msg.counter = blk.block;
		msg.mac_dest = r.addr;
		msg.distance = r.valid ? r.range_m : NAN;
		msg.status = r.valid ? 0 : 1;
		msg.nlos = 0;
		msg.aoa_azimuth_dev = NAN;
		msg.aoa_elevation_dev = NAN;
		msg.aoa_azimuth_resp = NAN;
		msg.aoa_elevation_resp = NAN;
		_sensor_uwb_pub.publish(msg);
	}

	if (nsolve < 3) {
		return;
	}

	float x = NAN;
	float y = NAN;

	if (!solve2d(ax, ay, rr, nsolve, x, y)) {
		return;
	}

	// Rotate the survey frame into NED. DWM_YAW_OFF is the bearing of the
	// survey +X axis measured clockwise from True North, in degrees. With
	// DWM_YAW_OFF = 0 the survey frame IS NED: +X North, +Y East.
	const float yaw = math::radians(_param_yaw_off.get());
	const float cy = cosf(yaw);
	const float sy = sinf(yaw);
	const float north = x * cy - y * sy;
	const float east  = x * sy + y * cy;

	const float var = _param_pos_noise.get() * _param_pos_noise.get();

	vehicle_odometry_s odom{};
	odom.timestamp = hrt_absolute_time();
	odom.timestamp_sample = now;
	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;

	odom.position[0] = north;
	odom.position[1] = east;
	odom.position[2] = NAN;   // altitude comes from the rangefinder, not UWB

	odom.q[0] = NAN;          // no attitude from UWB

	odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_UNKNOWN;
	odom.velocity[0] = NAN;
	odom.velocity[1] = NAN;
	odom.velocity[2] = NAN;
	odom.angular_velocity[0] = NAN;
	odom.angular_velocity[1] = NAN;
	odom.angular_velocity[2] = NAN;

	odom.position_variance[0] = var;
	odom.position_variance[1] = var;
	odom.position_variance[2] = NAN;

	odom.orientation_variance[0] = NAN;
	odom.orientation_variance[1] = NAN;
	odom.orientation_variance[2] = NAN;
	odom.velocity_variance[0] = NAN;
	odom.velocity_variance[1] = NAN;
	odom.velocity_variance[2] = NAN;

	odom.quality = 0;

	_visual_odometry_pub.publish(odom);

	perf_count(_fix_perf);
	_fix_count++;
	_last_fix = odom.timestamp;
}

bool UWB_DWM3001::solve2d(const float ax[], const float ay[], const float r[],
			  int n, float &x, float &y)
{
	if (n < 3) {
		return false;
	}

	const float x0 = ax[0];
	const float y0 = ay[0];
	const float r0 = r[0];

	float Sxx = 0.f, Sxy = 0.f, Syy = 0.f, Sxb = 0.f, Syb = 0.f;

	for (int i = 1; i < n; i++) {
		const float Ax = 2.f * (ax[i] - x0);
		const float Ay = 2.f * (ay[i] - y0);
		const float b = (r0 * r0 - r[i] * r[i])
				+ (ax[i] * ax[i] - x0 * x0)
				+ (ay[i] * ay[i] - y0 * y0);

		Sxx += Ax * Ax;
		Sxy += Ax * Ay;
		Syy += Ay * Ay;
		Sxb += Ax * b;
		Syb += Ay * b;
	}

	const float det = Sxx * Syy - Sxy * Sxy;

	if (fabsf(det) < DWM_MIN_DET) {
		return false;   // anchors collinear, no unique fix
	}

	x = (Syy * Sxb - Sxy * Syb) / det;
	y = (Sxx * Syb - Sxy * Sxb) / det;

	return PX4_ISFINITE(x) && PX4_ISFINITE(y);
}

void UWB_DWM3001::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	if (_uart < 0) {
		if (!openPort()) {
			return;
		}

		_session_started = false;
		_line_len = 0;
	}

	if (!_session_started) {
		startSession();
		return;
	}

	readSerial();

	// If the module has gone quiet, the session probably died (board wedged, or
	// power-cycled). Restart it rather than sit silent.
	if (_last_block != 0 && hrt_elapsed_time(&_last_block) > 5_s) {
		PX4_WARN("no ranging blocks for 5s, restarting session");
		_session_started = false;
		_last_block = 0;
	}
}

int UWB_DWM3001::print_status()
{
	PX4_INFO("port: %s (fd %d)", _port, _uart);
	PX4_INFO("anchors: %d", _anchor_count);

	for (int i = 0; i < _anchor_count; i++) {
		PX4_INFO("  A%d (addr %d): x %.3f  y %.3f  z %.3f",
			 i, i + 1, (double)_anchor_x[i], (double)_anchor_y[i], (double)_anchor_z[i]);
	}

	PX4_INFO("calibration: true_cm = %.4f * D_cm + %.2f",
		 (double)_param_rng_scale.get(), (double)_param_rng_off.get());
	PX4_INFO("de-slant: %s", _param_deslant.get() ? "on" : "off");
	PX4_INFO("survey yaw offset: %.1f deg", (double)_param_yaw_off.get());
	PX4_INFO("blocks: %lu   fixes: %lu",
		 (unsigned long)_block_count, (unsigned long)_fix_count);

	if (_last_fix != 0) {
		PX4_INFO("last fix: %.2f s ago", (double)hrt_elapsed_time(&_last_fix) * 1e-6);

	} else {
		PX4_INFO("last fix: never");
	}

	perf_print_counter(_block_perf);
	perf_print_counter(_fix_perf);
	perf_print_counter(_parse_err_perf);

	return 0;
}

int UWB_DWM3001::task_spawn(int argc, char *argv[])
{
	int ch;
	int option_index = 1;
	const char *option_arg;
	const char *device_name = UWB_DWM3001_DEFAULT_PORT;

	while ((ch = px4_getopt(argc, argv, "d:", &option_index, &option_arg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = option_arg;
			break;

		default:
			PX4_WARN("unrecognized flag: %c", ch);
			break;
		}
	}

	UWB_DWM3001 *instance = new UWB_DWM3001(device_name);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init()) {
		return PX4_OK;
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int UWB_DWM3001::custom_command(int argc, char *argv[])
{
	return print_usage("unrecognized command");
}

int UWB_DWM3001::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver for a Qorvo DWM3001C acting as a UWB tag (FiRa two-way-ranging initiator),
speaking the DW3_QM33 SDK CLI over serial.

Drives the ranging session, parses each JSON block, applies the linear
antenna-delay correction, de-slants the ranges using height above ground, then
trilaterates and publishes `vehicle_visual_odometry` for EKF2.

Raw per-anchor ranges are published on `sensor_uwb`. Add `sensor_uwb` to
`logged_topics.cpp` if you want them in the ulog.

UWB supplies horizontal position only. Set `EKF2_EV_CTRL` to 1 (bit 0). Altitude
must come from the rangefinder. Sensor lever arm is handled by
`EKF2_EV_POS_X/Y/Z`, not by this driver.

Anchor i is addressed as responder i+1 and is surveyed with `DWM_Ai_X/Y/Z`.

### Examples

Start on TELEM3:
$ uwb_dwm3001 start -d /dev/ttyS3
$ uwb_dwm3001 status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uwb_dwm3001", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', UWB_DWM3001_DEFAULT_PORT, "<file:dev>", "Serial device", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int uwb_dwm3001_main(int argc, char *argv[])
{
	return UWB_DWM3001::main(argc, argv);
}
