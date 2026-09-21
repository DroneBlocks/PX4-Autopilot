/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Maximum upwards acceleration in climb rate controlled modes
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_UP_MAX, 4.f);

/**
 * Maximum downwards acceleration in climb rate controlled modes
 *
 * @unit m/s^2
 * @min 2
 * @max 15
 * @decimal 1
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_ACC_DOWN_MAX, 3.f);

/**
 * Manual yaw rate input filter time constant
 *
 * Not used in Stabilized mode
 * Setting this parameter to 0 disables the filter
 *
 * @unit s
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_MAN_Y_TAU, 0.08f);

/**
 * Altitude reference mode
 *
 * Control height
 * 0: relative earth frame origin which may drift due to sensors
 * 1: relative to ground (requires distance sensor) which changes with terrain variation.
 * It will revert to relative earth frame if the distance to ground estimate becomes invalid.
 * 2: relative to ground (requires distance sensor) when stationary
 * and relative to earth frame when moving horizontally.
 * The speed threshold is MPC_HOLD_MAX_XY
 *
 * @min 0
 * @max 2
 * @value 0 Altitude following
 * @value 1 Terrain following
 * @value 2 Terrain hold
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_ALT_MODE, 2);

/**
 * Maximum horizontal velocity for which position hold is enabled (use 0 to disable check)
 *
 * Only used with MPC_POS_MODE Direct velocity or MPC_ALT_MODE 2
 *
 * @unit m/s
 * @min 0
 * @max 3
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_XY, 0.8f);

/**
 * Maximum vertical velocity for which position hold is enabled (use 0 to disable check)
 *
 * Only used with MPC_ALT_MODE 1
 *
 * @unit m/s
 * @min 0
 * @max 3
 * @decimal 2
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HOLD_MAX_Z, 0.6f);

/**
 * Maximum height above the floor in Altitude and Position modes
 *
 * Limits how far the vehicle climbs above whatever the downward range finder is
 * pointing at, for flying indoors under a known roof height. Enforced as a
 * proportional slowdown, exactly the way the range finder's own limit is, so the
 * vehicle eases to a stop instead of stopping dead.
 *
 * Requires a working downward range finder: it has no effect when the distance to
 * the ground is unknown. It does not apply to auto modes -- use MIS_TAKEOFF_ALT
 * to cap an auto takeoff.
 *
 * WARNING: this limits height above what is BELOW the vehicle, not height below
 * the ceiling. Flying over furniture lowers the measured distance, so the vehicle
 * believes it has regained headroom and climbs closer to the real roof. Treat it
 * as a convenience limit, not as protection against hitting the ceiling.
 *
 * Disabled if 0.
 *
 * @unit m
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_HAGL_MAX, 0.0f);
