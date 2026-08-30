/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#ifndef OPEN_DRONE_ID_SELF_ID_HPP
#define OPEN_DRONE_ID_SELF_ID_HPP

#include <uORB/topics/open_drone_id_self_id.h>

/*
 * Re-broadcast the SELF_ID a ground station supplied.
 * PX4 caches it into uORB but, before this stream existed, only the DroneCAN
 * driver read it back out, so a serial Remote ID module never saw it and could
 * never report GOOD_TO_ARM.
 */
class MavlinkStreamOpenDroneIdSelfId : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamOpenDroneIdSelfId(mavlink); }

	static constexpr const char *get_name_static() { return "OPEN_DRONE_ID_SELF_ID"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		if (_sub.advertised()) {
			return MAVLINK_MSG_ID_OPEN_DRONE_ID_SELF_ID_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
		}

		return 0;
	}

private:
	explicit MavlinkStreamOpenDroneIdSelfId(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _sub{ORB_ID(open_drone_id_self_id)};

	bool send() override
	{
		open_drone_id_self_id_s data;

		// copy(), not update(). The receiving module ages this data out after 22 s,
		// so it has to be re-sent on every tick, not only when the GCS changes it.
		if (_sub.copy(&data)) {
			mavlink_open_drone_id_self_id_t msg{};

			msg.target_system = 0; // 0 for broadcast
			msg.target_component = 0; // 0 for broadcast
			static_assert(sizeof(msg.id_or_mac) == sizeof(data.id_or_mac), "id_or_mac size mismatch");
			memcpy(msg.id_or_mac, data.id_or_mac, sizeof(msg.id_or_mac));
			msg.description_type = data.description_type;
			static_assert(sizeof(msg.description) == sizeof(data.description), "description size mismatch");
			memcpy(msg.description, data.description, sizeof(msg.description));

			mavlink_msg_open_drone_id_self_id_send_struct(_mavlink->get_channel(), &msg);

			return true;
		}

		return false;
	}
};

#endif // OPEN_DRONE_ID_SELF_ID_HPP
