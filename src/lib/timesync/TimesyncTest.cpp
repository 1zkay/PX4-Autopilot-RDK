/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>

#include "Timesync.hpp"

namespace
{
constexpr int64_t NS_PER_US = 1000;

void update_timesync(Timesync &timesync, int64_t client_transmit_us, int64_t offset_us,
		     int64_t forward_delay_us, int64_t agent_processing_us, int64_t reverse_delay_us)
{
	const int64_t agent_receive_us = client_transmit_us + forward_delay_us - offset_us;
	const int64_t agent_transmit_us = agent_receive_us + agent_processing_us;
	const int64_t client_receive_us = client_transmit_us + forward_delay_us + agent_processing_us + reverse_delay_us;

	timesync.update(client_transmit_us * NS_PER_US,
			agent_receive_us * NS_PER_US,
			agent_transmit_us * NS_PER_US,
			client_receive_us * NS_PER_US);
}
}

TEST(Timesync, UsesAllFourTimestamps)
{
	Timesync timesync;
	constexpr int64_t clock_offset_us = -1'000'000;

	// Agent processing exceeds MAX_RTT_SAMPLE, but network RTT does not.
	update_timesync(timesync, 1'000'000, clock_offset_us, 2'000, 25'000, 2'000);

	EXPECT_EQ(timesync.offset(), clock_offset_us);
}

TEST(Timesync, PreservesMavlinkThreeTimestampExchange)
{
	Timesync timesync;
	constexpr int64_t client_transmit_us = 1'000'000;
	constexpr int64_t remote_timestamp_us = 2'002'000;
	constexpr int64_t client_receive_us = 1'004'000;

	timesync.update(client_receive_us, remote_timestamp_us * NS_PER_US,
			client_transmit_us * NS_PER_US);

	EXPECT_EQ(timesync.offset(), -1'000'000);
}

TEST(Timesync, TracksClockSkewAcrossSampleRateChange)
{
	Timesync timesync;
	constexpr int64_t initial_client_time_us = 1'000'000;
	constexpr int64_t initial_offset_us = -1'000'000'000;
	constexpr double clock_skew_us_per_s = 2'400.;
	int64_t client_transmit_us = initial_client_time_us;

	auto offset_at = [&](int64_t timestamp_us) {
		return initial_offset_us + static_cast<int64_t>(clock_skew_us_per_s
				* static_cast<double>(timestamp_us - initial_client_time_us) / 1e6);
	};

	// Startup synchronization runs at approximately 100 Hz until 500 samples converge.
	for (int i = 0; i < 500; ++i) {
		const int64_t offset_us = offset_at(client_transmit_us);
		update_timesync(timesync, client_transmit_us, offset_us, 1'000, 100, 1'000);
		client_transmit_us += 10'000;
	}

	ASSERT_TRUE(timesync.sync_converged());

	// Runtime synchronization runs at 1 Hz. The skew estimate must retain units of us/s.
	for (int i = 0; i < 70; ++i) {
		const int64_t offset_us = offset_at(client_transmit_us);
		update_timesync(timesync, client_transmit_us, offset_us, 1'000, 100, 1'000);
		EXPECT_TRUE(timesync.sync_converged());
		EXPECT_NEAR(timesync.offset(), offset_us, 5'000);
		client_transmit_us += 1'000'000;
	}
}

TEST(Timesync, PredictsOffsetAtMessageTime)
{
	Timesync timesync;
	constexpr int64_t initial_client_time_us = 1'000'000;
	constexpr int64_t initial_offset_us = -1'000'000'000;
	constexpr double clock_skew_us_per_s = 2'400.;
	int64_t client_transmit_us = initial_client_time_us;

	auto offset_at = [&](int64_t timestamp_us) {
		return initial_offset_us + static_cast<int64_t>(clock_skew_us_per_s
				* static_cast<double>(timestamp_us - initial_client_time_us) / 1e6);
	};

	for (int i = 0; i < 500; ++i) {
		update_timesync(timesync, client_transmit_us, offset_at(client_transmit_us), 1'000, 100, 1'000);
		client_transmit_us += 10'000;
	}

	for (int i = 0; i < 70; ++i) {
		update_timesync(timesync, client_transmit_us, offset_at(client_transmit_us), 1'000, 100, 1'000);
		client_transmit_us += 1'000'000;
	}

	ASSERT_TRUE(timesync.sync_converged());
	EXPECT_GT(timesync.offset_at(client_transmit_us) - timesync.offset(), 1'000);
	EXPECT_NEAR(timesync.offset_at(client_transmit_us), offset_at(client_transmit_us), 5'000);
}
