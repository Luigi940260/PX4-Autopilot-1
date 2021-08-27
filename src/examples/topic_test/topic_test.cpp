/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
  * @file topic_test.cpp
  *
  *
  * @author Luigi Chiocci 
  */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <stdio.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>
#include <math.h>

// uORB includes

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>

extern "C" __EXPORT int topic_test_main(int argc, char* argv[]);

int topic_test_main(int argn, char* argv[]) {
	PX4_INFO("Topic_test has started\n");
	unsigned int Ts = 100; //in milliseconds

	int local_position_h = orb_subscribe(ORB_ID(vehicle_local_position));
	orb_set_interval(local_position_h, Ts);

	px4_pollfd_struct_t pfd[1] = { {.fd = local_position_h, .events = POLLIN} };

	for (int i = 0; i < 10; i++) {
		int poll_status = px4_poll(pfd, 1, 1000); //Wait for 1 second

		if (poll_status == 0)
			PX4_ERR("No data within a seconds");
		else if (poll_status < 0)
			PX4_ERR("Error!");
		else {
			if (pfd[0].revents & POLLIN) {
				struct vehicle_local_position_s local_position {};
				//Get the value
				orb_copy(ORB_ID(vehicle_local_position), local_position_h, &local_position);
				//Print the value
				PX4_INFO("Position:\tx=%f\ty=%f\tz=%f\n", (double)local_position.x, (double)local_position.y, (double)local_position.z);
				PX4_INFO("Position:\tvx=%f\tvy=%f\tvz=%f\n", (double)local_position.vx, (double)local_position.vy, (double)local_position.vz);
				PX4_INFO("Position:\tax=%f\tay=%f\taz=%f\n", (double)local_position.ax, (double)local_position.ay, (double)local_position.az);

			}
		}

	}

	PX4_INFO("Exit from topic test");

	return 0;
}