/****************************************************************************
 *
 *   Copyright (C) 2015 Mark Charlebois. All rights reserved.
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
  * @file px4_first_app.h
  * Example app for Linux
  *
  * @author Luigi Chiocci <gigi99.lc@gmail.com>
  */

#include "px4_first_app.h"
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/app.h>
#include <stdio.h>
#include <sched.h>
#include <stdlib.h>
#include <string.h>

static int task;

extern "C" __EXPORT int px4_first_app_main(int argc, char* argv[]);


int px4_first_app_main(int argc, char* argv[]) {
	if (argc < 2) {
		PX4_WARN("Wrong! The usage is: first_app {start|stop|increment|status} {quantity|Nothing}\n");
		return 1;
	}

	if (!strcmp(argv[1],"start")) {
		if (px4_first_app::appState.isRunning()) {
			PX4_INFO("Already Running");
			return 0;
		}

		task = px4_task_spawn_cmd("first_app",
			SCHED_DEFAULT,
			SCHED_PRIORITY_MAX - 5,
			2000,
			PX4_MAIN,
			(char* const*)NULL);

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		px4_first_app::appState.requestExit();
    px4_first_app::appState.setRunning(false);
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (px4_first_app::appState.isRunning())
			PX4_INFO("Is Running\n");
		else
			PX4_INFO("Not Started\n");
      
    PX4_INFO("Increment = %d", px4_first_app::getCounter());
		return 0;
	}

	if (!strcmp(argv[1], "increment")) {
		if (argv[2] == (char* )NULL) {
			PX4_WARN("Wrong! The usage is: first_app {start|stop|increment|status} {quantity|Nothing}\n");
			return 1;
		}
		int increment = atoi(argv[2]);
		px4_first_app::setCounter(increment);

		return 0;
	}
	PX4_WARN("Wrong! The usage is: first_app {start|stop|increment|status} {quantity|Nothing}\n");
	return 1;

}