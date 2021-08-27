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
  * @file custom.cpp
  *
  *
  * @author Luigi Chiocci
  */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/tasks.h>

#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/setpoint.h>
#include <uORB/topics/state.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h> 

#include "custom_class.h"

 
 extern "C" __EXPORT int custom_main(int argn, char* argv[]);
 
 static int _custom_task; 
 
 int custom_main(int argn, char* argv[]) {
	 
	if (argn < 2) {
		PX4_ERR("Wrong use\n");
		return 1;
	}
 
 if (!strcmp(argv[1], "stop")){
   if (custom_class::running)
     custom_class::running = false;
   else
     PX4_WARN("No task running\n");
   return 0;
 }else if (!strcmp(argv[1], "help")) {
   PX4_INFO("Usage: custom -\n");
   PX4_INFO("takeoff altitude(optional) = perform a takeoff of -altitude- is specified. Default is 10 Mt\n");
   PX4_INFO("setpoint X Y Z = go to the point (X, Y, Z)\n");
   PX4_INFO("circle radius = perform a circle of -radius- at the actual altitude\n");
   PX4_INFO("stop = close the current custom task. Mandaotory for starting a new path\n");   
 }else {
     if (custom_class::running) {
     PX4_WARN("Already running\n");
     }
     else
      _custom_task = px4_task_spawn_cmd("custom",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_MAX - 5,
						 5000,
						 PX4_MAIN,
						 argv);
 }
  
   
 return 0;
 }