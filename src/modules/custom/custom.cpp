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
  * @file custom.cpp
  *
  *
  * @author Luigi Chiocci
  */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/app.h>
#include <px4_platform_common/init.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>

#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/setpoint.h>
#include <uORB/topics/state.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <drivers/drv_hrt.h>

#include "custom_class.h"


int PX4_MAIN (int argn, char* argv[]) {
	
	custom_class custom;
 
  for (int i = 0; i < 3; i++)
    custom_class::update[i] = false;

	// Input Reading
	if (!strcmp(argv[1], "takeoff")) {

		//Simple takeoff. Default is 10 meters.
		if (argn == 3)
			custom_class::main_setp[2] = -strtof(argv[2], NULL);
		else
			custom_class::main_setp[2] = -10.0f;
    custom_class::update[2] = true;
   
    custom.main();
	}//custom setpoint by user
	else if (!strcmp(argv[1], "setpoint")) { 
		if (argn != 5) {
			PX4_ERR("Wrong use\n");
			return 1;
		}
		for (int i = 0; i < 3; i++){
			custom_class::main_setp[i] = strtof(argv[i + 2], NULL);
      if (abs(custom_class::main_setp[i] - custom_class::initial_cond[i]) > 0.5f)
        custom_class::update[i] = true;
   }
      
    custom.main();
	}// Perform a circle of a given radius
  else if (!strcmp(argv[1], "circle")) {
    if (argn != 3) {
      PX4_ERR("Wrong use\n");
			return 1;
    }
    //Reading Radius
    float r = strtof(argv[2], NULL);
    //Reading center
    float center[2];
    center[0] = custom_class::main_setp[0];
    center[1] = custom_class::main_setp[1]; 
    
    //Reaching Initial Position
    custom_class::main_setp[0] = center[0] + r;
    custom_class::update[0] = true;
    custom.main();
    
    //Performing the circle
    for (double angle = 0.0; angle < 6.28; angle += 0.1){
      custom_class::main_setp[0] = center[0] + r*(float)cos(angle);
      custom_class::main_setp[1] = center[1] + r*(float)sin(angle);
      custom_class::update[0] = true;
      custom_class::update[1] = true;
      custom.main();
    }
    
    //Return to Initial State
    custom_class::main_setp[0] = center[0];
    custom_class::main_setp[1] = center[1];
    custom_class::update[0] = true;
    custom_class::update[1] = true;
    custom.main();
    
  } else {
		PX4_ERR("Wrong use\n");
		return 1;
	}

  PX4_INFO("Custom Generator Terminated\n");

	return 0;
}