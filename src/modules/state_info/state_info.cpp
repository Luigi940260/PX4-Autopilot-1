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
  * @file state_info.cpp
  *
  *
  * @author Luigi Chiocci
  */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <poll.h>
#include <string.h>
#include <math.h>

  // uORB includes

#include <uORB/uORB.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>

extern "C" __EXPORT int state_info_main(int argn, char* argv[]);

int state_info_main(int argn, char* argv[]) {
	PX4_INFO("State_info has started\n");
	uint64_t Ts = 2000000; //in microiseconds
  int max = 10;
	if (argn > 1)
		max = atoi(argv[1]);
	 

	//subscribing linear position and linear velocity
	int local_position_h = orb_subscribe(ORB_ID(vehicle_local_position));

	//subscribing attitude
	int attitude_h = orb_subscribe(ORB_ID(vehicle_attitude));

	//subscribing angular velocity
	int angular_velocity_h = orb_subscribe(ORB_ID(vehicle_angular_velocity));

	//attitude (rpy) struct
	struct vehicle_attitude_rpy {
		double phi;
		double theta;
		double psi;
	};

	//angular velocity (rpy) struct
	struct vehicle_angular_velocity_rpy {
		double phi_dot;
		double theta_dot;
		double psi_dot;
	};

	int i = 0;
  uint64_t last = hrt_absolute_time();
    
  while (i < max) {
  
  uint64_t now = hrt_absolute_time(); 
  
  if (now - last > Ts) {
    last = now;
    i++;
    
		//position and velocity struct
		struct vehicle_local_position_s local_position {};
		//attitude (quat) struct
		struct vehicle_attitude_s attitude_quat {};
		//attitude (rpy) struct
		vehicle_attitude_rpy attitude_rpy{};
		//angular velocity (omega) struct
		struct vehicle_angular_velocity_s angular_velocity_xyz{};
		//angualr velocity (rpy) struct
		vehicle_angular_velocity_rpy angular_velocity_rpy{};
		//Get the value
		orb_copy(ORB_ID(vehicle_local_position), local_position_h, &local_position);
		orb_copy(ORB_ID(vehicle_attitude), attitude_h, &attitude_quat);
		orb_copy(ORB_ID(vehicle_angular_velocity), angular_velocity_h, &angular_velocity_xyz);

		// simplify the terms
		double q0 = (double)attitude_quat.q[0];
		double q1 = (double)attitude_quat.q[1];
		double q2 = (double)attitude_quat.q[2];
		double q3 = (double)attitude_quat.q[3];

		double wx = (double)angular_velocity_xyz.xyz[0];
		double wy = (double)angular_velocity_xyz.xyz[1];
		double wz = (double)angular_velocity_xyz.xyz[2];
		//From quaternion to rpy
		double phi = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2));
		double psi = atan2(2.0 * (q1 * q2 + q0 * q3), 1.0 - 2.0 * (q2 * q2 + q3 * q3));
		double theta = atan2(2.0 * (q0 * q2 - q1 * q3)*sin(attitude_rpy.psi), 2.0 * (q1 * q2 + q0 * q3));

		attitude_rpy.phi = phi;
		attitude_rpy.psi = psi;
		attitude_rpy.theta = theta;
		//From Omega_xyz to RPY_dot
		angular_velocity_rpy.phi_dot = cos(psi) * wx / cos(theta) + sin(phi) * cos(theta) * wy;
		angular_velocity_rpy.theta_dot = cos(psi) * wy - sin(phi) * wx;
		angular_velocity_rpy.psi_dot = cos(psi) * tan(theta) * wx + sin(psi) * tan(theta) * wy + wz;

    //Print the value
		PX4_INFO("Position:\tx=%f\ty=%f\tz=%f\n", (double)local_position.x, (double)local_position.y, (double)local_position.z);
		PX4_INFO("Velocity:\tvx=%f\tvy=%f\tvz=%f\n", (double)local_position.vx, (double)local_position.vy, (double)local_position.vz);
		PX4_INFO("Attitude:\tphi=%f\ttheta=%f\tpsi=%f\n", attitude_rpy.phi, attitude_rpy.theta, attitude_rpy.psi);
		PX4_INFO("Angular Velocity:\tvx=%f\tvy=%f\tvz=%f\n", angular_velocity_rpy.phi_dot, angular_velocity_rpy.theta_dot, angular_velocity_rpy.psi_dot);
		}

	}

	PX4_INFO("Exit from state_info");

	return 0;
}