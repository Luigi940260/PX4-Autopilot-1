  /****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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
  * @file Custom_Controller.hpp
  *
  * State_Feedback + Error_Integral_Action Controller
  */

#pragma once
#include <matrix/matrix/math.hpp>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/setpoint.h>
#include <uORB/topics/state.h>


class Custom_Controller {

private:
	// Gain Matrix for State Feedback
	const float K_S[4][12] = {
		{   -0.0000f,   -0.0000f,  -2.5582f,   -0.0000f,   -0.0000f,   -2.7721f,   -0.0000f,    0.0000f,    -0.0000f,   -0.0000f,  0.0000f,  -0.0000f},
		{   0.0000f,     0.5864f,    -0.0000f,   0.0000f,   0.3855f,   -0.0000f,     1.3891f,   -0.0000f,   0.0000f,   0.2862f, -0.0000f, 0.0000f},
		{   -0.5864f,    -0.0000f,    -0.0000f,   -0.3855f,   -0.0000f,  -0.0000f,    -0.0000f,     1.3891f,    -0.0000f,    -0.0000f,   0.2862f,  -0.0000f},
    {   -0.0000f,    -0.0000f,   -0.0000f,    -0.0000f,    -0.0000f,   -0.0000f,    -0.0000f,   0.0000f,    0.3162f,    -0.0000f, -0.0000f,    0.1895f}
	};
	//Gain Matrix for Integral Action on Error

	const float K_E[4][3] = {
		{ 0.0000f,   -0.0000f,  -1.000f},
		{-0.0000f,     0.3162f,    0.0000f},
		{-0.3162f,    0.0000f,   -0.0000f},
    {0.0000f,     0.0000f,   0.000f}
	};

	// State Matrix for Integral Action on Error

	const float A[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
	const float B[3][3] = { {0.001f, 0, 0}, {0, 0.001f, 0}, {0, 0, 0.001f} };
	const float C[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
	const float D[3][3] = { {0.00050f, 0, 0}, {0, 0.00050f, 0}, {0, 0, 0.00050f} };

	//Initial Condition for the Error
	const float e0[3] = { 0.0000f, 0.0000f, 14.7286f};
  
public:
	const uint64_t Ts = 1000;
	uint64_t last_time = 0;
	const float Max_Trust = 26.0442;
  float last_setp[3] = {0.0f, 0.0f, 0.0f};
   
	/**
	 * Calculate the integral action on the error
	 * @param  error  pointer to actual error 3_D array 
	 * @param output  pointer to output variable 4_D array
	 */
	void error_integral_action(float* error, float* output);

	/**
	 * Calculate the state feedback action
	 * @param State   pointer to actual state 12_D array
	 * @param output  pointer to output variable 4_D array
	 */
	void state_feedback(float* state, float* output);

	/**
	 * Converts the quaternion/omega rappresentation in Roll-Pitch-Yaw Coordinates
	 * @param quat		pointer to quaternion 3_D array
	 * @param W			pointer to angular velocity 3_D array
	 * @param rpy		pointer to output RPY 3_D array
	 * @param rpy_dot	pointer to output RPY_dot 3_D array
	 */
	void rpy_conversion(double* quat, float* W, float* rpy, float* rpy_dot);

};
