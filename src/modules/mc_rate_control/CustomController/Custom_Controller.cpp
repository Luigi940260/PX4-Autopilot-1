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
  * @file Custom_Controller.cpp
  *
  * State_Feedback + Error_Integral_Action Controller
  */

#include "Custom_Controller.hpp"

void Custom_Controller::error_integral_action(float* error, float* output) {
  static bool first_call = true;
  
	static float Error_State[3] = {0.0f, 0.0f, 0.0f};
   
   if (first_call){
     for (int i = 0; i < 3; i++)
       Error_State[i] = e0[i];
   first_call = false;
   }
   
	// Error State Dynamic
	float Future_State[3] = {0.0f, 0.0f, 0.0f};
	for (int i = 0; i < 3; i++) {
		float sum = 0; //support variable
		for (int j = 0; j < 3; j++)
			sum += A[i][j] * Error_State[j] + B[i][j] * error[j];
		Future_State[i] = sum;
	}

	// Integral Action Output
	float Integral_Action[3] = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < 3; i++) {
		float sum = 0; //support variable
		for (int j = 0; j < 3; j++)
			sum += C[i][j] * Error_State[j] + D[i][j] * error[j];
		Integral_Action[i] = sum;
	}

	//State Update
	for (int i = 0; i < 3; i++)
		Error_State[i] = Future_State[i];

	// Error Feedback Gain
	for (int i = 0; i < 4; i++) {
		float sum = 0; //support variable
		for (int j = 0; j < 3; j++)
			sum += - K_E[i][j] * Integral_Action[j];
		output[i] = sum;
	}
}

void Custom_Controller::state_feedback(float* state, float* output) {
	for (int i = 0; i < 4; i++) {
		float sum = 0; //Support variable
		for (int j = 0; j < 12; j++)
			sum += - K_S[i][j] * state[j];
		output[i] = sum;
	}
}

void Custom_Controller::rpy_conversion(double* quat, float* W, float* rpy, float* rpy_dot) {
	//RPY Conversion
	float phi = (float)atan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
	float psi = (float)atan2(2.0 * (quat[1] * quat[2] + quat[0] * quat[3]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
  float theta;
  float sinth = 2 * (quat[0] * quat[2] - quat[3] * quat[1]);
  if (std::abs(sinth) >= 1)
    theta = std::copysign(M_PI / 2, sinth); // use 90 degrees if out of range
  else
    theta = std::asin(sinth);

	float phi_dot = cos(psi) * W[0] / cos(theta) + sin(phi) * cos(theta) * W[1];
	float theta_dot = cos(psi) * W[1] - sin(phi) * W[0];
	float psi_dot = cos(psi) * tan(theta) * W[0] + sin(psi) * tan(theta) * W[1] + W[2];

	rpy[0] = phi;
	rpy[1] = theta;
	rpy[2] = psi;

	rpy_dot[0] = phi_dot;
	rpy_dot[1] = theta_dot;
	rpy_dot[2] = psi_dot;

}
