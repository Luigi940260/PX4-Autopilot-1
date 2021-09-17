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
  * @file custom_class.cpp
  *
  *
  * @author Luigi Chiocci
  */

#include "custom_class.h"

bool custom_class::running = false;

float custom_class::main_setp[3] = {0.0f, 0.0f, 0.0f};

float custom_class::initial_cond[3] = {0.0f, 0.0f, 0.0f};

bool custom_class::update[3] = {false, false, false};

int custom_class::main()
{
	running = true;

	PX4_INFO("Setpoint Generator Started\n");

	uORB::Publication<setpoint_s>	_setpoint_pub{ ORB_ID(setpoint) };
 
  //Filter Coefficient Creation
  float a[3];
  float b[3];
  float ref[3];
  
  for (int i = 0; i < 3; i++)
    if (update[i]){
      if (abs(main_setp[i]) < 0.01f){
        a[i] = 1.0f;
        b[i] = 0.0f;
        ref[i] = -initial_cond[i];
      }
      else {
        a[i] = (main_setp[i] - initial_cond[i])/main_setp[i];
        b[i] = 1/a[i];
        ref[i] = main_setp[i];
      }
    }
    else {
      ref[i] = initial_cond[i];
    }
  float output[3];

	//Loop variable creation
	uint64_t last_time = hrt_absolute_time();

	float time = 0.0f;

  while (running) {

    //Time Reading
    uint64_t now = hrt_absolute_time();
		if (now - last_time > Ts) {
		  last_time = now;

      //Time update
			time = time + Ts_sec;

      //Filter Application
      Filter(a, b, time, ref, output);
      
      // Check for the setpoint reached
      bool test = true;
      for (int i = 0; i < 3; i++)
        if (abs(main_setp[i] - output[i]) <= delta)
          output[i] = main_setp[i];
        else
          test = false;

      //setpoint publications
      setpoint_s setp{};
      for (int i = 0; i < 3; i++)
        setp.xyz[i] = output[i];
        
      _setpoint_pub.publish(setp);
      
      // Check End-Of-Loop
      if (test)
        running = false;
    }

  }
  //Update Position    
  for (int i = 0; i < 3; i++){
    initial_cond[i] = output[i];
    update[i] = false;
  }

	return 0;

}

/*

a(b - 1)*i = in
abi = i

-> a = 1/b
-> 1 - a = in/i

-> a = (i-in)/i

-> b = 1/a

*/

void custom_class::Filter (float* a, float* b, float t, float* input, float* output) {
  
  float tau = Tau/(float)7;
  
	//Filter Dynamic
	for (int i = 0; i < 3; i++)
    if (update[i])
		  output[i] = a[i]*(b[i] - exp(-t / tau)) * input[i];
    else
      output[i] = input[i];
}

void custom_class::circle(float radius, float omega, float* center){
  uORB::Publication<setpoint_s>	_setpoint_pub{ ORB_ID(setpoint) };
  
  setpoint_s setp{};

  //Loop variable creation
	uint64_t last_time = hrt_absolute_time();
 
  float time = 0.0f;

  while (running) {

    //Time Reading
    uint64_t now = hrt_absolute_time();
		if (now - last_time > Ts) {
		  last_time = now;

      //Time update
			time = time + Ts_sec;
      
      //Performing the circle
    
      double angle = (double)omega*(double)time;
    
      setp.xyz[0] = center[0] + radius*(float)cos(angle);
      setp.xyz[1] = center[1] + radius*(float)sin(angle);
      setp.xyz[2] = custom_class::initial_cond[2];
      
      _setpoint_pub.publish(setp);
      
      if (angle > 6.28)
        running = false;
      
    }
  }
  
    //Update Position    
  for (int i = 0; i < 3; i++){
    initial_cond[i] = setp.xyz[i];
    update[i] = false;
  }
    
    
}
