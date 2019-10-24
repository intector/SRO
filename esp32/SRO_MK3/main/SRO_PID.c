/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * SRO PID controller
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
 */

#include "SRO_PID.h"

SRO_ParSet SolParSet;
SRO_PID_Data PID_Data;

void Compute()
{
	if (!PID_Data.inAuto) return;

	/*Compute all the working error variables*/
	double error = PID_Data.Setpoint - PID_Data.Input;
	PID_Data.ITerm += (PID_Data.Ki * error);
	if (PID_Data.ITerm > PID_Data.outMax) PID_Data.ITerm = PID_Data.outMax;
	else if (PID_Data.ITerm < PID_Data.outMin) PID_Data.ITerm = PID_Data.outMin;
	double dInput = (PID_Data.Input - PID_Data.lastInput);
 
	/*Compute PID Output*/
	PID_Data.Output = PID_Data.Kp * error + PID_Data.ITerm - PID_Data.Kd * dInput;
	if (PID_Data.Output > PID_Data.outMax) PID_Data.Output = PID_Data.outMax;
	else if (PID_Data.Output < PID_Data.outMin) PID_Data.Output = PID_Data.outMin;
 
	/*Remember some variables for next time*/
	PID_Data.lastInput = PID_Data.Input;

	// get actual runtime in nano seconds
	clock_gettime(CLOCK_MONOTONIC, &PID_Data.actTime);
	PID_Data.Runtime = BILLION * (PID_Data.lastTime.tv_sec - PID_Data.actTime.tv_sec) + PID_Data.lastTime.tv_nsec - PID_Data.actTime.tv_nsec;

	PID_Data.lastTime.tv_sec = PID_Data.actTime.tv_sec;
	PID_Data.lastTime.tv_nsec = PID_Data.actTime.tv_nsec;
}
 
void SetTunings(double Kp, double Ki, double Kd)
{
	if (Kp < 0 || Ki < 0 || Kd < 0) return;
 
	double SampleTimeInSec = ((double)PID_Data.SampleTime) / 1000;
	PID_Data.Kp = Kp;
	PID_Data.Ki = Ki * SampleTimeInSec;
	PID_Data.Kd = Kd / SampleTimeInSec;
 
	if (PID_Data.controllerDirection == REVERSE)
	{
		PID_Data.Kp = (0 - PID_Data.Kp);
		PID_Data.Ki = (0 - PID_Data.Ki);
		PID_Data.Kd = (0 - PID_Data.Kd);
	}
}
 
void SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{
		double ratio  = (double)NewSampleTime / (double)PID_Data.SampleTime;
		PID_Data.Ki *= ratio;
		PID_Data.Kd /= ratio;
		PID_Data.SampleTime = (unsigned long)NewSampleTime;
	}
}
 
void SetOutputLimits(double Min, double Max)
{
	if (Min > Max) return;
	PID_Data.outMin = Min;
	PID_Data.outMax = Max;
 
	PID_Data.Output = (PID_Data.Output > PID_Data.outMax) ? PID_Data.outMax : (PID_Data.Output < PID_Data.outMin) ? PID_Data.outMin : PID_Data.Output;
	PID_Data.ITerm = (PID_Data.ITerm > PID_Data.outMax) ? PID_Data.outMax : (PID_Data.ITerm < PID_Data.outMin) ? PID_Data.outMin : PID_Data.ITerm;
}
 
void Initialize()
{
	PID_Data.lastInput = PID_Data.Input;
	PID_Data.ITerm = (PID_Data.ITerm > PID_Data.outMax) ? PID_Data.outMax : (PID_Data.ITerm < PID_Data.outMin) ? PID_Data.outMin : PID_Data.Output;
}
 
void SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !PID_Data.inAuto)
	{
		/*we just went from manual to auto*/
		Initialize();
	}
	PID_Data.inAuto = newAuto;
}
 
void SetControllerDirection(int Direction)
{
	PID_Data.controllerDirection = Direction;
}

void PID_task(void *pvParameter)
{
	// init PID
	PID_Data.inAuto = false;
	PID_Data.SampleTime = 1000;
	PID_Data.controllerDirection = DIRECT;
	
	
	while (1)
	{
		Compute();
		vTaskDelay(PID_Data.SampleTime / portTICK_PERIOD_MS);
	}
}


