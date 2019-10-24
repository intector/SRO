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

#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <time.h>
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "globals.h"

typedef struct __Par {
	double InputA_min, InputA_set, InputA_max;
	double InputB_min, InputB_set, InputB_max;
	double InputC_min, InputC_set, InputC_max;
	double InputD_min, InputD_set, InputD_max;
	double InputE_min, InputE_set, InputE_max;
	double InputF_min, InputF_set, InputF_max;
	double InputG_min, InputG_set, InputG_max;
	double InputH_min, InputH_set, InputH_max;
	double InputI_min, InputI_set, InputI_max;
} SRO_Par;

typedef struct __ParameterSet {
	uint8_t ActParSet;
	SRO_Par ParSet;
} SRO_ParSet;

typedef struct __PID_Data {
	double Kp, Ki, Kd;
	double Input, Output, Setpoint;
	double ITerm, lastInput;
	double outMin, outMax;
	bool inAuto;
	uint64_t SampleTime, Runtime;
	struct timespec actTime, lastTime;
	int controllerDirection;
	double HU_Ramp, CD_Ramp;
	double TestTemp, OvenMaxTemp, HoldTime;
} SRO_PID_Data;


extern SRO_ParSet SolParSet;
extern SRO_PID_Data PID_Data;


#define BILLION 1000000000L

#define MANUAL 0
#define AUTOMATIC 1
 
#define DIRECT 0
#define REVERSE 1


void SetTunings(double Kp, double Ki, double Kd);
void SetMode(int Mode);
void SetOutputLimits(double Min, double Max);
void SetSampleTime(int NewSampleTime);
void SetControllerDirection(int Direction);
void PID_task(void *pvParameter);
