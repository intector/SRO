/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * a2s servo lib
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


#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"


#pragma once

//You can get these value from the datasheet of servo you use, in general pulse width varies between 1000 to 2000 mocrosecond
#define SERVO_MIN_PULSEWIDTH 500	// Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500	// Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 180		// Maximum angle in degree upto which servo can rotate
#define SERVO_SPEED 10				// servo speed

#define SERVO_ANGLE_TO_PWM(degree_of_rotation) ({ (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE))); })

