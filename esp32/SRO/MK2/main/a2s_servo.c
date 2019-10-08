/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * SRO servo controller
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

#include "a2s_servo.h"


/******************************************************************************
 *         function to calcute pulse width for per degree rotation
 *
 * parameter:	degree_of_rotation	- degree_of_rotation the angle in degree
 *                                    to which servo has to rotate
 *				
 * return:
 *				calculated pulse width
 * 
******************************************************************************/
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
	uint32_t cal_pulsewidth = 0;
	cal_pulsewidth = (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
	return cal_pulsewidth;
}
