/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * APA102C LED
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

#ifndef __APA102C_DRV_H__
#define __APA102C_DRV_H__

#include <stdio.h>
#include <stdbool.h>
//#include <string>
#include <math.h>
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board.h"


typedef struct _led_pixel
{
	uint8_t A;
	uint8_t B;
	uint8_t G;
	uint8_t R;
} LED_PIXEL;

// LED status descriptor
typedef struct
{
	uint8_t _led_status;
	uint8_t _Color_Idx;
	uint8_t _Brightness;
	uint8_t _blink_step;
	uint8_t _blink_value;
} LED_Status_Desc;

// Define a vector type
typedef struct
{
	int led_count;
	int size;
	int start_frame_size;
	int pixel_data_size;
	int end_frame_size;
	uint8_t *data;
} APA102_DATA_BLOCK;

#define APA102_MAX_CNT 300

#define APA102_DATA_INIT_CONFIG_DEFAULT() { \
	.led_count = 0,			\
	.size = 0,				\
	.start_frame_size = 4,	\
	.pixel_data_size = 0,	\
	.end_frame_size = 0,	\
	.data = NULL,			\
};

#define APA102C_GREEN_OFFS       0
#define APA102C_RED_OFFS         1
#define APA102C_BLUE_OFFS        2

//	the start frame concists of 32 bits were each must be zero (<0x00> <0x00> <0x00> <0x00>)
//	A 32 bit LED frame for each LED in the string (<0xE0+brightness> <blue> <green> <red>)
//	An end frame consisting of at least (n/2) bits of 1, where n is the number of LEDs in the string.

#define APA102C_LED_MAX_NUM     300	
#define APA102C_START_TEL_CNT	4
#define APA102C_SPI_BYTE_PER_LED 4

#define LED_BRIGHTNESS_25		25			// LED brighness level = 25%
#define LED_BRIGHTNESS_50		50			// LED brighness level = 50%
#define LED_BRIGHTNESS_75		75			// LED brighness level = 75%
#define LED_BRIGHTNESS_100		100			// LED brighness level = 100%

#define LED_OFF_IDX			0
#define LED_RED_IDX			1
#define LED_ORANGE_IDX		2
#define LED_YELLOW_IDX		3
#define LED_GREEN_IDX		4
#define LED_AQUA_IDX		5
#define LED_BLUE_IDX		6
#define LED_MAGENTA_IDX		7
#define LED_WHITE_IDX		8

#define LED_IDX_WIFI		3			// status LED for WIFI
#define LED_IDX_MQTT		2			// status LED for MQTT
#define LED_IDX_TEMP		4			// status LED for oven temperature
#define LED_IDX_HEATER		1			// status LED for heater
#define LED_IDX_FAN			0			// status LED for convection fan

#define LED_STATUS_OFF		0			// LED status OFF
#define LED_STATUS_ON		1			// LED status ON
#define LED_STATUS_BLINK	2			// LED status blinking

#define CMD_NOP				0xFF		// do nothing
#define CMD_LED_OFF			0xFE		// led off

#define CMD_RED_ON			0x01		// turn on red
#define CMD_ORANGE_ON		0x02		// turn on orange
#define CMD_YELLOW_ON		0x03		// turn on yellow
#define CMD_GREEN_ON		0x04		// turn on green
#define CMD_AQUA_ON			0x05		// turn on aqua
#define CMD_BLUE_ON			0x06		// turn on blue
#define CMD_MAGENTA_ON		0x07		// turn on magenta
#define CMD_WHITE_ON		0x08		// turn on white

#define CMD_RED_BLINK		0x11		// blink on red
#define CMD_ORANGE_BLINK	0x12		// blink on orange
#define CMD_YELLOW_BLINK	0x13		// blink on yellow
#define CMD_GREEN_BLINK		0x14		// blink on green
#define CMD_AQUA_BLINK		0x15		// blink on aqua
#define CMD_BLUE_BLINK		0x16		// blink on blue
#define CMD_MAGENTA_BLINK	0x17		// blink on magenta
#define CMD_WHITE_BLINK		0x18		// blink on white

#define SPI_IF_BIT_RATE  SPI_MASTER_FREQ_20M

extern LED_PIXEL LED_Pixel;
extern LED_Status_Desc _LED_Status_Desc[];
extern APA102_DATA_BLOCK APA102_Data;

static LED_PIXEL ledcOFF = { 0x00, 0x00, 0x00, 0x00 };
static LED_PIXEL ledcMagenta = { 0x0F, 0xFF, 0x00, 0xFF };
static LED_PIXEL ledcYellow = { 0x0F, 0x00, 0xFF, 0xFF };
static LED_PIXEL ledcAqua = { 0x0F, 0xFF, 0xFF, 0x00 };
static LED_PIXEL ledcBlue = { 0x0F, 0xFF, 0x00, 0x00 };
static LED_PIXEL ledcGreen = { 0x0F, 0x00, 0xFF, 0x00 };
static LED_PIXEL ledcRed = { 0x0F, 0x00, 0x00, 0xFF };
static LED_PIXEL ledcOrange = { 0x0F, 0x00, 0x7F, 0xFF };
static LED_PIXEL ledcWhite = { 0x0F, 0xFF, 0xFF, 0xFF };


//*****************************************************************************
//
// Function prototypes
//
//*****************************************************************************

extern void APA102C_Init(int LED_CNT);
extern void LED_InitSequence();
extern void SetLEDBrightness(int LED_Num, uint8_t Level);
extern void SetLEDColor(int LED_IDX, LED_PIXEL LED_Color);
extern void LED_SetStatus(int LED_IDX, uint8_t LED_Status, uint8_t LED_ColorIdx, uint8_t Brightness);
extern void LED_Status_CTRL();
extern void LED_CTRL_Task(void *pvParameter);


#endif // __APA102C_DRV_H__
