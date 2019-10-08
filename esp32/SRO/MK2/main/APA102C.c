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

#include "APA102C.h"
#include "globals.h"

//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"


APA102_DATA_BLOCK APA102_Data = APA102_DATA_INIT_CONFIG_DEFAULT();
LED_PIXEL LED_Pixel = { 0x00, 0x00, 0x00, 0x00 };
LED_Status_Desc _LED_Status_Desc[5];
esp_err_t ret;
spi_device_handle_t spi;
static spi_transaction_t trans;


/******************************************************************************
 *                LED blink timer call back function
 *
 * parameter:	pxTimer	- handle of timer what called this function
 *				
 * 
******************************************************************************/
void LED_Blink_Timer_Callback(TimerHandle_t pxTimer)
{
	LED_Status_CTRL();
}

/******************************************************************************
 *                            APA102C Init
 *
 * parameter:	LED_CNT	- number of LES's in stripe
 *				
 * 
******************************************************************************/

void APA102C_Init(int LED_CNT)
{
	// plausibility check for LED count
	LED_CNT = (LED_CNT <= 0) ? 1 : LED_CNT;
	LED_CNT = (LED_CNT > APA102C_LED_MAX_NUM) ? APA102C_LED_MAX_NUM : LED_CNT;
	
	APA102_Data.led_count = LED_CNT;

	APA102_Data.end_frame_size = ((APA102_Data.led_count / 2.0) > (APA102_Data.led_count / 2)) ? (int)((APA102_Data.led_count / 2.0) + 1) : (int)(APA102_Data.led_count / 2.0);
	APA102_Data.pixel_data_size = sizeof(LED_PIXEL) * LED_CNT;
	APA102_Data.size = APA102_Data.start_frame_size + APA102_Data.pixel_data_size  + APA102_Data.end_frame_size;
	APA102_Data.data = calloc(APA102_Data.size, sizeof(uint8_t));

	for (int Idx = 0; Idx < APA102_Data.led_count; Idx++)
	{
		_LED_Status_Desc[Idx]._led_status = LED_STATUS_OFF;
		_LED_Status_Desc[Idx]._Color_Idx = LED_OFF_IDX;
		_LED_Status_Desc[Idx]._Brightness = 0xEF;
		_LED_Status_Desc[Idx]._blink_value = 0;
		_LED_Status_Desc[Idx]._blink_step = 0;
	}
	
	SetLEDBrightness(-1, 0x03);

	memset(&trans, 0, sizeof(trans));

	spi_bus_config_t buscfg = {
		.miso_io_num = SPI_NOT_USED,
		.mosi_io_num = PIN_NUM_MOSI,
		.sclk_io_num = PIN_NUM_CLK,
		.quadwp_io_num = SPI_NOT_USED,
		.quadhd_io_num = SPI_NOT_USED
	};

	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = SPI_IF_BIT_RATE, 
		.mode = 3, 
		.spics_io_num = SPI_NOT_USED, 
		.queue_size = 1, 
	};

	
	// Initialize the SPI bus
	// HSPI_HOST or VSPI_HOST can be used
	ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
	assert(ret == ESP_OK);

	// Attach the device to the SPI bus
	// ATENTION: !!! trans.length is in bits !!!
	ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spi);
	trans.length = APA102_Data.size * 8;     // !!! trans.length is in bits !!!
	trans.tx_buffer = APA102_Data.data;
	trans.user = (void*)1;
	
	LED_InitSequence();
	
	LED_SetStatus(LED_IDX_TEMP, LED_STATUS_OFF, LED_OFF_IDX, 0x00);
	LED_SetStatus(LED_IDX_WIFI, LED_STATUS_OFF, LED_OFF_IDX, 0x00);
	LED_SetStatus(LED_IDX_MQTT, LED_STATUS_OFF, LED_OFF_IDX, 0x00);
	LED_SetStatus(LED_IDX_HEATER, LED_STATUS_OFF, LED_OFF_IDX, 0x00);
	LED_SetStatus(LED_IDX_FAN, LED_STATUS_OFF, LED_OFF_IDX, 0x00);


	xReturned = xTaskCreate(&LED_CTRL_Task, "LED_Task", 2048, NULL, 5, &hLED_CTRL_Task);
	if (xReturned != pdPASS) {
		// task could not be ceated
	}
	
//	LED_Blink_Timer = xTimerCreate("LED_BlinkTimer", 15 / portTICK_PERIOD_MS, pdTRUE, NULL, LED_Blink_Timer_Callback);
//	if (LED_Blink_Timer == NULL) {
		// The timer was not created.
//	}
//	else {
//		if (xTimerStart(LED_Blink_Timer, 0) != pdPASS) {
			// The timer could not be set into the Active state.
//		}
//	}
	
	// signal that LED's are ready
	LED_InitReadyFlag = true;
	
}

/******************************************************************************
 *                            LED Intit Sequence
 *
 * parameter:	none
 *				
 * just for signaling the ESP32 init is running, but mainly for awesome looks
 * 
******************************************************************************/
void LED_InitSequence()
{
	uint8_t _tmp_LED_Brightness = 100;
	LED_PIXEL tmpColor = ledcRed;

	for (int Idx1 = 0; Idx1 < 3; Idx1++)
	{
		switch (Idx1)
		{
		case 0:
			tmpColor = ledcRed;
			break;
		case 1:
			tmpColor = ledcBlue;
			break;
		case 2:
			tmpColor = ledcGreen;
			break;
		default:
			tmpColor = ledcRed;
		}
		for (int Idx2 = 0; Idx2 < 16; Idx2++)
		{
			SetLEDColor(-1, ledcOFF);
			switch (Idx2)
			{
			case 0: 
			case 14:
				SetLEDColor(0, tmpColor);
				SetLEDBrightness(-1, _tmp_LED_Brightness);
				break;
			case 1: 
			case 13:
				SetLEDColor(0, tmpColor);
				SetLEDColor(1, tmpColor);
				SetLEDBrightness(-1, _tmp_LED_Brightness);
				break;
			case 2: 
			case 12:
				SetLEDColor(0, tmpColor);
				SetLEDColor(1, tmpColor);
				SetLEDColor(2, tmpColor);
				SetLEDBrightness(-1, _tmp_LED_Brightness);
				break;
			case 3: 
			case 11:
				SetLEDColor(1, tmpColor);
				SetLEDColor(2, tmpColor);
				SetLEDColor(3, tmpColor);
				SetLEDBrightness(-1, _tmp_LED_Brightness);
				break;
			case 4: 
			case 10:
				SetLEDColor(2, tmpColor);
				SetLEDColor(3, tmpColor);
				SetLEDColor(4, tmpColor);
				SetLEDBrightness(-1, _tmp_LED_Brightness);
				break;
			case 5: 
			case 9:
				SetLEDColor(3, tmpColor);
				SetLEDColor(4, tmpColor);
				SetLEDBrightness(-1, _tmp_LED_Brightness);
				break;
			case 6: 
			case 8:
				SetLEDColor(4, tmpColor);
				SetLEDBrightness(-1, _tmp_LED_Brightness);
				break;
			default:
				SetLEDColor(-1, ledcOFF);
			}
			ret = spi_device_transmit(spi, &trans);
			ESP_ERROR_CHECK(ret);
			vTaskDelay(50 / portTICK_PERIOD_MS);
		}
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
}


/******************************************************************************
 *                            SET LED brightness
 *
 * parameter:	LED_IDX	- index of LED starting with 0
 *				Level	- level of brightness in %
 *				
 * if parameter LED_IDX is set to -1, all LED's will be modified
 * 
******************************************************************************/
void SetLEDBrightness(int LED_IDX, uint8_t Level)
{
	// plausibility check for LED count
	LED_IDX = (LED_IDX > APA102_Data.led_count) ? APA102_Data.led_count : LED_IDX;

	uint8_t _tmpLevel = (Level <= 0) ? 0 : ((int)(31 * Level / 100));
	
	if (LED_IDX < 0)
	{
		for (int idx = 0; idx < APA102_Data.led_count; idx++)
		{
			APA102_Data.data[APA102_Data.start_frame_size + 4 * idx] = 0xE0 | _tmpLevel;
		}
	}
	else
	{
		APA102_Data.data[LED_IDX] = 0xE0 | _tmpLevel;
	}
	
}

/******************************************************************************
 *                            SET LED color
 *
 * parameter:	LED_IDX		- index of LED starting with 0
 *				LED_Color	- color definition in form of "LED_PIXEL"
 *				
 * if parameter LED_IDX is set to -1, all LED's will be changed
 * 
******************************************************************************/
void SetLEDColor(int LED_IDX, LED_PIXEL LED_Color)
{
	// plausibility check for LED count
	LED_IDX = (LED_IDX > APA102_Data.led_count) ? APA102_Data.led_count : LED_IDX;
	
	if (LED_IDX < 0)
	{
		for (int idx = 0; idx < APA102_Data.led_count; idx++)
		{
			APA102_Data.data[APA102_Data.start_frame_size + 4 * idx + 0] = 0xE0 | LED_Color.A;
			APA102_Data.data[APA102_Data.start_frame_size + 4 * idx + 1] = LED_Color.B;
			APA102_Data.data[APA102_Data.start_frame_size + 4 * idx + 2] = LED_Color.G;
			APA102_Data.data[APA102_Data.start_frame_size + 4 * idx + 3] = LED_Color.R;
		}
	}
	else
	{
		APA102_Data.data[APA102_Data.start_frame_size + 4 * LED_IDX + 0] = 0xE0 | LED_Color.A;
		APA102_Data.data[APA102_Data.start_frame_size + 4 * LED_IDX + 1] = LED_Color.B;
		APA102_Data.data[APA102_Data.start_frame_size + 4 * LED_IDX + 2] = LED_Color.G;
		APA102_Data.data[APA102_Data.start_frame_size + 4 * LED_IDX + 3] = LED_Color.R;
	}
}

/******************************************************************************
 *                            LED set status
 *
 * parameter:	LED_IDX			- index of LED starting with 0
 *				LED_Status		- LED status OFF, ON or BLINK
 *				LED_ColorIdx	- color index
 *				Brightness		- LED brightness
 *				
 * function to set the APA102C LED mode
 * 
******************************************************************************/
void LED_SetStatus(int LED_IDX, uint8_t LED_Status, uint8_t LED_ColorIdx, uint8_t Brightness)
{
	_LED_Status_Desc[LED_IDX]._led_status = LED_Status;
	_LED_Status_Desc[LED_IDX]._Color_Idx = LED_ColorIdx;
	_LED_Status_Desc[LED_IDX]._Brightness = Brightness;
	_LED_Status_Desc[LED_IDX]._blink_value = 0;
	_LED_Status_Desc[LED_IDX]._blink_step = 0;
}

/******************************************************************************
 *                            LED status CTRL
 *
 * parameter:	none
 *				
 * function for LED status timer
 * 
******************************************************************************/
void LED_Status_CTRL()
{
	for (int Idx = 0; Idx < APA102_Data.led_count; Idx++)
	{
		switch (_LED_Status_Desc[Idx]._led_status)
		{
			case LED_STATUS_OFF:
				SetLEDColor(Idx, ledcOFF);
				break;
			case LED_STATUS_ON: 
				switch (_LED_Status_Desc[Idx]._Color_Idx)
				{
					case LED_RED_IDX:
						SetLEDColor(Idx, ledcRed);
						break;
					case LED_ORANGE_IDX:
						SetLEDColor(Idx, ledcOrange);
						break;
					case LED_YELLOW_IDX:
						SetLEDColor(Idx, ledcYellow);
						break;
					case LED_GREEN_IDX:
						SetLEDColor(Idx, ledcGreen);
						break;
					case LED_AQUA_IDX:
						SetLEDColor(Idx, ledcAqua);
						break;
					case LED_BLUE_IDX:
						SetLEDColor(Idx, ledcBlue);
						break;
					case LED_MAGENTA_IDX:
						SetLEDColor(Idx, ledcMagenta);
						break;
					case LED_WHITE_IDX:
						SetLEDColor(Idx, ledcWhite);
						break;
					default:
						SetLEDColor(Idx, ledcOFF);
				}
				APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0 | _LED_Status_Desc[Idx]._Brightness;
				break;
			case LED_STATUS_BLINK:
				if (_LED_Status_Desc[Idx]._blink_step < 0x66)
					_LED_Status_Desc[Idx]._blink_step++;
				else
					_LED_Status_Desc[Idx]._blink_step = 0x00;
			
				_LED_Status_Desc[Idx]._blink_value = (_LED_Status_Desc[Idx]._blink_step < 0x33) ? (_LED_Status_Desc[Idx]._blink_step * 5) : (0xFF - (_LED_Status_Desc[Idx]._blink_step - 0x33) * 5);
			
				switch (_LED_Status_Desc[Idx]._Color_Idx)
				{
					case LED_ORANGE_IDX:
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0 | _LED_Status_Desc[Idx]._Brightness;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 1] = 0x00;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 2] = _LED_Status_Desc[Idx]._blink_value / 2;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 3] = _LED_Status_Desc[Idx]._blink_value;
						break;
					case LED_RED_IDX:
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0 | _LED_Status_Desc[Idx]._Brightness;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 1] = 0x00;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 2] = 0x00;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 3] = _LED_Status_Desc[Idx]._blink_value;
						break;
					case LED_YELLOW_IDX:
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0 | _LED_Status_Desc[Idx]._Brightness;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 1] = 0x00;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 2] = _LED_Status_Desc[Idx]._blink_value;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 3] = _LED_Status_Desc[Idx]._blink_value;
						break;
					case LED_GREEN_IDX:
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0 | _LED_Status_Desc[Idx]._Brightness;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 1] = 0x00;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 2] = _LED_Status_Desc[Idx]._blink_value;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 3] = 0x00;
						break;
					case LED_BLUE_IDX:
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0 | _LED_Status_Desc[Idx]._Brightness;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 1] = _LED_Status_Desc[Idx]._blink_value;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 2] = 0x00;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 3] = 0x00;
						break;
					case LED_AQUA_IDX:
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0 | _LED_Status_Desc[Idx]._Brightness;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 1] = _LED_Status_Desc[Idx]._blink_value;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 2] = _LED_Status_Desc[Idx]._blink_value;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 3] = 0x00;
						break;
					case LED_MAGENTA_IDX:
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0 | _LED_Status_Desc[Idx]._Brightness;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 1] = _LED_Status_Desc[Idx]._blink_value;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 2] = 0x00;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 3] = _LED_Status_Desc[Idx]._blink_value;
						break;
					case LED_WHITE_IDX:
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0 | _LED_Status_Desc[Idx]._Brightness;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 1] = _LED_Status_Desc[Idx]._blink_value;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 2] = _LED_Status_Desc[Idx]._blink_value;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 3] = _LED_Status_Desc[Idx]._blink_value;
						break;
					default:
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 0] = 0xE0;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 1] = 0x00;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 2] = 0x00;
						APA102_Data.data[APA102_Data.start_frame_size + 4 * Idx + 3] = 0x00;
					}
					break;
			}
	}
	ret = spi_device_transmit(spi, &trans);
	ESP_ERROR_CHECK(ret);
}	

void LED_CTRL_Task(void *pvParameter)
{
	uint32_t ulNotifiedValue;
	BaseType_t xReturned; 

	while (1)
	{
		xReturned = xTaskNotifyWait(0x00, ULONG_MAX, &ulNotifiedValue, 15 / portTICK_PERIOD_MS);

		if (xReturned)
		{
			// events for WIFI LED
			if((ulNotifiedValue & TN_WIFI_CONNECTED) && (_LED_Status_Desc[LED_IDX_WIFI]._led_status != LED_STATUS_ON) && (_LED_Status_Desc[LED_IDX_WIFI]._Color_Idx != LED_GREEN_IDX))
				LED_SetStatus(LED_IDX_WIFI, LED_STATUS_ON, LED_GREEN_IDX, 0x07);
		
			if ((ulNotifiedValue & TN_WIFI_DISCONNECTED) && (_LED_Status_Desc[LED_IDX_WIFI]._led_status != LED_STATUS_BLINK) && (_LED_Status_Desc[LED_IDX_WIFI]._Color_Idx != LED_WHITE_IDX))
				LED_SetStatus(LED_IDX_WIFI, LED_STATUS_BLINK, LED_WHITE_IDX, 0x03);

			if ((ulNotifiedValue & TN_WIFI_ERROR) && (_LED_Status_Desc[LED_IDX_WIFI]._led_status != LED_STATUS_BLINK) && (_LED_Status_Desc[LED_IDX_WIFI]._Color_Idx != LED_RED_IDX))
				LED_SetStatus(LED_IDX_WIFI, LED_STATUS_BLINK, LED_RED_IDX, 0x07);
		
			// events for MQTT LED
			if((ulNotifiedValue & TN_MQTT_CONNECTED) && (_LED_Status_Desc[LED_IDX_MQTT]._led_status != LED_STATUS_ON) && (_LED_Status_Desc[LED_IDX_MQTT]._Color_Idx != LED_GREEN_IDX))
				LED_SetStatus(LED_IDX_MQTT, LED_STATUS_ON, LED_GREEN_IDX, 0x07);
		
			if ((ulNotifiedValue & TN_MQTT_DISCONNECTED) && (_LED_Status_Desc[LED_IDX_MQTT]._led_status != LED_STATUS_BLINK) && (_LED_Status_Desc[LED_IDX_MQTT]._Color_Idx != LED_WHITE_IDX))
				LED_SetStatus(LED_IDX_MQTT, LED_STATUS_BLINK, LED_WHITE_IDX, 0x03);

			if ((ulNotifiedValue & TN_MQTT_OFF) && (_LED_Status_Desc[LED_IDX_MQTT]._led_status != LED_STATUS_OFF) && (_LED_Status_Desc[LED_IDX_MQTT]._Color_Idx != LED_OFF_IDX))
				LED_SetStatus(LED_IDX_MQTT, LED_STATUS_OFF, LED_OFF_IDX, 0x00);

			if ((ulNotifiedValue & TN_MQTT_ERROR) && (_LED_Status_Desc[LED_IDX_TEMP]._led_status != LED_STATUS_BLINK) && (_LED_Status_Desc[LED_IDX_TEMP]._Color_Idx != LED_RED_IDX))
				LED_SetStatus(LED_IDX_MQTT, LED_STATUS_BLINK, LED_RED_IDX, 0x07);

			// events for temparature LED
			if((ulNotifiedValue & TN_TEMP_COLD) && (_LED_Status_Desc[LED_IDX_TEMP]._led_status != LED_STATUS_ON) && (_LED_Status_Desc[LED_IDX_TEMP]._Color_Idx != LED_BLUE_IDX))
				LED_SetStatus(LED_IDX_TEMP, LED_STATUS_ON, LED_BLUE_IDX, 0x07);
		
			if ((ulNotifiedValue & TN_TEMP_HOT) && (_LED_Status_Desc[LED_IDX_TEMP]._led_status != LED_STATUS_BLINK) && (_LED_Status_Desc[LED_IDX_TEMP]._Color_Idx != LED_MAGENTA_IDX))
				LED_SetStatus(LED_IDX_TEMP, LED_STATUS_BLINK, LED_MAGENTA_IDX, 0x1F);

			if ((ulNotifiedValue & TN_TEMP_ERROR) && (_LED_Status_Desc[LED_IDX_TEMP]._led_status != LED_STATUS_BLINK) && (_LED_Status_Desc[LED_IDX_TEMP]._Color_Idx != LED_RED_IDX))
				LED_SetStatus(LED_IDX_TEMP, LED_STATUS_BLINK, LED_RED_IDX, 0x07);
	
			// events for heater LED
			if((ulNotifiedValue & TN_HEATER_ON) && (_LED_Status_Desc[LED_IDX_HEATER]._led_status != LED_STATUS_ON) && (_LED_Status_Desc[LED_IDX_HEATER]._Color_Idx != LED_GREEN_IDX))
				LED_SetStatus(LED_IDX_HEATER, LED_STATUS_ON, LED_GREEN_IDX, 0x07);
		
			if ((ulNotifiedValue & TN_HEATER_OFF) && (_LED_Status_Desc[LED_IDX_HEATER]._led_status != LED_STATUS_OFF) && (_LED_Status_Desc[LED_IDX_HEATER]._Color_Idx != LED_OFF_IDX))
				LED_SetStatus(LED_IDX_HEATER, LED_STATUS_OFF, LED_OFF_IDX, 0x07);

			if ((ulNotifiedValue & TN_HEATER_ERROR) && (_LED_Status_Desc[LED_IDX_HEATER]._led_status != LED_STATUS_BLINK) && (_LED_Status_Desc[LED_IDX_HEATER]._Color_Idx != LED_RED_IDX))
				LED_SetStatus(LED_IDX_HEATER, LED_STATUS_BLINK, LED_RED_IDX, 0x07);

			// events for fan LED
			if((ulNotifiedValue & TN_FAN_ON) && (_LED_Status_Desc[LED_IDX_FAN]._led_status != LED_STATUS_ON) && (_LED_Status_Desc[LED_IDX_FAN]._Color_Idx != LED_GREEN_IDX))
				LED_SetStatus(LED_IDX_FAN, LED_STATUS_ON, LED_GREEN_IDX, 0x07);
		
			if ((ulNotifiedValue & TN_FAN_OFF) && (_LED_Status_Desc[LED_IDX_FAN]._led_status != LED_STATUS_OFF) && (_LED_Status_Desc[LED_IDX_FAN]._Color_Idx != LED_OFF_IDX))
				LED_SetStatus(LED_IDX_FAN, LED_STATUS_OFF, LED_OFF_IDX, 0x07);

			if ((ulNotifiedValue & TN_FAN_ERROR) && (_LED_Status_Desc[LED_IDX_FAN]._led_status != LED_STATUS_BLINK) && (_LED_Status_Desc[LED_IDX_FAN]._Color_Idx != LED_RED_IDX))
				LED_SetStatus(LED_IDX_FAN, LED_STATUS_BLINK, LED_RED_IDX, 0x07);
		}
		
		LED_Status_CTRL();
		
	}
}



