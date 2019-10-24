/* Soldering Reflow Oven
 * Copyright (c) 2019-2020 Intector
 *
 * MAX6675 lib
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

//#include "freertos/FreeRTOS.h"
//#include "freertos/task.h"
//#include "freertos/timers.h"
//#include "freertos/semphr.h"
//#include "freertos/queue.h"
//#include "freertos/task.h"

#include "board.h"
#include "a2s_mqtt.h"
#include "MAX6675.h"
#include "APA102C.h"
#include "globals.h"

SRO_Temp ActTemp = TEMPERATURE_INIT();

void ReadTemperature_Timer_Callback(TimerHandle_t pxTimer)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	MAX6675_ReadTemperature(&ActTemp);
	
	if (!ActTemp.Sensor_OK)
	{
		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hLED_CTRL_Task, TN_TEMP_ERROR, eSetBits, &xHigherPriorityTaskWoken);
	}
	else if (ActTemp.int_C_Value < 45)
	{
		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hLED_CTRL_Task, TN_TEMP_COLD, eSetBits, &xHigherPriorityTaskWoken);
	}
	else
	{
		xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(hLED_CTRL_Task, TN_TEMP_HOT, eSetBits, &xHigherPriorityTaskWoken);
	}

	if (th_mqtt_periodic_task != NULL)
	{
		if (pdTRUE == xQueueSend(Temperature_QueueHandleId, (void *) &ActTemp, 0))
		{
			// temperature reading was succsessfull posted			
		}
	}

	if (xHigherPriorityTaskWoken)
		taskYIELD(); 
	
}

void MAX6675_Init()
{
	Temperature_QueueHandleId = xQueueCreate(1, sizeof(SRO_Temp));
	
	TemperatureRead_Timer = xTimerCreate("Timer", 1000 / portTICK_PERIOD_MS, pdTRUE, NULL, ReadTemperature_Timer_Callback);

	if (TemperatureRead_Timer == NULL)
	{
		xTaskNotify(hLED_CTRL_Task, TN_TEMP_ERROR, eSetBits);
		} 
	else
	{
		if (xTimerStart(TemperatureRead_Timer, 0) != pdPASS)
		{
			xTaskNotify(hLED_CTRL_Task, TN_TEMP_ERROR, eSetBits);
		}
	}
	
}

void WaitingLoop()
{
	for(int Idx=0;Idx<10;Idx++){
		// do nothing
	}
}

uint8_t MAX6675_spiread(void) { 
	uint8_t d = 0;

	for (int Idx = 7; Idx >= 0; Idx--)
	{
		gpio_set_level(GPIO_6675_SCK, 0);
		WaitingLoop();
		
		if(gpio_get_level(GPIO_6675_SO)) {
			//set the bit to 0 no matter what
			d |= (1 << Idx);
		}

		gpio_set_level(GPIO_6675_SCK, 1);
		WaitingLoop();

	}

	return d;
}

void MAX6675_ReadTemperature(SRO_Temp *data) {

//	volatile uint16_t v;
	uint16_t v;

	gpio_set_level(GPIO_6675_CS, 0);
	WaitingLoop();
	
	v = MAX6675_spiread();
	v <<= 8;
	v |= MAX6675_spiread();

	gpio_set_level(GPIO_6675_CS, 1);
	WaitingLoop();

	if (v & 0x4) {
		data->Sensor_OK = false;
		data->int_C_Value = 0; 
		data->int_F_Value = 0; 
	}
	else {
		v >>= 3;
		data->Sensor_OK = true;
		data->int_C_Value = v * 0.25; 
		data->int_F_Value = (v * 0.25) * 9.0 / 5.0 + 32; 
	}
	itoa(data->int_C_Value, data->str_C_Value, 10);
	itoa(data->int_F_Value, data->str_F_Value, 10);
}

	

