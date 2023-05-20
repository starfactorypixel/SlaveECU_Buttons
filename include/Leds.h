#pragma once

#include <LEDLibrary.h>

namespace Leds
{
	InfoLeds<2> ledsObj;

	inline void Setup()
	{
		ledsObj.AddLed( {GPIOB, GPIO_PIN_4}, ledsObj.LED_GREEN );
		ledsObj.AddLed( {GPIOB, GPIO_PIN_2}, ledsObj.LED_RED );

		ledsObj.SetOn(ledsObj.LED_RED);
		HAL_Delay(100);
		ledsObj.SetOff(ledsObj.LED_RED);

		// Когда было LED_YELLOW, всё падало..
		ledsObj.SetOn(ledsObj.LED_GREEN);
		HAL_Delay(100);
		ledsObj.SetOff(ledsObj.LED_GREEN);
	}

	inline void Loop(uint32_t &current_time)
	{
		ledsObj.Processing(current_time);

		current_time = HAL_GetTick();

		return;
	}
}
