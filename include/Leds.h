#pragma once

#include <LEDLibrary.h>

namespace Leds
{
	/*
		Инициализация пинов перенесена в бибилиотеку.
		Из main необходимо выпилить HAL_GPIO_Init со всеми пинами светодиодов.
		При портировании - удалить !
	*/
	#warning !!! READ ABOVE !!!
	
	static constexpr uint8_t CFG_LedCount = 2;
	enum leds_t : uint8_t
	{
		LED_NONE = 0,
		LED_RED = 1,
		LED_WHITE = 2,
	};
	
	InfoLeds<CFG_LedCount> obj;
	
	inline void Setup()
	{
		obj.AddLed( {GPIOB, GPIO_PIN_2}, LED_RED );
		obj.AddLed( {GPIOB, GPIO_PIN_4}, LED_WHITE );
		
		obj.SetOn(LED_RED);
		HAL_Delay(100);
		obj.SetOff(LED_RED);
		
		obj.SetOn(LED_WHITE);
		HAL_Delay(100);
		obj.SetOff(LED_WHITE);
		
		return;
	}

	inline void Loop(uint32_t &current_time)
	{
		obj.Processing(current_time);

		current_time = HAL_GetTick();

		return;
	}
}
