#pragma once
#include <inttypes.h>
#include <DrakePinD.hpp>
#include <CanObj/IButtonObjSender.hpp>

extern IButtonObjSender &ButtonLed1;
extern IButtonObjSender &ButtonLed2;
extern IButtonObjSender &ButtonLed3;
extern IButtonObjSender &ButtonLed4;

namespace ButtonsLeds
{

	DrakePinD LedEn({GPIOA, GPIO_PIN_8}, DrakePin::Output, DrakePin::Low);
	
	// Вызывается при изменении состояния любой из кнопок
	void OnButtonsUpdate(uint8_t device, uint8_t pin, bool state)
	{
		DEBUG_LOG_TOPIC("HC165", "device: %d, pin: %d, state: %d\n", device, pin, state);
		
		uint8_t btn_number = ((device * 8) + (pin + 1));
		uint8_t btn_state = state ? 0x0F : 0xF0;

		switch(device)
		{
			case 0:
			{
				ButtonLed1.EventOk(btn_number, btn_state);
				break;
			}
			case 1:
			{
				ButtonLed2.EventOk(btn_number, btn_state);
				break;
			}
			case 2:
			{
				ButtonLed3.EventOk(btn_number, btn_state);
				break;
			}
			case 3:
			{
				ButtonLed4.EventOk(btn_number, btn_state);
				break;
			}
		}
		
		return;
	}
	
	// Вызывается при отправка команды Set через CAN
	void OnButtonSet(uint8_t btn, uint8_t state)
	{

	}
	
	inline void Setup()
	{
		LedEn.Init();
		// Реализовать управление LedEn

		LedEn.On();
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		
		current_time = HAL_GetTick();
		return;
	}
}
