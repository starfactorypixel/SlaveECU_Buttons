#pragma once
#include <inttypes.h>
#include <DrakePinD.hpp>

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
				CANLib::obj_buttonled_cn2.SetValue(0, btn_number);
				CANLib::obj_buttonled_cn2.SetValue(1, btn_state, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				break;
			}
			case 1:
			{
				CANLib::obj_buttonled_cn3.SetValue(0, btn_number);
				CANLib::obj_buttonled_cn3.SetValue(1, btn_state, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				break;
			}
			case 2:
			{
				CANLib::obj_buttonled_cn4.SetValue(0, btn_number);
				CANLib::obj_buttonled_cn4.SetValue(1, btn_state, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				break;
			}
			case 3:
			{
				CANLib::obj_buttonled_cn5.SetValue(0, btn_number);
				CANLib::obj_buttonled_cn5.SetValue(1, btn_state, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				break;
			}
		}
		
		return;
	}
	
	// Вызывается при отправка команды Set серез CAN
	can_result_t OnButtonSet(can_frame_t &can_frame, can_error_t &error)
	{
		// Временная заглушка. Отвечаем такими-же данными как получили
		//CANLib::obj_buttonled_cn2.SetValue(0, can_frame.data[0], CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
		//CANLib::obj_buttonled_cn2.SetValue(1, can_frame.data[1], CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
		
		can_frame.initialized = true;
		can_frame.function_id = CAN_FUNC_EVENT_OK;
		
		return CAN_RESULT_CAN_FRAME;
	}
	
	inline void Setup()
	{
		LedEn.Init();
		// Реализовать управление LedEn

		SPI::hc165.SetCallback(OnButtonsUpdate);
		CANLib::obj_buttonled_cn2.RegisterFunctionSet(OnButtonSet);
		CANLib::obj_buttonled_cn3.RegisterFunctionSet(OnButtonSet);
		CANLib::obj_buttonled_cn4.RegisterFunctionSet(OnButtonSet);
		CANLib::obj_buttonled_cn5.RegisterFunctionSet(OnButtonSet);

		LedEn.On();
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		
		
		// При выходе обновляем время
		current_time = HAL_GetTick();
		
		return;
	}
}
