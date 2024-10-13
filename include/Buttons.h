#pragma once
#include <inttypes.h>

namespace ButtonsLeds
{
	/*
	void OnChange(uint8_t port, bool state)
	{
		Logger.PrintTopic("BTN").Printf("port: %2d, state: %d;", port, state).PrintNewLine();
		
		obj.SetLedState(port, state);

		//uint8_t set_byte = (state == true) ? 0xFF : 0x00;
		uint8_t param = (state == true) ? 0xFF : 0x00;
		//uint8_t send_data[1] = {param};
		
		switch(port)
		{
			case 1:
			{
				// Ближний свет
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_low_beam, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				CANLib::obj_buttonled_01.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				
				break;
			}
			case 2:
			{
				// Габариты, перёд, зад.
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_side_beam_f, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_side_beam_r, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				CANLib::obj_buttonled_02.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 3:
			{
				// Свет в салоне
				CANLib::obj_buttonled_03.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 4:
			{
				// Клаксон
				CANLib::obj_buttonled_04.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 5:
			{
				// Аварийка
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_hazard_beam_f, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_hazard_beam_r, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				CANLib::obj_buttonled_05.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 6:
			{
				// Вентилятор
				CANLib::obj_buttonled_06.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 7:
			{
				// Левый поворотник
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_left_indicator_f, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_left_indicator_r, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				CANLib::obj_buttonled_07.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				
				break;
			}
			case 8:
			{
				// Педаль тормоза
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_brake_light, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				CANLib::obj_buttonled_08.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 9:
			{
				// Кнопка открытия капота
				CANLib::obj_buttonled_09.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 10:
			{
				// Кнопка открытия багажника
				CANLib::obj_buttonled_10.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 11:
			{
				// Кнопка открытия левой двери
				CANLib::obj_buttonled_11.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 12:
			{
				// Правый поворотник
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_right_indicator_f, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_right_indicator_r, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				CANLib::obj_buttonled_12.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 13:
			{
				// Концевик левая дверь
				CANLib::obj_buttonled_13.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 14:
			{
				// Концевик правая дверь
				CANLib::obj_buttonled_14.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 15:
			{
				// Кнопка открытия правой двери
				CANLib::obj_buttonled_15.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);

				break;
			}
			case 16:
			{
				// Дальний свет
				//CANLib::can_manager.SendCustomFrame(CANLib::obj_high_beam, CAN_FUNC_SET_IN, send_data, sizeof(send_data));
				CANLib::obj_buttonled_16.SetValue(0, param, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
				
				break;
			}
		}
		
		return;
	}
	*/
	
	inline void Setup()
	{
		
		SPI::hc165.SetCallback([](uint8_t device, uint8_t pin, bool state)
		{
			DEBUG_LOG_TOPIC("HC165", "device:%d, pin:%d, state:%d\n", device, pin, state);

			uint8_t btn_number = ((device * 8) + (pin + 1));
			uint8_t btn_state = state ? 255 : 0;
			
			CANLib::obj_button_action.SetValue(0, btn_number);
			CANLib::obj_button_action.SetValue(1, btn_state, CAN_TIMER_TYPE_NONE, CAN_EVENT_TYPE_NORMAL);
		});
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		current_time = HAL_GetTick();
		
		return;
	}
}
