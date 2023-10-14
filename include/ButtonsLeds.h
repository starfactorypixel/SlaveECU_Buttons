#pragma once

#include <UserButtons.h>

#define CLK_H()        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);	// установить выход SCK(clock) в High
#define CLK_L()         HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);	// установить выход SCK(clock) в Low
#define SDO_595_H()        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);	// установить выход SDO(Data) в High
#define SDO_595_L()     	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);	// установить выход SDO(Data) в Low
#define CE_H()      		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);	// установить выход защелка CE(SH_CP) в High
#define CE_L()       		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);	// установить выход защелка CE(SH_CP) в Low

#define OE_595_H()      		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);	// установить выход защелка в High
#define OE_595_L()       		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);	// установить выход защелка в Low
#define SRCLR_595_H()      	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);	// установить выход защелка в High
#define SRCLR_595_L()       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);	// установить выход защелка в Low

#define SH_LD_165_H()      	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);	// установить выход защелка в High
#define SH_LD_165_L()       HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);	// установить выход защелка в Low

extern SPI_HandleTypeDef hspi2;

namespace ButtonsLeds
{
	static constexpr uint8_t CFG_Count = 2;

	void hw_spi(uint8_t *buttons, uint8_t *leds, uint8_t length);
	
	UserButtons<CFG_Count> obj(&hw_spi);
	
	void hw_spi(uint8_t *buttons, uint8_t *leds, uint8_t length)
	{
		CE_L();        		// Write enadle
		SH_LD_165_L();		// Reset HC165
		SH_LD_165_H();
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t*) leds, (uint8_t*) buttons, length, 5000);
		CE_H();       		// Write disable

		//Serial::Printf("led: %02X %02X\r\n", leds[0], leds[1]);
	}
	
/*
	Port map
		[09]	[13]	[01]	[05]
		[10]	[14]	[02]	[06]
		[11]	[15]	[03]	[07]
		[12]	[16]	[04]	[08]

					[ SWD ]
*/

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
	
	inline void Setup()
	{
		// Reset shift register 74HC595 & 74HC165
		SRCLR_595_L();
		OE_595_L();
		SRCLR_595_H();
		SH_LD_165_H();
		
		obj.RegChangeFunction(&OnChange);
		//obj.SetButtonMode(9, obj.MODE_TRIGGER);
		//obj.SetButtonMode(10, obj.MODE_TRIGGER);
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		obj.Processing(current_time);
		
		current_time = HAL_GetTick();
		
		return;
	}
}
