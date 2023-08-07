#pragma once

#include <CANLibrary.h>

void HAL_CAN_Send(uint16_t id, uint8_t *data_raw, uint8_t length_raw);

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart1;

namespace CANLib
{
	//*********************************************************************
	// CAN Library settings
	//*********************************************************************

	/// @brief Number of CANObjects in CANManager
	static constexpr uint8_t CFG_CANObjectsCount = 12;

	/// @brief The size of CANManager's internal CAN frame buffer
	static constexpr uint8_t CFG_CANFrameBufferSize = 16;

	//*********************************************************************
	// CAN Manager & CAN Object configuration
	//*********************************************************************

	CANManager<CFG_CANObjectsCount, CFG_CANFrameBufferSize> can_manager(&HAL_CAN_Send);

	// ******************** common blocks ********************
	// 0x0120	BlockInfo
	// request | timer:15000
	// byte	1 + 7	{ type[0] data[1..7] }
	// Основная информация о блоке. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_info(0x0120);

	// 0x0121	BlockHealth
	// request | event
	// byte	1 + 7	{ type[0] data[1..7] }
	// Информация о здоровье блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_health(0x0121);

	// 0x0122	BlockCfg
	// request
	// byte	1 + 1 + X	{ type[0] param[1] data[2..7] }
	// Чтение и запись настроек блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_features(0x0122);


	// 0x0123	BlockError
	// request | event
	// byte	1 + X	{ type[0] data[1..7] }
	// Ошибки блока. См. "Системные параметры".
	CANObject<uint8_t, 7> obj_block_error(0x0123);

	// ...
	CANObject<uint8_t, 1> obj_low_beam(0x00C5);

	CANObject<uint8_t, 1> obj_side_beam_f(0x00C4);
	CANObject<uint8_t, 1> obj_side_beam_r(0x00E4);

	CANObject<uint8_t, 1> obj_hazard_beam_f(0x00C9);
	CANObject<uint8_t, 1> obj_hazard_beam_r(0x00E9);

	CANObject<uint8_t, 1> obj_left_indicator_f(0x00C7);
	CANObject<uint8_t, 1> obj_left_indicator_r(0x00E7);

	CANObject<uint8_t, 1> obj_brake_light(0x00E5);

	CANObject<uint8_t, 1> obj_right_indicator_f(0x00C8);
	CANObject<uint8_t, 1> obj_right_indicator_r(0x00E8);

	CANObject<uint8_t, 1> obj_high_beam(0x00C6);
	// ...

	
	inline void Setup()
	{
		// system blocks
		set_block_info_params(obj_block_info);
		set_block_health_params(obj_block_health);
		set_block_features_params(obj_block_features);
		set_block_error_params(obj_block_error);

		// common blocks
		can_manager.RegisterObject(obj_block_info);
		can_manager.RegisterObject(obj_block_health);
		can_manager.RegisterObject(obj_block_features);
		can_manager.RegisterObject(obj_block_error);
		
		// Set versions data to block_info.
		obj_block_info.SetValue(0, (About::board_type << 3 | About::board_ver), CAN_TIMER_TYPE_NORMAL);
		obj_block_info.SetValue(1, (About::soft_ver << 2 | About::can_ver), CAN_TIMER_TYPE_NORMAL);
		
		return;
	}

	inline void Loop(uint32_t &current_time)
	{
		can_manager.Process(current_time);
		
		// Set uptime to block_info.
		static uint32_t iter = 0;
		if(current_time - iter > 1000)
		{
			iter = current_time;

			uint8_t *data = (uint8_t *)&current_time;
			obj_block_info.SetValue(2, data[0], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(3, data[1], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(4, data[2], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(5, data[3], CAN_TIMER_TYPE_NORMAL);
		}
		
		current_time = HAL_GetTick();

		return;
	}
}
