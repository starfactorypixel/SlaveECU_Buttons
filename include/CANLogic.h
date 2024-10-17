#pragma once
#include <CANLibrary.h>

extern CAN_HandleTypeDef hcan;
extern void HAL_CAN_Send(uint16_t id, uint8_t *data_raw, uint8_t length_raw);

namespace CANLib
{
	EasyPinD can_rs(GPIOA, {GPIO_PIN_15, GPIO_MODE_OUTPUT_OD, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW}, GPIO_PIN_SET);


	//*********************************************************************
	// CAN Library settings
	//*********************************************************************

	/// @brief Number of CANObjects in CANManager
	static constexpr uint8_t CFG_CANObjectsCount = 23;

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

/*
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
*/

	
	
	// --------------------------------------------------------------------------------------------
	
	// 0x0124 .. 0x0133
	// set | event
	// uint8_t	bitmask	1 + X	{ type[0] data[1] }
	// Кнопка / светодиод 01 .. 16
	CANObject<uint8_t, 1> obj_buttonled_01(0x0124);
	CANObject<uint8_t, 1> obj_buttonled_02(0x0125);
	CANObject<uint8_t, 1> obj_buttonled_03(0x0126);
	CANObject<uint8_t, 1> obj_buttonled_04(0x0127);
	CANObject<uint8_t, 1> obj_buttonled_05(0x0128);
	CANObject<uint8_t, 1> obj_buttonled_06(0x0129);
	CANObject<uint8_t, 1> obj_buttonled_07(0x012A);
	CANObject<uint8_t, 1> obj_buttonled_08(0x012B);
	CANObject<uint8_t, 1> obj_buttonled_09(0x012C);
	CANObject<uint8_t, 1> obj_buttonled_10(0x012D);
	CANObject<uint8_t, 1> obj_buttonled_11(0x012E);
	CANObject<uint8_t, 1> obj_buttonled_12(0x012F);
	CANObject<uint8_t, 1> obj_buttonled_13(0x0130);
	CANObject<uint8_t, 1> obj_buttonled_14(0x0131);
	CANObject<uint8_t, 1> obj_buttonled_15(0x0132);
	CANObject<uint8_t, 1> obj_buttonled_16(0x0133);

	// 0x0134 .. 0x0135
	// event
	// uint8_t	bitmask	1 + X	{ type[0] data[1] }
	// Подрулевой переключатель 1 .. 2
	CANObject<uint8_t, 1> obj_switch_1(0x0134);
	CANObject<uint8_t, 1> obj_switch_2(0x0135);


	CANObject<uint8_t, 2> obj_button_action(0x0789);
	
	// --------------------------------------------------------------------------------------------

	void HardwareSetup()
	{
		can_rs.Init();
		
		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
		HAL_CAN_Start(&hcan);
	}
	
	inline void Setup()
	{
		HardwareSetup();
		
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

		can_manager.RegisterObject(obj_buttonled_01);
		can_manager.RegisterObject(obj_buttonled_02);
		can_manager.RegisterObject(obj_buttonled_03);
		can_manager.RegisterObject(obj_buttonled_04);
		can_manager.RegisterObject(obj_buttonled_05);
		can_manager.RegisterObject(obj_buttonled_06);
		can_manager.RegisterObject(obj_buttonled_07);
		can_manager.RegisterObject(obj_buttonled_08);
		can_manager.RegisterObject(obj_buttonled_09);
		can_manager.RegisterObject(obj_buttonled_10);
		can_manager.RegisterObject(obj_buttonled_11);
		can_manager.RegisterObject(obj_buttonled_12);
		can_manager.RegisterObject(obj_buttonled_13);
		can_manager.RegisterObject(obj_buttonled_14);
		can_manager.RegisterObject(obj_buttonled_15);
		can_manager.RegisterObject(obj_buttonled_16);
		can_manager.RegisterObject(obj_switch_1);
		can_manager.RegisterObject(obj_switch_2);
		
		can_manager.RegisterObject(obj_button_action);
		
		
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
		if(current_time - iter > 1500)
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
