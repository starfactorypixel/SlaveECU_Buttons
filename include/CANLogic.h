#pragma once
#include <DrakePinD.hpp>
#include <CANLibrary.h>

extern CAN_HandleTypeDef hcan;
extern void HAL_CAN_Send(uint16_t id, uint8_t *data_raw, uint8_t length_raw);

namespace CANLib
{
	static constexpr uint8_t CFG_CANObjectsCount = 8;
	static constexpr uint8_t CFG_CANFrameBufferSize = 16;
	static constexpr uint16_t CFG_CANFirstId = 0x0220;
	
	DrakePinD can_rs({GPIOA, GPIO_PIN_15}, DrakePin::OutputOpenDrain, DrakePin::High);
	
	CANManager<CFG_CANObjectsCount, CFG_CANFrameBufferSize> can_manager(&HAL_CAN_Send);
	
	CANObject<uint8_t, 7> obj_block_info(CFG_CANFirstId + 0);
	CANObject<uint8_t, 7> obj_block_health(CFG_CANFirstId + 1);
	CANObject<uint8_t, 7> obj_block_features(CFG_CANFirstId + 2);
	CANObject<uint8_t, 7> obj_block_error(CFG_CANFirstId + 3);
	
	CANObject<uint8_t, 2> obj_buttonled_cn2(CFG_CANFirstId + 4);
	CANObject<uint8_t, 2> obj_buttonled_cn3(CFG_CANFirstId + 5);
	CANObject<uint8_t, 2> obj_buttonled_cn4(CFG_CANFirstId + 6);
	CANObject<uint8_t, 2> obj_buttonled_cn5(CFG_CANFirstId + 7);
	
	
	void CAN_Enable()
	{
		HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
		HAL_CAN_Start(&hcan);
		
		can_rs.Off();
		
		return;
	}
	
	void CAN_Disable()
	{
		HAL_CAN_DeactivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);
		HAL_CAN_Stop(&hcan);
		
		can_rs.On();
		
		return;
	}
	
	inline void Setup()
	{
		can_rs.Init();
		
		set_block_info_params(obj_block_info);
		set_block_health_params(obj_block_health);
		set_block_features_params(obj_block_features);
		set_block_error_params(obj_block_error);
		
		can_manager.RegisterObject(obj_block_info);
		can_manager.RegisterObject(obj_block_health);
		can_manager.RegisterObject(obj_block_features);
		can_manager.RegisterObject(obj_block_error);
		
		can_manager.RegisterObject(obj_buttonled_cn2);
		can_manager.RegisterObject(obj_buttonled_cn3);
		can_manager.RegisterObject(obj_buttonled_cn4);
		can_manager.RegisterObject(obj_buttonled_cn5);
		
		
		// Передача версий и типов в объект block_info
		obj_block_info.SetValue(0, (About::board_type << 3 | About::board_ver), CAN_TIMER_TYPE_NORMAL);
		obj_block_info.SetValue(1, (About::soft_ver << 2 | About::can_ver), CAN_TIMER_TYPE_NORMAL);

		CAN_Enable();
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		can_manager.Process(current_time);

		// Передача UpTime блока в объект block_info
		static uint32_t iter1000 = 0;
		if(current_time - iter1000 > 1000)
		{
			iter1000 = current_time;
			
			uint8_t *data = (uint8_t *)&current_time;
			obj_block_info.SetValue(2, data[0], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(3, data[1], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(4, data[2], CAN_TIMER_TYPE_NORMAL);
			obj_block_info.SetValue(5, data[3], CAN_TIMER_TYPE_NORMAL);
		}
		
		// При выходе обновляем время
		current_time = HAL_GetTick();
		
		return;
	}
}
