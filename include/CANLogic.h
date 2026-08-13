#pragma once
#include <CANLibrary.h>
#include "CanObj/CanBlockInfo.hpp"
#include "CanObj/CanBlockCfg.hpp"
#include "CanObj/CanButtonObj.hpp"
#include "CANFunc.h"
#include <DrakePinD.hpp>

extern CAN_HandleTypeDef hcan;
extern bool HAL_CAN_Send(can_object_id_t id, uint8_t *data, uint8_t length);

namespace CANLib
{
	static constexpr uint8_t CFG_CANObjectsCount = 6;
	static constexpr uint16_t CAN_BASE_ID = 0x0220;
	
	DrakePinD can_rs({GPIOA, GPIO_PIN_15}, DrakePin::OutputOpenDrain, DrakePin::High);
	
	CANManager<CFG_CANObjectsCount> can_manager(&HAL_CAN_Send, &HAL_GetTick, &OnInterruptCtrl);
	
	CanBlockInfo obj_block_info(CAN_BASE_ID+0, OnStaticInfoReq, OnDynamicInfoReq);
	CanBlockCfg obj_block_cfg(CAN_BASE_ID+1, OnCfgSaveReset, block_cfg_table, block_cfg_table_count);

	CanButtonObj obj_buttonled_1(CAN_BASE_ID+4, ButtonsLeds::OnButtonSet);
	CanButtonObj obj_buttonled_2(CAN_BASE_ID+5, ButtonsLeds::OnButtonSet);
	CanButtonObj obj_buttonled_3(CAN_BASE_ID+6, ButtonsLeds::OnButtonSet);
	CanButtonObj obj_buttonled_4(CAN_BASE_ID+7, ButtonsLeds::OnButtonSet);
	
	
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

		can_manager.AddObject(obj_block_info);
		can_manager.AddObject(obj_block_cfg);
		
		can_manager.AddObject(obj_buttonled_1);
		can_manager.AddObject(obj_buttonled_2);
		can_manager.AddObject(obj_buttonled_3);
		can_manager.AddObject(obj_buttonled_4);

		CAN_Enable();
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		can_manager.Processing();
		
		current_time = HAL_GetTick();
		return;
	}
}

IBlockInfoSender &BlockInfoSender = CANLib::obj_block_info;
IButtonObjSender &ButtonLed1 = CANLib::obj_buttonled_1;
IButtonObjSender &ButtonLed2 = CANLib::obj_buttonled_2;
IButtonObjSender &ButtonLed3 = CANLib::obj_buttonled_3;
IButtonObjSender &ButtonLed4 = CANLib::obj_buttonled_4;
