#pragma once
#include <inttypes.h>
#include <PowerOut.h>

extern ADC_HandleTypeDef hadc1;

namespace Out
{
	
	/* Настройки */
	static constexpr uint8_t CFG_PortCount = 1;			// Кол-во портов управления.
	static constexpr uint32_t CFG_RefVoltage = 3300000;	// Опорное напряжение, микровольты.
	static constexpr uint8_t CFG_INA180_Gain = 20;		// Усиление микросхемы INA180.
	static constexpr uint8_t CFG_ShuntResistance = 5;	// Сопротивление шунта, миллиомы.
	/* */
	
	PowerOut<CFG_PortCount> obj(&hadc1, CFG_RefVoltage, CFG_INA180_Gain, CFG_ShuntResistance);
	
	
	void OnShortCircuit(uint8_t num, uint16_t current)
	{

	}
	
	inline void Setup()
	{
		obj.RegShortCircuitEvent(OnShortCircuit);
		obj.AddPort( {GPIOA, GPIO_PIN_2}, {GPIOA, GPIO_PIN_1, ADC_CHANNEL_1}, 0 );
		obj.Init();
		
		return;
	}
	
	inline void Loop(uint32_t &current_time)
	{
		current_time = HAL_GetTick();

		static uint32_t last = 0;
		if(current_time - last > 500)
		{
			last = current_time;
			
			obj.SetToggle(1);
		}
		
		return;
	}
}
