#pragma once
#include <inttypes.h>
#include <CanObjectBase.h>
#include <CanObj/IButtonObjSender.hpp>

// Данный объект реализует обработку кнопок: set, event

class CanButtonObj : public CANObjectBase, public IButtonObjSender
{
	struct __attribute__((packed)) set_t { uint8_t fId; uint8_t btn; uint8_t state; };
	struct __attribute__((packed)) event_ok_t { uint8_t fId = CAN_FUNC_EVENT_OK; uint8_t btn; uint8_t state; };
	
	using function_set_t = void (*)(uint8_t btn, uint8_t state);

	public:
		CanButtonObj(can_object_id_t id, function_set_t ctrl) : CANObjectBase(id), _FuncSet(ctrl)
		{
			return;
		}

		virtual void EventOk(uint8_t btn, uint8_t state) override
		{
			event_ok_t answer = {};
			answer.btn = btn;
			answer.state = state;
			this->SendFrame((uint8_t *)&answer, sizeof(answer));
			
			return;
		}
		
	protected:
		virtual void OnTick(uint32_t time) noexcept override
		{
			return;
		}

		virtual void OnProcessFrame(can_frame_t &can_frame) noexcept override
		{
			uint8_t fId = can_frame.raw_data[0];
			switch(fId)
			{
				case CAN_FUNC_SET_IN:
				{
					set_t *obj = (set_t *)can_frame.raw_data;
					_FuncSet(obj->btn, obj->state);
					
					break;
				}
			}
			
			return;
		}
		
		virtual void OnTimer() noexcept override
		{
			return;
		}

	private:
		function_set_t _FuncSet;
};
