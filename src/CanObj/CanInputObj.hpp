#pragma once
#include <inttypes.h>
#include <CanObjectBase.h>

// Данный объект реализует информационный параметр, который отправляется по request, timer, event

template <typename T, uint8_t N = 1> 
class CanInputObj : public CANObjectBase
{
	//using T = uint16_t; static constexpr uint8_t N = 1;
	struct __attribute__((packed)) request_t { uint8_t fId; };
	struct __attribute__((packed)) timer_t { uint8_t fId = CAN_FUNC_TIMER_NORMAL; T val[N]; };
	struct __attribute__((packed)) event_ok_t { uint8_t fId = CAN_FUNC_EVENT_OK; T val[N]; };
	
	using function_classifier_t = uint8_t (*)(uint8_t idx, const T value);
	
	public:
		template <typename ...Args> 
		CanInputObj(can_object_id_t id, uint16_t timer, Args ...args) : CANObjectBase(id), _value_ptr{args...}
		{
			static_assert(sizeof...(Args) == N, "Wrong number of parameters");
			
			this->SetTimerPeriod(timer);
			
			return;
		}

		void SetValueClassifier(function_classifier_t function)
		{
			_FuncClassifier = function;
			
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
				case CAN_FUNC_REQUEST_IN:
				{
					//request_t *obj = (request_t *)can_frame.raw_data;
					
					event_ok_t answer = {};
					for(uint8_t i = 0; i < N; ++i)
					{
						
						answer.val[i] = Analog::VoltCalcInput.GetmV(*_value_ptr[i]);
					}
					this->SendFrame((uint8_t *)&answer, sizeof(answer));
					
					break;
				}
			}

			return;
		}

		virtual void OnTimer() noexcept override
		{
			timer_t answer = {};
			for(uint8_t i = 0; i < N; ++i)
			{
				const T val = Analog::VoltCalcInput.GetmV(*_value_ptr[i]);
				uint8_t fId = _FuncClassifier(i, val);
				if(fId > answer.fId)
					answer.fId = fId;
				answer.val[i] = val;
			}
			this->SendFrame((uint8_t *)&answer, sizeof(answer));

			return;
		}
		
	private:
		const T *_value_ptr[N];
		function_classifier_t _FuncClassifier = [](uint8_t idx, const T value) -> uint8_t { return CAN_FUNC_TIMER_NORMAL; };
};
