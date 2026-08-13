#pragma once
#include <inttypes.h>

class IButtonObjSender
{
	public:
		virtual ~IButtonObjSender() = default;

		virtual void EventOk(uint8_t btn, uint8_t state) = 0;
};
