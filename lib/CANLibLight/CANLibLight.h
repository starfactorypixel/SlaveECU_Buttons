/*

*/

#pragma once

#include <inttypes.h>
#include <string.h>

class CANLibLight
{
	using can_send_function_t = void (*)(uint16_t id, uint8_t *data, uint8_t length);

	static constexpr uint8_t _max_data_length = 7;
	static constexpr uint8_t _max_buffer_size = 8;
	
	public:
		
		CANLibLight(can_send_function_t func) : _can_send_func(func)
		{
			_ClearTXBuffer();

			return;
		}

		void SendRaw(uint16_t id, uint8_t funcID, uint8_t *data, uint8_t length)
		{
			if(id > 0x07FF) return;
			if(length > _max_data_length) return;
			
			_ClearTXBuffer();
			
			_tx_buffer[0] = funcID;
			memcpy( (_tx_buffer) + 1, data, length );
			_can_send_func(id, _tx_buffer, length + 1);
			
			return;
		}
		
		template <typename T> 
		void SendSet8(uint16_t id, T value1)
		{
			if(id > 0x07FF) return;
			
			_ClearTXBuffer();

			_tx_buffer[0] = 0x01;
			memcpy( (_tx_buffer + 1), &value1, sizeof(value1) );
			_can_send_func( id, _tx_buffer, (sizeof(value1) + 1) );

			return;
		}

		template <typename T> 
		void SendSet8(uint16_t id, T value1, T value2)
		{
			if(id > 0x07FF) return;
			
			_ClearTXBuffer();
			
			_tx_buffer[0] = 0x01;
			memcpy( (_tx_buffer + 1), &value1, sizeof(value1) );
			memcpy( (_tx_buffer + 1 + sizeof(value1)), &value2, sizeof(value2) );
			_can_send_func( id, _tx_buffer, (sizeof(T) * 2 + 1) );
			
			return;
		}

	private:

		void _ClearTXBuffer()
		{
			memset(_tx_buffer, 0x00, _max_buffer_size);

			return;
		}

		can_send_function_t _can_send_func;

		uint8_t _tx_buffer[_max_buffer_size];
};
