/*

*/

#pragma once

#include <inttypes.h>
#include <string.h>
#include <RingBuffer.h>

class CANLibLight
{
	static constexpr uint8_t _max_data_length = 7;
	static constexpr uint8_t _max_buffer_size = 8;
	static constexpr uint8_t _max_receive_func = 16;
	static constexpr uint8_t _ring_buffer_size = 16;
	
	using send_function_t = void (*)(uint16_t id, uint8_t *data_raw, uint8_t length_raw);
	using receive_function_t = void (*)(uint16_t id, uint8_t funcID, uint8_t *data, uint8_t length);
	
	struct frame_t
	{
		uint16_t id;
		union
		{
			uint8_t data_raw[_max_buffer_size];
			struct
			{
				uint8_t func_id;
				uint8_t data[_max_data_length];
			};
		};
		uint8_t length_raw;
	};
	
	public:
		
		CANLibLight(send_function_t func) : _can_send_func(func)
		{
			return;
		}

		/// @brief Вставка входящего CAN пакета в буфер.
		/// @param id 
		/// @param data_raw 
		/// @param length_raw 
		/// @return true в случае успеха.
		bool IncomingCANFrame(uint16_t id, uint8_t *data_raw, uint8_t length_raw)
		{
			frame_t frame;
			frame.id = id;
			memcpy(frame.data_raw, data_raw, length_raw);
			frame.length_raw = length_raw;
			
			return _rx_buffer.Write(frame);
		}

		/// @brief Привязка входящего CAN пакета к функции-обработчику.
		/// @param id ID параметра;
		/// @param func Указатель на функцию;
		/// @return true в случае успеха.
		bool RegRXCallback(uint16_t id, receive_function_t func)
		{
			if(_receive_functions_idx == _max_receive_func) return false;
			
			_receive_functions[_receive_functions_idx].id = id;
			_receive_functions[_receive_functions_idx].func = func;
			_receive_functions_idx++;
			
			return true;
		}

		/// @brief Отправка RAW пакета в CAN.
		/// @param id ID параметра;
		/// @param data_raw Данные пакета CAN;
		/// @param length_raw Длина данных пакета CAN;
		void SendRaw(uint16_t id, uint8_t *data_raw, uint8_t length_raw)
		{
			if(id > 0x07FF) return;
			if(length_raw > _max_buffer_size) return;
			
			_ClearTXBuffer();
			
			memcpy(_tx_buffer, data_raw, length_raw);
			_can_send_func(id, _tx_buffer, length_raw);
			
			return;
		}
		
		/// @brief Отправка пакета Set в CAN.
		/// @tparam T Тип отправляемой переменной;
		/// @param id ID параметра;
		/// @param value1 Отправляемая переменная;
		template <typename T> 
		void SendSet(uint16_t id, T value1)
		{
			if(id > 0x07FF) return;
			if(sizeof(value1) > _max_data_length) return;
			
			_ClearTXBuffer();

			_tx_buffer[0] = 0x01;
			memcpy( (_tx_buffer + 1), &value1, sizeof(value1) );
			_can_send_func( id, _tx_buffer, (sizeof(value1) + 1) );

			return;
		}

		/// @brief Отправка пакета Set в CAN.
		/// @tparam T Тип отправляемой переменной;
		/// @param id ID параметра;
		/// @param value1 Отправляемая переменная 1;
		/// @param value2 Отправляемая переменная 2;
		template <typename T> 
		void SendSet(uint16_t id, T value1, T value2)
		{
			if(id > 0x07FF) return;
			if(sizeof(value1) + sizeof(value2) > _max_data_length) return;
			
			_ClearTXBuffer();
			
			_tx_buffer[0] = 0x01;
			memcpy( (_tx_buffer + 1), &value1, sizeof(value1) );
			memcpy( (_tx_buffer + 1 + sizeof(value1)), &value2, sizeof(value2) );
			_can_send_func( id, _tx_buffer, (sizeof(T) * 2 + 1) );
			
			return;
		}
		
		/// @brief Обработка принятых CAN пакетов и вызод callback функций.
		/// @param time Текущее время;
		void Processing(uint32_t time)
		{
			if(time - time_last < 5) return;
			time_last = time;
			
			frame_t frame;
			while(_rx_buffer.IsEmpty() == false)
			{
				_rx_buffer.Read(frame);
				
				for(uint8_t i = 0; i < _max_receive_func; ++i)
				{
					if(_receive_functions[i].id == frame.id)
					{
						_receive_functions[i].func(frame.id, frame.data[0], &frame.data[1], frame.length_raw - 1);
						
						break;
					}
				}
			}

			return;
		}

	private:

		void _ClearTXBuffer()
		{
			memset(_tx_buffer, 0x00, _max_buffer_size);

			return;
		}

		send_function_t _can_send_func;

		uint8_t _tx_buffer[_max_buffer_size];

		RingBuffer<_ring_buffer_size, frame_t> _rx_buffer;

		uint32_t time_last = 0;

		struct
		{
			uint16_t id;
			receive_function_t func;
		} _receive_functions[_max_receive_func];
		uint8_t _receive_functions_idx = 0;
};
