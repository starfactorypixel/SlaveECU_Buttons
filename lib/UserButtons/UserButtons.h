/*

*/

#pragma once

#include <inttypes.h>

template <uint8_t _bytes_count> 
class UserButtons
{
	using func_spi_rw_t = void (*)(uint8_t *buttons, uint8_t *leds, uint8_t length);
	using func_change_t = void (*)(uint8_t port, bool state);
	
	public:

		enum mode_t : uint8_t { MODE_NORMAL, MODE_TRIGGER };
		
		UserButtons() = delete;
		
		UserButtons(func_spi_rw_t func) : _func_spi_rw(func)
		{
			return;
		}
		
		void RegChangeFunction(func_change_t func)
		{
			_func_change = func;
			
			return;
		}

		bool GetButtonState(uint8_t button)
		{
			if(button == 0 || button > (_bytes_count * 8)) return false;
			uint8_t idx = button - 1;
			
			return !((_buttons_state_curent[(idx / 8)] >> (idx % 8)) & 0x01);
		}
		
		bool GetLedState(uint8_t led)
		{
			if(led == 0 || led > (_bytes_count * 8)) return false;
			uint8_t idx = led - 1;
			
			return ((_leds_state_curent[(idx / 8)] >> (idx % 8)) & 0x01);
		}

/*
		void SetButtonState(uint8_t button, bool state)
		{
			if(button == 0 || button > (_bytes_count * 8)) return;
			uint8_t idx = button - 1;
		}
*/
		
		void SetLedState(uint8_t led, bool state)
		{
			if(led == 0 || led > (_bytes_count * 8)) return;
			uint8_t idx = led - 1;

			if(state == true)
				_leds_state_curent[(idx / 8)] |= (1 << (idx % 8));
			else
				_leds_state_curent[(idx / 8)] &= ~(1 << (idx % 8));
			
			return;
		}
		
		void SetButtonMode(uint8_t led, mode_t mode)
		{
			if(led == 0 || led > (_bytes_count * 8)) return;
			uint8_t idx = led - 1;
			
			_buttons_mode[idx] = mode;
			
			return;
		}
		
		void Processing(uint32_t current_time)
		{
			if(current_time - _time_last_update >= 25)
			{
				_time_last_update = current_time;
				
				_HW_SPI_RW_Offload();

				for(uint8_t i = 0; i < (_bytes_count * 8); ++i)
				{
					bool bit_old = (_buttons_state_old[(i / 8)] >> (i % 8)) & 0x01;
					bool bit_new = (_buttons_state_curent[(i / 8)] >> (i % 8)) & 0x01;
					
					if( _buttons_mode[i] == MODE_NORMAL )
					{
						if( bit_old != bit_new )
						{
							_func_change( (i + 1), !bit_new );
						}
					}
					else if( _buttons_mode[i] == MODE_TRIGGER )
					{
						if( bit_new == false && bit_old != bit_new)
						{
							_buttons_state[i] = !_buttons_state[i];
							
							_func_change( (i + 1), _buttons_state[i] );
						}
					}
				}
			}
			
			return;
		}

	private:

		void _HW_SPI_RW_Offload()
		{
			memcpy(_buttons_state_old, _buttons_state_curent, _bytes_count);
			
			// Реверсировать  _leds_state_curent ?
			_func_spi_rw(_buttons_state_curent, _leds_state_curent, _bytes_count);
			
			return;
		}
		
		func_spi_rw_t _func_spi_rw = nullptr;
		func_change_t _func_change = nullptr;

		uint32_t _time_last_update = 0;

		uint8_t _buttons_state_old[_bytes_count];
		uint8_t _buttons_state_curent[_bytes_count];
		mode_t _buttons_mode[_bytes_count * 8] = { MODE_NORMAL };
		bool _buttons_state[_bytes_count * 8] = { false };

		//uint8_t _state_leds_old[_bytes_count];
		uint8_t _leds_state_curent[_bytes_count];
};
