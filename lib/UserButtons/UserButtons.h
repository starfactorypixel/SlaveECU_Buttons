/*

*/

#pragma once

#include <inttypes.h>
#include <string.h>

template <uint8_t _bytes_count> 
class UserButtons
{
	using func_spi_rw_t = void (*)(uint8_t *buttons, uint8_t *leds, uint8_t length);
	using func_change_t = void (*)(uint8_t port, bool state);
	
	public:
		
		enum mode_t : uint8_t { MODE_NORMAL, MODE_TRIGGER };
		
		UserButtons() = delete;
		
		UserButtons(func_spi_rw_t func, bool use_init = true) : _func_spi_rw(func), _init(use_init)
		{
			memset(_btn_mode, MODE_NORMAL, sizeof(_btn_mode));
			memset(_btn_state_old, 0xFF, sizeof(_btn_state_old));
			memset(_btn_state_new, 0xFF, sizeof(_btn_state_new));
			memset(_btn_state_trig, false, sizeof(_btn_state_trig));
			memset(_led_state_new, 0x00, sizeof(_led_state_new));
			
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
			
			return !((_btn_state_new[(idx / 8)] >> (idx % 8)) & 0x01);
		}
		
		bool GetLedState(uint8_t led)
		{
			if(led == 0 || led > (_bytes_count * 8)) return false;
			uint8_t idx = led - 1;
			
			return ((_led_state_new[(idx / 8)] >> (idx % 8)) & 0x01);
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
				_led_state_new[(idx / 8)] |= (1 << (idx % 8));
			else
				_led_state_new[(idx / 8)] &= ~(1 << (idx % 8));
			
			return;
		}
		
		void SetButtonMode(uint8_t led, mode_t mode)
		{
			if(led == 0 || led > (_bytes_count * 8)) return;
			uint8_t idx = led - 1;
			
			_btn_mode[idx] = mode;
			
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
					bool bit_old = (_btn_state_old[(i / 8)] >> (i % 8)) & 0x01;
					bool bit_new = (_btn_state_new[(i / 8)] >> (i % 8)) & 0x01;

					if( _btn_mode[i] == MODE_NORMAL || _init == true )
					{
						if( bit_old != bit_new || _init == true )
						{
							_func_change( (i + 1), !bit_new );
						}
					}
					else if( _btn_mode[i] == MODE_TRIGGER )
					{
						if( (bit_new == false && bit_old != bit_new) )
						{
							_btn_state_trig[i] = !_btn_state_trig[i];
							
							_func_change( (i + 1), _btn_state_trig[i] );
						}
					}
				}
				
				_init = false;
			}
			
			return;
		}

	private:

		void _HW_SPI_RW_Offload()
		{
			memcpy(_btn_state_old, _btn_state_new, _bytes_count);
			
			// Реверсировать  _led_state_new ?
			_func_spi_rw(_btn_state_new, _led_state_new, _bytes_count);
			
			return;
		}
		
		uint32_t _time_last_update = 0;
		
		func_spi_rw_t _func_spi_rw = nullptr;
		func_change_t _func_change = nullptr;
		
		mode_t _btn_mode[_bytes_count * 8];			// Режим кнопки, нормальная или триггерная.
		uint8_t _btn_state_old[_bytes_count];		// Состояние кнопк до считывания.
		uint8_t _btn_state_new[_bytes_count];		// Состояние кнопки после считывания.
		uint8_t _btn_state_trig[_bytes_count * 8];	// Состояние кнопки в режиме триггера.
		uint8_t _led_state_new[_bytes_count];		// Состояние светодиодов.

		bool _init;									// Костыль, который заставит все кнопки бросить своё состояние при инициализации.

};
