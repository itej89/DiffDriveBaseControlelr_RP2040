#include "hardware/pwm.h"

#include "hardware/structs/pwm.h"

uint32_t pwm_set_freq_duty(uint slice_num,
       uint chan,uint32_t f, int d)
{
        uint32_t clock = 125000000;
        uint32_t divider16 = clock / f / 4096 + 
                                (clock % (f * 4096) != 0);
        if (divider16 / 16 == 0)
        divider16 = 16;
        uint32_t wrap = clock * 16 / divider16 / f - 1;
        pwm_set_clkdiv_int_frac(slice_num, divider16/16,
                                        divider16 & 0xF);
        pwm_set_wrap(slice_num, wrap);
        pwm_set_chan_level(slice_num, chan, wrap * d / 100);
        return wrap;
}

void analogWrite(uint8_t pin_num, uint32_t frequency , uint32_t duty_cycle){
        // printf("\nDuty %d %d", frequency, duty_cycle);
        uint slice_num = pwm_gpio_to_slice_num(pin_num);
        uint chan = pwm_gpio_to_channel(pin_num);
        pwm_set_freq_duty(slice_num, chan, frequency, duty_cycle);
}

void analogInit(uint8_t pin_num, uint32_t frequency , uint32_t duty_cycle){
        gpio_set_function(pin_num, GPIO_FUNC_PWM);
        uint slice_num = pwm_gpio_to_slice_num(pin_num);
        uint chan = pwm_gpio_to_channel(pin_num);
        pwm_set_freq_duty(slice_num, chan, frequency, duty_cycle);
        pwm_set_enabled(slice_num, true);
}