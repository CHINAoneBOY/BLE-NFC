#include "lock_gpio.h"
#include "nrf_gpio.h"
#include "pca10028.h"
#include "low_power_pwm.h"




static low_power_pwm_t low_power_pwm_0;


static void pwm_handler(void * p_context)
{
	
}

void  beep_gpio_init(void)
{
	  uint32_t err_code;
    low_power_pwm_config_t low_power_pwm_config;

    APP_TIMER_DEF(lpp_timer_0);
    low_power_pwm_config.active_high    = false;
    low_power_pwm_config.period         = 50;
    low_power_pwm_config.bit_mask       = (1<<QK_BEEP);
    low_power_pwm_config.p_timer_id     = &lpp_timer_0;
    low_power_pwm_config.p_port         = NRF_GPIO;

    err_code = low_power_pwm_init((&low_power_pwm_0), &low_power_pwm_config, pwm_handler);
    APP_ERROR_CHECK(err_code);
    err_code = low_power_pwm_duty_set(&low_power_pwm_0, 20);
    APP_ERROR_CHECK(err_code);

    err_code = low_power_pwm_start((&low_power_pwm_0), low_power_pwm_0.bit_mask);
    APP_ERROR_CHECK(err_code);
		nrf_delay_ms(600);
	  low_power_pwm_stop((&low_power_pwm_0));
}


//void moto_gpio_init(void)
//{
//	
//}

//void irda_gpio_init(void)
//{
//	
//}

//void  led_gpio_init(void)
//{
//	nrf_gpio_cfg_output(LED1,GPIO_PIN_CNF_PULL_Pullup); //pwm
//}


void qk_lock_init(void)
{

	beep_gpio_init();
}

uint8_t   new_duty_cycle= 10;

void beep_test(void)
{
    low_power_pwm_start((&low_power_pwm_0), low_power_pwm_0.bit_mask);
		nrf_delay_ms(300);
	  low_power_pwm_stop((&low_power_pwm_0));
}
