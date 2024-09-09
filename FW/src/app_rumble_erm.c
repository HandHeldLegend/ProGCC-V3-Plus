#include "hoja_includes.h"
#include "interval.h"
#include "main.h"

uint main_slice_num = 0;
uint brake_slice_num = 0;

const uint32_t _rumble_interval = 8000;

int _rumble_cap = 0;

int _rumble_floor = 0;

int _rumble_min = 0;
int _rumble_current = 0;

#define RUMBLE_MAX 100
#define RUMBLE_MAX_ADD 50
static uint8_t _rumble_max = RUMBLE_MAX;

static bool _declining = false;

void app_rumble_output()
{
    if (_rumble_current < _rumble_cap)
    {
        _rumble_current += 10;
    }
    else
    {
        _rumble_current -= 10;
        
        if (_rumble_current <= _rumble_min)
        {
            _rumble_current = _rumble_min; 
        }

        _rumble_cap = _rumble_current;
    }

    pwm_set_gpio_level(PGPIO_RUMBLE_BRAKE, (!_rumble_current) ? 255 : 0);
    pwm_set_gpio_level(PGPIO_RUMBLE_MAIN, (_rumble_current > 0) ? _rumble_current : 0);
}

void app_rumble_set(float amphi, float amplo)
{
    if(!_rumble_floor)
    return;
    
    float amp1 = (amphi > amplo) ? amphi : amplo;

    // Clamp rumble
    if(amp1>0)
    {
        const float range = 0.75f;
        amp1 *= 0.75f;
        amp1 += 0.15f;
    }
    
    float p = ((float) RUMBLE_MAX_ADD * amp1);
    uint16_t tmp = (uint16_t) p;


    if(amp1 > 0)
    {
        tmp += _rumble_floor;
    }

    _rumble_min = tmp;

    if(tmp>_rumble_cap)
    {
        _rumble_cap = tmp;
    }
}


bool testing = false;

void app_rumble_task(uint32_t timestamp)
{
    static interval_s interval = {0};
    static amfm_s amfm = {0};

    if(interval_run(timestamp, _rumble_interval, &interval))
    {
        if(haptics_l_get(&amfm))
        {
            app_rumble_set(amfm.a_hi, amfm.a_lo);
        }

        app_rumble_output();
    }
}


void cb_hoja_rumble_test()
{
    app_rumble_set(0, 1);

    for(int i = 0; i < 62; i++)
    {   
        app_rumble_output();
        watchdog_update();
        sleep_ms(8);
    }

    app_rumble_set(0, 0);
    
    app_rumble_output();
}

bool app_rumble_hwtest()
{
    cb_hoja_rumble_test();
    return true;
}

void cb_hoja_rumble_init()
{   
    // Set up Rumble GPIO
    gpio_init(PGPIO_RUMBLE_MAIN);
    gpio_init(PGPIO_RUMBLE_BRAKE);

    gpio_set_dir(PGPIO_RUMBLE_MAIN, GPIO_OUT);
    gpio_set_dir(PGPIO_RUMBLE_BRAKE, GPIO_OUT);

    gpio_set_function(PGPIO_RUMBLE_MAIN, GPIO_FUNC_PWM);
    gpio_set_function(PGPIO_RUMBLE_BRAKE, GPIO_FUNC_PWM);

    main_slice_num = pwm_gpio_to_slice_num(PGPIO_RUMBLE_MAIN);
    brake_slice_num = pwm_gpio_to_slice_num(PGPIO_RUMBLE_BRAKE);

    pwm_set_wrap(main_slice_num, 255);
    pwm_set_wrap(brake_slice_num, 255);

    pwm_set_chan_level(main_slice_num, PWM_CHAN_B, 0);    // B for odd pins
    pwm_set_chan_level(brake_slice_num, PWM_CHAN_B, 255); // B for odd pins

    pwm_set_enabled(main_slice_num, true);
    pwm_set_enabled(brake_slice_num, true);

    pwm_set_gpio_level(PGPIO_RUMBLE_BRAKE, 255);
    pwm_set_gpio_level(PGPIO_RUMBLE_MAIN, 0);

    sleep_us(150); // Stabilize voltages

    uint8_t fl;
    uint8_t max;
    rumble_type_t type;
    hoja_get_rumble_settings(&fl, &type);

    _rumble_floor = fl;
    _rumble_max   = (fl+RUMBLE_MAX_ADD) > 255 ? 255 : fl+RUMBLE_MAX_ADD;

    if(!_rumble_floor)
    {
        _rumble_floor = 0;
        _rumble_max = 0;
    }
    

    sleep_ms(350);

}
