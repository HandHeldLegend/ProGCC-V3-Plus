#include "hoja_includes.h"
#include "interval.h"
#include "main.h"

#define DRV2605_SLAVE_ADDR 0x58

// LRA SDA Select (left/right)
#define GPIO_LRA_SDA_SEL    25
// LRA Audio Output
#define GPIO_LRA_IN         24

uint slice_num;

void play_pwm_frequency(float frequency) {
    
    uint32_t top =  1000000/frequency -1; // TOP is u16 has a max of 65535, being 65536 cycles
	pwm_set_wrap(slice_num, top);

	// set duty cycle
	uint16_t level = (top+1) * 55 / 100 -1; // calculate channel level from given duty cycle in %
	pwm_set_chan_level(slice_num, PWM_CHAN_A, level); 
	
	pwm_set_enabled(slice_num, true); // let's go!
}

void cb_hoja_rumble_set(float frequency, float amplitude)
{
    (void) amplitude;
    frequency = (frequency>1252) ? 1252 : frequency;
    frequency = (frequency<0) ? 0 : frequency;

    play_pwm_frequency(frequency);
}

void cb_hoja_rumble_init()
{
    // Init GPIO for LRA switch
    gpio_init(GPIO_LRA_SDA_SEL);
    gpio_set_dir(GPIO_LRA_SDA_SEL, GPIO_OUT);
    gpio_put(GPIO_LRA_SDA_SEL, 0);

    // Set PWM function
    gpio_set_function(GPIO_LRA_IN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to our GPIO pin
    slice_num = pwm_gpio_to_slice_num(GPIO_LRA_IN);

    float divider = 125000000.0f / 1000000;

    // Set the PWM clock divider to run at 1Khz
    pwm_set_clkdiv(slice_num, divider);

    play_pwm_frequency(0);

    // Set LRA drivers to proper mode
    // Wake from standby
    // 0x01 - Mode register
    // 0x00 Remove from standby
    uint8_t _wake[] = {0x01, 0x00};
    //i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _wake, 2, false);

}

bool app_rumble_hwtest()
{
    // Check LRA status on both
    bool _left  = false;
    bool _right = false;

    gpio_put(GPIO_LRA_SDA_SEL, 0);
    sleep_ms(100);
    uint8_t _status[] = {0x00};
    uint8_t _status_out[1] = {0};

    i2c_write_timeout_us(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _status, 1, true, 100000);
    int read_left = i2c_read_timeout_us(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _status_out, 1, false, 100000);

    if (_status_out[0] > 0) _left = true;

    _status_out[0] = 0;

    gpio_put(GPIO_LRA_SDA_SEL, 1);
    sleep_ms(100);

    i2c_write_timeout_us(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _status, 1, true, 100000);
    int read_right = i2c_read_timeout_us(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _status_out, 1, false, 100000);

    if (_status_out[0] > 0) _right = true;

    gpio_put(GPIO_LRA_SDA_SEL, 0);
    return _left && _right;
}

void app_rumble_task(uint32_t timestamp)
{

}