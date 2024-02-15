#include "hoja_includes.h"
#include "interval.h"
#include "main.h"
#include "math.h"
#include "float.h"

#define B3 246.94f  // Frequency of B3 in Hz
#define E4 329.63f  // Frequency of E4 in Hz
#define A4 440.00f  // Frequency of A4 in Hz
#define D5 587.33f  // Frequency of D5 in Hz
#define Db5 554.37f // Frequency of Db5 in Hz
#define Ab4 415.30f // Frequency of Ab4 in Hz
#define Eb4 311.13f // Frequency of Eb4 in Hz
#define Fs4 369.99f // Frequency of Fs4 in Hz
#define Db4 277.18f // Frequency of Db4 in Hz
#define Ab3 207.65f // Frequency of Ab3 in Hz
#define D4 293.66f  // Frequency of D4 in Hz
#define G4 392.00f  // Frequency of G4 in Hz
#define C5 523.25f  // Frequency of C5 in Hz
#define F5 698.46f  // Frequency of F5 in Hz
#define Bb5 932.33f // Frequency of Bb5 in Hz
#define A5 880.00f  // Frequency of A5 in Hz
#define E5 659.26f  // Frequency of E5 in Hz
#define B4 493.88f  // Frequency of B4 in Hz
#define Ab5 830.61f // Frequency of Ab5 in Hz


float song[28] = {
    B3, E4, A4, E4, D5, E4, Db5, Ab4, Eb4, Ab4, Fs4, Db4, Ab3, Db4, D4, G4, C5, F5, Bb5, F5, C5, A5, E5, B4, Ab5, 0, 0, E4
    };

#define DRV2605_SLAVE_ADDR 0x5A

// LRA SDA Select (left/right)
#define GPIO_LRA_SDA_SEL    25
// LRA Audio Output
#define GPIO_LRA_IN         24

uint slice_num;

float _current_amplitude = 0;
float _current_frequency = 0;

void play_pwm_frequency(float frequency, float amplitude) {
    
    if(!amplitude) 
    {
        pwm_set_enabled(slice_num, false);
        return;
    }

    frequency = (frequency>1300) ? 1300 : frequency;
    frequency = (frequency<40) ? 40 : frequency;

    float plug = 100 * amplitude;

    uint32_t top =  1000000/frequency -1; // TOP is u16 has a max of 65535, being 65536 cycles
	pwm_set_wrap(slice_num, top);

	// set duty cycle
	uint16_t level = (top+1) * ((uint16_t) plug) / 100 -1; // calculate channel level from given duty cycle in %
	pwm_set_chan_level(slice_num, PWM_CHAN_A, level); 
	
	pwm_set_enabled(slice_num, true); // let's go!
}
bool fflip = false;
void cb_hoja_rumble_set(float frequency, float amplitude)
{
    _current_amplitude = amplitude;
    _current_frequency = frequency;

    play_pwm_frequency(_current_frequency, _current_amplitude);
}

#define RATED_VOLTAGE_HEX 0x4
#define ODCLAMP_HEX 0x54
#define HAPTIC_COMPENSATION 12
#define HAPTIC_BACKEMF 108
#define HAPTIC_BACKFEEDBACK 2

// Obtain and dump calibration values for auto-init LRA
void rumble_get_calibration()
{
    // Init GPIO for LRA switch
    gpio_init(GPIO_LRA_SDA_SEL);
    gpio_set_dir(GPIO_LRA_SDA_SEL, GPIO_OUT);
    gpio_put(GPIO_LRA_SDA_SEL, 1);

    // Perform auto-calibrate sequence
    sleep_ms(100);

    uint8_t _set_mode1[] = {0x01, 0x00};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode1, 2, false);

    sleep_ms(10);

    uint8_t _set_rated_voltage[] = {0x16, RATED_VOLTAGE_HEX};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_rated_voltage, 2, false);

    sleep_ms(10);

    uint8_t _set_odclamped[] = {0x17, ODCLAMP_HEX};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_odclamped, 2, false);

    sleep_ms(10);

    uint8_t _set_lra[] = {0x1A, 0xB6};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_lra, 2, false);

    sleep_ms(10);

    uint8_t _set_control1[] = {0x1B, 0xB3};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control1, 2, false);
    sleep_ms(10); 

    uint8_t _set_control2[] = {0x1C, 0xF5};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control2, 2, false);
    sleep_ms(10);

    uint8_t _set_control3[] = {0x1D, 0x80};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control3, 2, false);
    sleep_ms(10);

    uint8_t _set_mode[] = {0x01, 0x07};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode, 2, false);

    sleep_ms(10);
    
    uint8_t _set_autocalmem[] = {0x1E, 0x20};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_autocalmem, 2, false);
    sleep_ms(10);

    uint8_t _set_go[] = {0x0C, 0x01};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_go, 2, false);

    sleep_ms(600);

    // Get status
    uint8_t _get_calibrate_status[] = {0x0C};
    uint8_t _out[1] = {0};

    bool calibration_stopped = false;

    while(!calibration_stopped)
    {
        // Read status
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _get_calibrate_status, 1, true);
        i2c_read_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _out, 1, false);
        if(!(_out[0] & 0x01))
        {
            calibration_stopped = true;
        }
        sleep_ms(10);
    }

    // Check our diag result
    uint8_t _get_diag_bit[] = {0x00};
    uint8_t _diag_bit = 0;
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _get_diag_bit, 1, true);
    i2c_read_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, &_diag_bit, 1, false);

    // Read results and save later
    uint8_t auto_compensation = 0;
    uint8_t auto_back_emf = 0;
    uint8_t auto_feedback = 0;

    uint8_t _get_acalcomp[] = {0x18};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _get_acalcomp, 1, true);
    i2c_read_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, &auto_compensation, 1, false);

    uint8_t _get_acalbemf[] = {0x19};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _get_acalbemf, 1, true);
    i2c_read_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, &auto_back_emf, 1, false);

    uint8_t _get_bemfgain[] = {0x1A};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _get_bemfgain, 1, true);
    i2c_read_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, &auto_feedback, 1, false);

    // Isolate bemfgain bits
    auto_feedback &= 0b00000011;

    uint8_t out_dump[4] = {auto_compensation, auto_back_emf, auto_feedback, _diag_bit};

    webusb_send_debug_dump(4, out_dump);
}

bool played = false;
void test_sequence()
{
    if(played) return;
    played = true;
    for(int i = 0; i < 28; i+=1)
    {
        play_pwm_frequency(song[i], 0.5f);
        sleep_ms(150);
    }
    sleep_ms(150);
}

bool lra_init = false;
// Obtain and dump calibration values for auto-init LRA
void cb_hoja_rumble_init()
{
    if(lra_init) return;
    lra_init = true;

    // Init GPIO for LRA switch
    gpio_init(GPIO_LRA_SDA_SEL);
    gpio_set_dir(GPIO_LRA_SDA_SEL, GPIO_OUT);
    gpio_put(GPIO_LRA_SDA_SEL, 1);
    
    // Set PWM function
    gpio_set_function(GPIO_LRA_IN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to our GPIO pin
    slice_num = pwm_gpio_to_slice_num(GPIO_LRA_IN);

    float divider = 125000000.0f / 1000000;

    // Set the PWM clock divider to run at 1Khz
    pwm_set_clkdiv(slice_num, divider);

    //play_pwm_frequency(0);

    for(uint set = 0; set<2; set++)
    {
        // Get out of standbyu
        //uint8_t _set_mode1[] = {0x01, 0x00};
        //i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode1, 2, false);

        uint8_t _set_rated_voltage[] = {0x16, RATED_VOLTAGE_HEX};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_rated_voltage, 2, false);

        uint8_t _set_odclamped[] = {0x17, ODCLAMP_HEX};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_odclamped, 2, false);

        uint8_t _set_lra[] = {0x1A, 0x9E};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_lra, 2, false);

        uint8_t _set_control1[] = {0x1B, 0x2F};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control1, 2, false);

        uint8_t _set_control2[] = {0x1C, 0xFF};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control2, 2, false);

        uint8_t _set_control3[] = {0x1D, 0x43};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control3, 2, false);

        uint8_t _set_compensation[] = {0x18, HAPTIC_COMPENSATION};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_compensation, 2, false);

        uint8_t _set_bemf[] = {0x19, HAPTIC_BACKEMF};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_bemf, 2, false);

        /*
        uint8_t _set_audio1[] = {0x11, 0x05};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_audio1, 2, false);

        uint8_t _set_audio2[] = {0x12, 0x19};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_audio2, 2, false);

        uint8_t _set_audio3[] = {0x13, 0xFF};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_audio3, 2, false);

        // Minimum output drive
        uint8_t _set_audio4[] = {0x14, 0x19};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_audio4, 2, false);

        // Max output drive
        uint8_t _set_audio5[] = {0x15, 0x19};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_audio5, 2, false);
        */

        uint8_t _set_mode[] = {0x01, 0x03};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode, 2, false);
        gpio_put(GPIO_LRA_SDA_SEL, 0);
    }

}

bool app_rumble_hwtest()
{
    /*
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
    return _left && _right;*/
    /*
    play_pwm_frequency(100, 1);
    float frequency = 40.0f;
    for(uint16_t i = 0; i < 1000; i++)
    {
        play_pwm_frequency(frequency, 1);
        sleep_ms(10);
        frequency+=1;
    }
    play_pwm_frequency(0, 0);
    sleep_ms(500);*/

    return true;
}

void app_rumble_task(uint32_t timestamp)
{
    static interval_s interval = {0};
    if(interval_run(timestamp, 8000, &interval))
    {
        return;
        if(_current_amplitude>0)
        {
            //_current_amplitude -= 0.015f;
            //_current_amplitude = (_current_amplitude<0) ? 0 : _current_amplitude;
            play_pwm_frequency(_current_frequency, _current_amplitude);
            
        }
    }
}