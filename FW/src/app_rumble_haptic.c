#include "hoja_includes.h"
#include "interval.h"
#include "main.h"
#include "math.h"
#include "float.h"

#define PWM_CLOCK_BASE 2000000 // 2Mhz
#define PWM_CLOCK_MULTIPLIER 1000 // Scale back up to where we need it

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

#if ( (HOJA_DEVICE_ID == 0xA002) || (HOJA_DEVICE_ID == 0xA003) )
    // LRA SDA Select (left/right)
    #define GPIO_LRA_SDA_SEL    25
    // LRA Audio Output
    #define GPIO_LRA_IN         24
#elif (HOJA_DEVICE_ID == 0xA004)
    // LRA Audio Output
    #define GPIO_LRA_IN         21 // Low
    #define GPIO_LRA_IN_HI      24
#endif

uint slice_num;

float _current_amplitude = 0;
float _current_frequency = 0;

#define RATED_VOLTAGE_HEX 0x3
#define RATED_VOLTAGE_REGISTER 0x16

#define HAPTIC_COMPENSATION 0
#define HAPTIC_BACKEMF 0


#define BLANKING_TIME_BASE (1)//(1)
#define IDISS_TIME_BASE (1)

#define DRIVE_TIME (26)
#define N_ERM_LRA (1 << 7)
#define FB_BRAKE_FACTOR (7 << 4) // Disable brake
#define LOOP_GAIN (1U << 2)
#define HAPTIC_BEMF_GAIN 2
// Write to register 0x1A
#define FEEDBACK_CTRL_BYTE (N_ERM_LRA | FB_BRAKE_FACTOR | LOOP_GAIN | HAPTIC_BEMF_GAIN)
#define FEEDBACK_CTRL_REGISTER 0x1A

// CTRL 1 Registers START
#define STARTUP_BOOST (0 << 7) // 1 to enable
#define AC_COUPLE (0 << 5)
#define DRIVE_TIME (26) // Set default

#define CTRL1_BYTE (STARTUP_BOOST | AC_COUPLE | DRIVE_TIME)
#define CTRL1_REGISTER 0x1B
// CTRL 1 Registers END

// CTRL 2 Registers START
#define BIDIR_INPUT (1<<7) // Enable bidirectional input for Open-loop operation (<50% input is braking applied)
#define BRAKE_STABILIZER (1<<6) // Improve loop stability? LOL no clue.
#define SAMPLE_TIME (3U << 4)
#define BLANKING_TIME_LOWER ( (BLANKING_TIME_BASE & 0b00000011) << 2 )
#define IDISS_TIME_LOWER (IDISS_TIME_BASE & 0b00000011)
// Write to register 0x1C
#define CTRL2_BYTE (BIDIR_INPUT | BRAKE_STABILIZER | SAMPLE_TIME | BLANKING_TIME_LOWER | IDISS_TIME_LOWER)
#define CTRL2_REGISTER 0x1C
// CTRL 2 Registers END

/*
    In open-loop mode, the RATED_VOLTAGE[7:0] bit is ignored. Instead, the OD_CLAMP[7:0] bit (in register 0x17)
    is used to set the rated voltage for the open-loop drive modes. For the ERM, Equation 6 calculates the rated
    voltage with a full-scale input signal. For the LRA, Equation 7 calculates the RMS voltage with a full-scale input
    signal.

    Equation 7
    V(Lra-OL_RMS) = 21.32 * (10^-3) * OD_CLAMP * sqrt(1-resonantfreq * 800 * (10^-6))
*/
#define ODCLAMP_BYTE (uint8_t) 42 // Using the equation from the DRV2604L datasheet for Open Loop mode
// First value for 3.3v at 320hz is 179
// Alt value for 3v is uint8_t 163 (320hz)
// Alt value for 3v at 160hz is uint8_t 150
// Write to register 0x17
#define ODCLAMP_REGISTER 0x17

// CTRL 3 Registers START

/*
    To configure the DRV2605L device in LRA open-loop operation, the LRA must be selected by writing the
    N_ERM_LRA bit to 1 in register 0x1A, and the LRA_OPEN_LOOP bit to 1 in register 0x1D. If PWM interface is
    used, the open-loop frequency is given by the PWM frequency divided by 128. If PWM interface is not used, the
    open-loop frequency is given by the OL_LRA_PERIOD[6:0] bit in register 0x20.
*/
#define OL_LRA_PERIOD 42

#define NG_THRESH_DISABLED 0
#define NG_THRESH_2 1 // Percent
#define NG_THRESH_4 2
#define NG_THRESH_8 3
#define NG_THRESH (NG_THRESH_4 << 6)

#define ERM_OPEN_LOOP (0 << 5)

#define SUPPLY_COMP_ENABLED 0
#define SUPPLY_COMP_DISABLED 1
#define SUPPLY_COMP_DIS (SUPPLY_COMP_ENABLED << 4)

#define DATA_FORMAT_RTP_SIGNED 0
#define DATA_FORMAT_RTP_UNSIGNED 1
#define DATA_FORMAT_RTP (DATA_FORMAT_RTP_SIGNED << 3)

#define LRA_DRIVE_MODE_OPC 0 // Once per cycle
#define LRA_DRIVE_MODE_TPC 1 // Twice per cycle
#define LRA_DRIVE_MODE (LRA_DRIVE_MODE_OPC << 2)

#define N_PWM_ANALOG_PWM 0
#define N_PWM_ANALOG_ANALOG 1
#define N_PWM_ANALOG (N_PWM_ANALOG_PWM << 1)

#define LRA_OPEN_LOOP (1)

#define CTRL3_BYTE (NG_THRESH | ERM_OPEN_LOOP | SUPPLY_COMP_DIS | DATA_FORMAT_RTP | LRA_DRIVE_MODE | N_PWM_ANALOG | LRA_OPEN_LOOP)
#define CTRL3_REGISTER 0x1D

// CTRL 3 Register END

// CTRL 4 Registers START
#define ZC_DET_TIME (0<<6) // 100us
#define AUTO_CAL_TIME (2U << 4)
// Write to register 0x1E
#define CTRL4_BYTE (ZC_DET_TIME | AUTO_CAL_TIME)
#define CTRL4_REGISTER 0x1E
// CTRL 4 Registers END

// CTRL 5 Registers START
#define BLANKING_TIME_UPPER ( (BLANKING_TIME_BASE & 0b00001100) )
#define IDISS_TIME_UPPER ( (IDISS_TIME_BASE & 0b00001100) >> 2 )
// Write to register 0x1F
#define CTRL5_BYTE (BLANKING_TIME_UPPER | IDISS_TIME_UPPER)
#define CTRL5_REGISTER 0x1F
// CTRL 5 Registers END

// MODE Register START
#define DEV_RESET (0<<7)
#define STANDBY (1<<6)
#define MODE (3)
#define MODE_BYTE (DEV_RESET | MODE)
#define STANDBY_MODE_BYTE (DEV_RESET | STANDBY | 0x00)
#define MODE_REGISTER 0x01
// Write to register 0x01
#define MODE_CALIBRATE (0x07)
// MODE Register END

#define GO_REGISTER 0x0C
#define GO_SET (1U)

#define STATUS_REGISTER 0x00

#define RTP_AMPLITUDE_REGISTER 0x02
#define RTP_FREQUENCY_REGISTER 0x22


/* Note from datasheet
    When using the PWM input in open-loop mode, the DRV2604L device employs a fixed divider that observes the
    PWM signal and commutates the output drive signal at the PWM frequency divided by 128. To accomplish LRA
    drive, the host should drive the PWM frequency at 128 times the desired operating frequency.
*/
void play_pwm_frequency(float frequency, float amplitude) 
{
    static bool disabled = false;
    static float playing_freq = 0;
    static float playing_amp = 0;

    if(!amplitude) 
    {
        //disabled = true;
        playing_amp = -1;
        playing_freq = -1;

        uint8_t _set_mode[] = {MODE_REGISTER, STANDBY_MODE_BYTE};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode, 2, false);
        disabled = true;
        return;
    }
    else if(disabled)
    {
        uint8_t _set_mode[] = {MODE_REGISTER, MODE_BYTE};
        i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode, 2, false);
        disabled = false;
    }

    frequency = (frequency>1300) ? 1300 : frequency;
    frequency = (frequency<40) ? 40 : frequency;


    //pwm_set_enabled(slice_num, false);
    //frequency *= 128; // Neets to multiply by 128 to get appropriate output signal

    // Calculate wrap value
    float target_wrap = (float) PWM_CLOCK_BASE / frequency;

    if(frequency != playing_freq)
    {
        pwm_set_wrap(slice_num, (uint16_t) target_wrap);
        playing_freq = frequency;
    }
	

    float min_amp = (amplitude < 0.2f) ? 0.2f : amplitude;
 
    float amp_val_base = target_wrap * amplitude ;

    if(amp_val_base != playing_amp)
    {
        pwm_set_chan_level(slice_num, PWM_CHAN_A, (uint16_t) amp_val_base); 
        playing_amp = amp_val_base;
    }
	
	
	pwm_set_enabled(slice_num, true); // let's go!
}

void cb_hoja_rumble_set(rumble_data_s *data)
{
    if(data->amplitude_low > data->amplitude_high)
    {
        _current_amplitude = data->amplitude_low;
        _current_frequency = data->frequency_low;
    }
    else
    {
        _current_amplitude = data->amplitude_high;
        _current_frequency = data->frequency_high;
    }

    play_pwm_frequency(_current_frequency, _current_amplitude);
}


bool played = false;
void test_sequence()
{
    if(played) return;
    played = true;
    for(int i = 0; i < 27; i+=1)
    {
        play_pwm_frequency(song[i], 1);
        watchdog_update();
        sleep_ms(150);
    }
    sleep_ms(150);
    play_pwm_frequency(100, 0);
    played = false;
}

bool lra_init = false;
// Obtain and dump calibration values for auto-init LRA
void cb_hoja_rumble_init()
{
    if(lra_init) return;
    sleep_ms(100);
    lra_init = true;

    #if ( HOJA_DEVICE_ID == 0xA003 )
        // Init GPIO for LRA switch
        gpio_init(GPIO_LRA_SDA_SEL);
        gpio_set_dir(GPIO_LRA_SDA_SEL, GPIO_OUT);
        gpio_put(GPIO_LRA_SDA_SEL, 1);
    #endif
    
    // Set PWM function
    gpio_init(GPIO_LRA_IN);
    gpio_set_function(GPIO_LRA_IN, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to our GPIO pin
    slice_num = pwm_gpio_to_slice_num(GPIO_LRA_IN);

    // We want a 2Mhz clock driving the PWM
    // this means 1000 ticks = 2khz, 2000 ticks = 1khz
    float divider = 125000000.0f / PWM_CLOCK_BASE;

    // Set the PWM clock divider to run at 125MHz
    pwm_set_clkdiv(slice_num, divider);

    // Take MODE out of standby
    uint8_t _set_mode1[] = {MODE_REGISTER, 0x00};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode1, 2, false);
    
    uint8_t _set_feedback[] = {FEEDBACK_CTRL_REGISTER, FEEDBACK_CTRL_BYTE};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_feedback, 2, false);
    sleep_ms(10);

    //uint8_t _set_control1[] = {CTRL1_REGISTER, CTRL1_BYTE};
    //i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control1, 2, false);
    //sleep_ms(10);

    //uint8_t _set_control2[] = {CTRL2_REGISTER, CTRL2_BYTE};
    //i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control2, 2, false);
    //sleep_ms(10);

    uint8_t _set_control3[] = {CTRL3_REGISTER, CTRL3_BYTE};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control3, 2, false);
    sleep_ms(10);

    //uint8_t _set_control4[] = {CTRL4_REGISTER, CTRL4_BYTE};
    //i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control4, 2, false);
    //sleep_ms(10);
    
    //uint8_t _set_control5[] = {CTRL5_REGISTER, CTRL5_BYTE};
    //i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control5, 2, false);
    //sleep_ms(10);

    //uint8_t _set_compensation[] = {0x18, HAPTIC_COMPENSATION};
    //i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_compensation, 2, false);

    //uint8_t _set_bemf[] = {0x19, HAPTIC_BACKEMF};
    //i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_bemf, 2, false);

    //uint8_t _set_freq[] = {RTP_FREQUENCY_REGISTER, 31};
    //i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_freq, 2, false);

    uint8_t _set_odclamped[] = {ODCLAMP_REGISTER, ODCLAMP_BYTE};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_odclamped, 2, false);

    uint8_t _set_mode2[] = {MODE_REGISTER, STANDBY_MODE_BYTE};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode2, 2, false);

    uint8_t _set_mode3[] = {MODE_REGISTER, MODE_BYTE};
    i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode3, 2, false);

    play_pwm_frequency(300, 0.0);

}

bool app_rumble_hwtest()
{

    test_sequence();
    //rumble_get_calibration();
    return true;
}

void app_rumble_task(uint32_t timestamp)
{
    return;
    static interval_s interval = {0};
    if(interval_run(timestamp, 8000, &interval))
    {
        
        if(_current_amplitude>0)
        {
            //_current_amplitude -= 0.015f;
            //_current_amplitude = (_current_amplitude<0) ? 0 : _current_amplitude;
            play_pwm_frequency(_current_frequency, _current_amplitude);
            
        }
    }
}