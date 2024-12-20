#include "hoja_includes.h"
#include "interval.h"
#include "main.h"
#include "math.h"

// #include "egg.h"

void app_rumble_task(uint32_t timestamp);

#define SOC_CLOCK (float)HOJA_SYS_CLK_HZ

#define SAMPLE_RATE 12000
#define BUFFER_SIZE 255
#define SAMPLE_TRANSITION 30
#define PWM_WRAP BUFFER_SIZE
#define PWM_PIN GPIO_LRA_IN_LO

#define STARTING_LOW_FREQ 160.0f
#define STARTING_HIGH_FREQ 320.0f

// Example from https://github.com/moefh/pico-audio-demo/blob/main/audio.c

#define REPETITION_RATE 4

static uint32_t single_sample = 0;
static uint32_t *single_sample_ptr = &single_sample;
static int pwm_dma_chan, trigger_dma_chan, sample_dma_chan;

hoja_rumble_msg_s msg_left = {0};
hoja_rumble_msg_s msg_right = {0};

// static volatile int cur_audio_buffer;

#define M_PI 3.14159265358979323846
#define TWO_PI (M_PI * 2)

static volatile int audio_buffer_idx_used = 0;
static uint8_t audio_buffers[2][BUFFER_SIZE] = {0};
static volatile bool ready_next_sine = false;

// Which sample we're processing
uint8_t samples_idx;
// How many sample frames we have to go through
uint8_t samples_count;
// How many samples we've generated
uint8_t sample_counter;
float _rumble_scaler = 0;

typedef struct
{
    // How much we increase the phase each time
    float phase_step;
    // How much to increase our phase step each time
    float phase_accumulator;
    // Our current phase
    float phase;
    float f;
    float f_step;
    float f_target;
    float a;
    float a_step;
    float a_target;
} haptic_s;

typedef struct
{
    haptic_s lo_state;
    haptic_s hi_state;
} haptic_both_s;

haptic_both_s haptic_state = {0};

float clamp_rumble_lo(float amplitude)
{
    const float min = 0.05f;
    const float max = 1.0f;

    const float upper_expander = 0.5f; // We want to treat 0 to 0.75 as 0 to 1

    if (!amplitude)
        return 0;
    if (!_rumble_scaler)
        return 0;

    float expanded_amp = amplitude / upper_expander;
    expanded_amp = (expanded_amp > 1) ? 1 : expanded_amp;

    float range = max - min;
    range *= _rumble_scaler;

    float retval = 0;

    retval = range * expanded_amp;
    retval += min;
    if (retval > max)
        retval = max;
    return retval;
}

float clamp_rumble_hi(float amplitude)
{
    const float min = 0.2f;
    const float max = 1.0f;

    const float upper_expander = 0.5f; // We want to treat 0 to 0.75 as 0 to 1

    if (!amplitude)
        return 0;
    if (!_rumble_scaler)
        return 0;

    float expanded_amp = amplitude / upper_expander;
    expanded_amp = (expanded_amp > 1) ? 1 : expanded_amp;

    float range = max - min;
    range *= _rumble_scaler;

    float retval = 0;

    retval = range * expanded_amp;
    retval += min;
    if (retval > max)
        retval = max;
    return retval;
}

#define SIN_TABLE_SIZE 4096
int16_t sin_table[SIN_TABLE_SIZE] = {0};

void sine_table_init()
{
    float inc = TWO_PI / SIN_TABLE_SIZE;
    float fi = 0;

    for (int i = 0; i < SIN_TABLE_SIZE; i++)
    {
        float sample = sinf(fi);
        sin_table[i] = (int16_t)((sample) * (255.0f));

        fi += inc;
        fi = fmodf(fi, TWO_PI);
    }
}

// This function is designed to be broken
// up into many steps to avoid causing slowdown
// or starve other tasks
// Returns true when the buffer is filled
bool generate_sine_wave(uint8_t *buffer, haptic_both_s *state)
{
    amfm_s      cur[3] = {0};
    uint16_t    transition_idx = 0;
    uint8_t     sample_idx = 0;
    int8_t samples = haptics_get(true, cur, false, NULL);

    for (uint16_t i = 0; i < BUFFER_SIZE; i++)
    {
        // This means we need to reset our state and
        // calculate our next step
        if (!transition_idx)
        {
            if ((samples > 0) && (sample_idx < samples))
            {
                // Set high frequency
                float hif = cur[sample_idx].f_hi;
                float hia = cur[sample_idx].a_hi;

                state->hi_state.f_target = hif;
                state->hi_state.f_step = (hif - state->hi_state.f) / SAMPLE_TRANSITION;

                state->hi_state.phase_step = (SIN_TABLE_SIZE * state->hi_state.f) / SAMPLE_RATE;
                float hphase_step_end = (SIN_TABLE_SIZE * state->hi_state.f_target) / SAMPLE_RATE;
                state->hi_state.phase_accumulator = (hphase_step_end - state->hi_state.phase_step) / SAMPLE_TRANSITION;

                state->hi_state.a_target = hia;
                state->hi_state.a_step = (hia - state->hi_state.a) / SAMPLE_TRANSITION;

                float lof = cur[sample_idx].f_lo;
                float loa = cur[sample_idx].a_lo;

                // Set low frequency
                state->lo_state.f_target = lof;
                state->lo_state.f_step = (lof - state->lo_state.f) / SAMPLE_TRANSITION;

                state->lo_state.phase_step = (SIN_TABLE_SIZE * state->lo_state.f) / SAMPLE_RATE;
                float lphase_step_end = (SIN_TABLE_SIZE * state->lo_state.f_target) / SAMPLE_RATE;
                state->lo_state.phase_accumulator = (lphase_step_end - state->lo_state.phase_step) / SAMPLE_TRANSITION;

                state->lo_state.a_target = loa;
                state->lo_state.a_step = (loa - state->lo_state.a) / SAMPLE_TRANSITION;

                sample_idx++;
            }
            else
            {
                // reset all steps so we just
                // continue generating
                state->hi_state.f_step = 0;
                state->hi_state.a_step = 0;
                state->lo_state.f_step = 0;
                state->lo_state.a_step = 0;
                state->hi_state.phase_accumulator = 0;
                state->lo_state.phase_accumulator = 0;
            }
        }

        if (haptic_state.hi_state.phase_accumulator > 0)
            haptic_state.hi_state.phase_step += haptic_state.hi_state.phase_accumulator;

        if (haptic_state.lo_state.phase_accumulator > 0)
            haptic_state.lo_state.phase_step += haptic_state.lo_state.phase_accumulator;

        // float sample_low    = sinf(lo_state.phase); //sinf
        float sample_high = sin_table[(uint16_t)haptic_state.hi_state.phase];
        float sample_low = sin_table[(uint16_t)haptic_state.lo_state.phase];

        float hi_a = clamp_rumble_hi(haptic_state.hi_state.a);
        float lo_a = clamp_rumble_lo(haptic_state.lo_state.a);
        float comb_a = hi_a + lo_a;

        if ((comb_a) > 1.0f)
        {
            float new_ratio = 1.0f / (comb_a);
            hi_a *= new_ratio;
            lo_a *= new_ratio;
        }

        sample_high *= hi_a;
        sample_low *= lo_a;

        // Combine samples
        float sample = sample_low + sample_high;

        sample = (sample > 255) ? 255 : (sample < 0) ? 0
                                                     : sample;

        buffer[i] = (uint8_t)sample;

        // Update phases and steps
        haptic_state.hi_state.phase += haptic_state.hi_state.phase_step;
        haptic_state.lo_state.phase += haptic_state.lo_state.phase_step;

        // Keep phases within [0, 2π) range
        haptic_state.hi_state.phase = fmodf(haptic_state.hi_state.phase, SIN_TABLE_SIZE);
        haptic_state.lo_state.phase = fmodf(haptic_state.lo_state.phase, SIN_TABLE_SIZE);

        if (transition_idx < SAMPLE_TRANSITION)
        {
            state->hi_state.f += state->hi_state.f_step;
            state->hi_state.a += state->hi_state.a_step;

            state->lo_state.f += state->lo_state.f_step;
            state->lo_state.a += state->lo_state.a_step;

            transition_idx++;
        }
        else
        {
            // Set values to target
            state->hi_state.f = state->hi_state.f_target;
            state->hi_state.a = state->hi_state.a_target;
            state->lo_state.f = state->lo_state.f_target;
            state->lo_state.a = state->lo_state.a_target;
            state->hi_state.phase_accumulator = 0;
            state->lo_state.phase_accumulator = 0;

            // Reset sample index
            transition_idx = 0;
        }
    }
}

// volatile uint32_t _audio_playback_idx = 0;
// volatile bool _audio_playback = false;

static void __isr __time_critical_func(dma_handler)()
{
    audio_buffer_idx_used = 1 - audio_buffer_idx_used;

    dma_hw->ch[sample_dma_chan].al1_read_addr = (intptr_t)&audio_buffers[audio_buffer_idx_used][0];
    dma_hw->ch[trigger_dma_chan].al3_read_addr_trig = (intptr_t)&single_sample_ptr;

    ready_next_sine = true;

    dma_hw->ints1 = 1u << trigger_dma_chan;
}

void audio_init(int audio_pin, int sample_freq)
{
    sine_table_init();

    gpio_set_function(audio_pin, GPIO_FUNC_PWM);

    int audio_pin_slice = pwm_gpio_to_slice_num(audio_pin);
    int audio_pin_chan = pwm_gpio_to_channel(audio_pin);

    uint f_clk_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
    float clock_div = ((float)f_clk_sys * 1000.0f) / 254.0f / (float)sample_freq / (float)REPETITION_RATE;

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, clock_div);
    pwm_config_set_wrap(&config, 254);
    pwm_init(audio_pin_slice, &config, true);

    pwm_dma_chan = dma_claim_unused_channel(true);
    trigger_dma_chan = dma_claim_unused_channel(true);
    sample_dma_chan = dma_claim_unused_channel(true);

    // setup PWM DMA channel
    dma_channel_config pwm_dma_chan_config = dma_channel_get_default_config(pwm_dma_chan);
    channel_config_set_transfer_data_size(&pwm_dma_chan_config, DMA_SIZE_32);        // transfer 32 bits at a time
    channel_config_set_read_increment(&pwm_dma_chan_config, false);                  // always read from the same address
    channel_config_set_write_increment(&pwm_dma_chan_config, false);                 // always write to the same address
    channel_config_set_chain_to(&pwm_dma_chan_config, sample_dma_chan);              // trigger sample DMA channel when done
    channel_config_set_dreq(&pwm_dma_chan_config, DREQ_PWM_WRAP0 + audio_pin_slice); // transfer on PWM cycle end
    dma_channel_configure(pwm_dma_chan,
                          &pwm_dma_chan_config,
                          &pwm_hw->slice[audio_pin_slice].cc, // write to PWM slice CC register
                          &single_sample,                     // read from single_sample
                          REPETITION_RATE,                    // transfer once per desired sample repetition
                          false                               // don't start yet
    );

    // setup trigger DMA channel
    dma_channel_config trigger_dma_chan_config = dma_channel_get_default_config(trigger_dma_chan);
    channel_config_set_transfer_data_size(&trigger_dma_chan_config, DMA_SIZE_32);        // transfer 32-bits at a time
    channel_config_set_read_increment(&trigger_dma_chan_config, false);                  // always read from the same address
    channel_config_set_write_increment(&trigger_dma_chan_config, false);                 // always write to the same address
    channel_config_set_dreq(&trigger_dma_chan_config, DREQ_PWM_WRAP0 + audio_pin_slice); // transfer on PWM cycle end
    dma_channel_configure(trigger_dma_chan,
                          &trigger_dma_chan_config,
                          &dma_hw->ch[pwm_dma_chan].al3_read_addr_trig, // write to PWM DMA channel read address trigger
                          &single_sample_ptr,                           // read from location containing the address of single_sample
                          REPETITION_RATE * BUFFER_SIZE,                // trigger once per audio sample per repetition rate
                          false                                         // don't start yet
    );
    dma_channel_set_irq1_enabled(trigger_dma_chan, true); // fire interrupt when trigger DMA channel is done
    irq_set_exclusive_handler(DMA_IRQ_1, dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);

    // setup sample DMA channel
    dma_channel_config sample_dma_chan_config = dma_channel_get_default_config(sample_dma_chan);
    channel_config_set_transfer_data_size(&sample_dma_chan_config, DMA_SIZE_8); // transfer 8-bits at a time
    channel_config_set_read_increment(&sample_dma_chan_config, true);           // increment read address to go through audio buffer
    channel_config_set_write_increment(&sample_dma_chan_config, false);         // always write to the same address
    dma_channel_configure(sample_dma_chan,
                          &sample_dma_chan_config,
                          (char *)&single_sample + 2 * audio_pin_chan, // write to single_sample
                          &audio_buffers[0][0],                        // read from audio buffer
                          1,                                           // only do one transfer (once per PWM DMA completion due to chaining)
                          false                                        // don't start yet
    );

    // clear audio buffers
    memset(audio_buffers[0], 0, BUFFER_SIZE);
    memset(audio_buffers[1], 0, BUFFER_SIZE);

    // kick things off with the trigger DMA channel
    dma_channel_start(trigger_dma_chan);
}

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
    B3, E4, A4, E4, D5, E4, Db5, Ab4, Eb4, Ab4, Fs4, Db4, Ab3, Db4, D4, G4, C5, F5, Bb5, F5, C5, A5, E5, B4, Ab5, 0, 0, E4};

#define DRV2605_SLAVE_ADDR 0x5A

#if (HOJA_DEVICE_ID == 0xA003)
#define LRA_LOW_PWM_CHAN PWM_CHAN_A
#define LRA_HI_PWM_CHAN PWM_CHAN_A
// LRA SDA Select (left/right)
#define GPIO_LRA_SDA_SEL 25
// LRA Audio Output
#define GPIO_LRA_IN_LO 24
#define GPIO_LRA_IN_HI 24
#elif (HOJA_DEVICE_ID == 0xA004)
// LRA Audio Output
#define LRA_LOW_PWM_CHAN PWM_CHAN_B
#define LRA_HI_PWM_CHAN PWM_CHAN_A
#define GPIO_LRA_IN_LO 21 // Low
#define GPIO_LRA_IN_HI 24
#endif

#define RATED_VOLTAGE_HEX 0x3
#define RATED_VOLTAGE_REGISTER 0x16

#define HAPTIC_COMPENSATION 0
#define HAPTIC_BACKEMF 0

#define BLANKING_TIME_BASE (1) //(1)
#define IDISS_TIME_BASE (1)

#define N_ERM_LRA (1 << 7)
#define FB_BRAKE_FACTOR (7 << 4) // Disable brake
#define LOOP_GAIN (1U << 2)
#define HAPTIC_BEMF_GAIN 2
// Write to register 0x1A
#define FEEDBACK_CTRL_BYTE (N_ERM_LRA | FB_BRAKE_FACTOR | LOOP_GAIN | HAPTIC_BEMF_GAIN)
#define FEEDBACK_CTRL_REGISTER 0x1A

// CTRL 1 Registers START
#define STARTUP_BOOST (0 << 7) // 1 to enable
#define AC_COUPLE (1 << 5)
#define DRIVE_TIME (20) // Set default

#define CTRL1_BYTE (STARTUP_BOOST | AC_COUPLE | DRIVE_TIME)
#define CTRL1_REGISTER 0x1B
// CTRL 1 Registers END

// CTRL 2 Registers START
#define BIDIR_INPUT (1 << 7)      // Enable bidirectional input for Open-loop operation (<50% input is braking applied)
#define BRAKE_STABILIZER (0 << 6) // Improve loop stability? LOL no clue.
#define SAMPLE_TIME (3U << 4)
#define BLANKING_TIME_LOWER ((BLANKING_TIME_BASE & 0b00000011) << 2)
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
#define ODCLAMP_BYTE (uint8_t)20 // Using the equation from the DRV2604L datasheet for Open Loop mode
// First value for 3.3v at 320hz is 179
// Alt value for 3v is uint8_t 163 (320hz)
// Alt value for 3v at 160hz is uint8_t 150
// Write to register 0x17
#define ODCLAMP_REGISTER 0x17

// LRA Open Loop Period (Address: 0x20)
#define OL_LRA_REGISTER 0x20
#define OL_LRA_PERIOD 0b1111111

// CTRL 3 Registers START

/*
    To configure the DRV2605L device in LRA open-loop operation, the LRA must be selected by writing the
    N_ERM_LRA bit to 1 in register 0x1A, and the LRA_OPEN_LOOP bit to 1 in register 0x1D. If PWM interface is
    used, the open-loop frequency is given by the PWM frequency divided by 128. If PWM interface is not used, the
    open-loop frequency is given by the OL_LRA_PERIOD[6:0] bit in register 0x20.
*/

#define NG_THRESH_DISABLED 0
#define NG_THRESH_2 1 // Percent
#define NG_THRESH_4 2
#define NG_THRESH_8 3
#define NG_THRESH (NG_THRESH_DISABLED << 6)

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
#define N_PWM_ANALOG (N_PWM_ANALOG_ANALOG << 1)

#define LRA_OPEN_LOOP (1)

#define CTRL3_BYTE (NG_THRESH | ERM_OPEN_LOOP | SUPPLY_COMP_DIS | DATA_FORMAT_RTP | LRA_DRIVE_MODE | N_PWM_ANALOG | LRA_OPEN_LOOP)
#define CTRL3_REGISTER 0x1D

// CTRL 3 Register END

// CTRL 4 Registers START
#define ZC_DET_TIME (0 << 6) // 100us
#define AUTO_CAL_TIME (2U << 4)
// Write to register 0x1E
#define CTRL4_BYTE (ZC_DET_TIME | AUTO_CAL_TIME)
#define CTRL4_REGISTER 0x1E
// CTRL 4 Registers END

// CTRL 5 Registers START
#define BLANKING_TIME_UPPER ((BLANKING_TIME_BASE & 0b00001100))
#define IDISS_TIME_UPPER ((IDISS_TIME_BASE & 0b00001100) >> 2)
// Write to register 0x1F
#define CTRL5_BYTE (BLANKING_TIME_UPPER | IDISS_TIME_UPPER)
#define CTRL5_REGISTER 0x1F
// CTRL 5 Registers END

// MODE Register START
#define DEV_RESET (0 << 7)
#define STANDBY (1 << 6)
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

void cb_hoja_rumble_set(hoja_rumble_msg_s *left, hoja_rumble_msg_s *right)
{
    memcpy(&msg_left, left, sizeof(hoja_rumble_msg_s));
    msg_left.unread = true;
}

void cb_hoja_rumble_test()
{
    haptics_set_all(0, 0, HOJA_HAPTIC_BASE_LFREQ, 1);
    haptics_set_all(0, 0, HOJA_HAPTIC_BASE_LFREQ, 1);

    for (int i = 0; i < 62; i++)
    {
        app_rumble_task(0);
        watchdog_update();
        sleep_ms(8);
    }

    haptics_set_all(0, 0, HOJA_HAPTIC_BASE_LFREQ, 0);
    haptics_set_all(0, 0, HOJA_HAPTIC_BASE_LFREQ, 0);
}

bool played = false;
void test_sequence()
{
    hoja_rumble_msg_s msg = {.sample_count = 1, .samples[0] = {.high_amplitude = 0, .high_frequency = 320.0f, .low_amplitude = 1.0f, .low_frequency = 160.0f}, .unread = true};

    if (played)
        return;
    played = true;
    for (int i = 0; i < 27; i += 1)
    {
        msg.samples[0].low_amplitude = 1.0f;
        msg.samples[0].low_frequency = song[i];
        cb_hoja_rumble_set(&msg, &msg);
        app_rumble_task(0);
        watchdog_update();
        sleep_ms(150);
    }
    sleep_ms(150);
    msg.samples[0].low_amplitude = 0;
    msg.samples[0].low_frequency = 0;
    cb_hoja_rumble_set(&msg, &msg);
    played = false;
}

bool lra_init = false;
// Obtain and dump calibration values for auto-init LRA
void cb_hoja_rumble_init()
{
    //return; // Do not init debug
    if (!lra_init)
    {
        sleep_ms(250);
        lra_init = true;

        // Take MODE out of standby
        // uint8_t _set_mode1[] = {MODE_REGISTER, 0x00};
        // i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode1, 2, false);
        //
        uint8_t _set_feedback[] = {FEEDBACK_CTRL_REGISTER, FEEDBACK_CTRL_BYTE};
        i2c_safe_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_feedback, 2, false);
        sleep_ms(5);

        uint8_t _set_control1[] = {CTRL1_REGISTER, CTRL1_BYTE};
        i2c_safe_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control1, 2, false);
        sleep_ms(5);

        // uint8_t _set_control2[] = {CTRL2_REGISTER, CTRL2_BYTE};
        // i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control2, 2, false);
        // sleep_ms(10);

        uint8_t _set_control3[] = {CTRL3_REGISTER, CTRL3_BYTE};
        i2c_safe_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control3, 2, false);
        sleep_ms(5);

        // uint8_t _set_control4[] = {CTRL4_REGISTER, CTRL4_BYTE};
        // i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control4, 2, false);
        // sleep_ms(10);

        // uint8_t _set_control5[] = {CTRL5_REGISTER, CTRL5_BYTE};
        // i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_control5, 2, false);
        // sleep_ms(10);

        // uint8_t _set_compensation[] = {0x18, HAPTIC_COMPENSATION};
        // i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_compensation, 2, false);

        // uint8_t _set_bemf[] = {0x19, HAPTIC_BACKEMF};
        // i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_bemf, 2, false);

        // uint8_t _set_freq[] = {OL_LRA_REGISTER, OL_LRA_PERIOD};
        // i2c_safe_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_freq, 2, false);

        uint8_t _set_odclamped[] = {ODCLAMP_REGISTER, ODCLAMP_BYTE};
        i2c_safe_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_odclamped, 2, false);
        sleep_ms(5);

        // uint8_t _set_mode2[] = {MODE_REGISTER, STANDBY_MODE_BYTE};
        // i2c_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode2, 2, false);

        uint8_t _set_mode3[] = {MODE_REGISTER, MODE_BYTE};
        i2c_safe_write_blocking(HOJA_I2C_BUS, DRV2605_SLAVE_ADDR, _set_mode3, 2, false);
        sleep_ms(5);

        audio_init(GPIO_LRA_IN_LO, SAMPLE_RATE);
    }

    uint8_t intensity = 0;
    rumble_type_t type;
    hoja_get_rumble_settings(&intensity, &type);

    _rumble_scaler = ((float)intensity / 100.0f);
}

bool app_rumble_hwtest()
{
    //_audio_playback_idx = 0;
    //_audio_playback = true;
    cb_hoja_rumble_test();
    // test_sequence();
    // rumble_get_calibration();
    return true;
}

void app_rumble_task(uint32_t timestamp)
{
    if (ready_next_sine)
    {
        ready_next_sine = false;
        uint8_t available_buffer = 1 - audio_buffer_idx_used;
        generate_sine_wave(audio_buffers[available_buffer], &haptic_state);
    }
}