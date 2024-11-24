#include "hoja_includes.h"
#include "board_config.h"
#include "main.h"

button_remap_s user_map = {
    .dpad_up = MAPCODE_DUP,
    .dpad_down = MAPCODE_DDOWN,
    .dpad_left = MAPCODE_DLEFT,
    .dpad_right = MAPCODE_DRIGHT,

    .button_a = MAPCODE_B_A,
    .button_b = MAPCODE_B_B,
    .button_x = MAPCODE_B_X,
    .button_y = MAPCODE_B_Y,

    .trigger_l = MAPCODE_T_ZL,
    .trigger_r = MAPCODE_T_ZR,
    .trigger_zl = MAPCODE_T_L,
    .trigger_zr = MAPCODE_T_R,

    .button_plus = MAPCODE_B_PLUS,
    .button_minus = MAPCODE_B_MINUS,
    .button_stick_left = MAPCODE_B_STICKL,
    .button_stick_right = MAPCODE_B_STICKR,
};

void _gpio_put_od(uint gpio, bool level)
{
    if(level)
    {
        gpio_set_dir(gpio, GPIO_IN);
        //gpio_pull_up(gpio);
        gpio_disable_pulls(gpio);
        gpio_put(gpio, 1);
    }
    else
    {
        gpio_set_dir(gpio, GPIO_OUT);
        gpio_disable_pulls(gpio);
        gpio_put(gpio, 0);
    }
}

void cb_hoja_baseband_update_loop(button_data_s *buttons)
{
    if(buttons->trigger_l)
    {
        watchdog_reboot(0, 0, 0);
    }

    static bool enstate;

    if(buttons->button_plus && !enstate)
    {
        enstate = true;
        _gpio_put_od(PGPIO_ESP_EN, false);
    }
    else if(!buttons->button_plus && enstate)
    {
        _gpio_put_od(PGPIO_ESP_EN, true);
        enstate = false;
    }
    sleep_ms(10);
    
}

void cb_hoja_set_uart_enabled(bool enable)
{
    if(enable)
    {
        gpio_put(PGPIO_BUTTON_USB_EN, 1);
        sleep_ms(100);
        gpio_put(PGPIO_BUTTON_USB_SEL, 1);
        sleep_ms(100);
        gpio_put(PGPIO_BUTTON_USB_EN, 0);
    }
    else
    {
        gpio_put(PGPIO_BUTTON_USB_EN, 1);
        sleep_ms(100);
        gpio_put(PGPIO_BUTTON_USB_SEL, 0);
        sleep_ms(100);
        gpio_put(PGPIO_BUTTON_USB_EN, 0);
    }
}

void cb_hoja_set_bluetooth_enabled(bool enable)
{
    if(enable)
    {
        //cb_hoja_set_uart_enabled(true);
        // Release ESP to be controlled externally
        _gpio_put_od(PGPIO_ESP_EN, true);
    }
    else
    {
        _gpio_put_od(PGPIO_ESP_EN, false);
    }
}

void cb_hoja_hardware_setup()
{
    // Set up GPIO for input buttons
    hoja_setup_gpio_button(PGPIO_BUTTON_RS);
    hoja_setup_gpio_button(PGPIO_BUTTON_LS);

    hoja_setup_gpio_push(PGPIO_PUSH_A);
    hoja_setup_gpio_push(PGPIO_PUSH_B);
    hoja_setup_gpio_push(PGPIO_PUSH_C);
    hoja_setup_gpio_push(PGPIO_PUSH_D);

    hoja_setup_gpio_scan(PGPIO_SCAN_A);
    hoja_setup_gpio_scan(PGPIO_SCAN_B);
    hoja_setup_gpio_scan(PGPIO_SCAN_C);
    hoja_setup_gpio_scan(PGPIO_SCAN_D);
}

bool set = false;
bool unset = true;
static uint32_t this_timestamp = 0;

void cb_hoja_read_buttons(button_data_s *data)
{
    // Keypad version
    gpio_put(PGPIO_SCAN_A, false);
    sleep_us(5);
    data->button_a  = !gpio_get(PGPIO_PUSH_C);
    data->button_b  = !gpio_get(PGPIO_PUSH_D);
    data->button_x  = !gpio_get(PGPIO_PUSH_A);
    data->button_y  = !gpio_get(PGPIO_PUSH_B);
    gpio_put(PGPIO_SCAN_A, true);

    gpio_put(PGPIO_SCAN_B, false);
    sleep_us(5);
    data->dpad_left     = !gpio_get(PGPIO_PUSH_D);
    data->dpad_right    = !gpio_get(PGPIO_PUSH_C);
    data->dpad_down     = !gpio_get(PGPIO_PUSH_B);
    data->dpad_up       = !gpio_get(PGPIO_PUSH_A);
    gpio_put(PGPIO_SCAN_B, true);

    gpio_put(PGPIO_SCAN_C, false);
    sleep_us(5);
    data->button_plus       = !gpio_get(PGPIO_PUSH_A);
    data->button_home       = !gpio_get(PGPIO_PUSH_B);
    data->button_capture    = !gpio_get(PGPIO_PUSH_D);
    data->button_minus      = !gpio_get(PGPIO_PUSH_C);
    gpio_put(PGPIO_SCAN_C, true);

    /*
    if (data->button_capture)
    {
        reset_usb_boot(0, 0);
    }
    if (data->button_home)
    {
        watchdog_reboot(0, 0, 0);
    }*/

    gpio_put(PGPIO_SCAN_D, false);
    sleep_us(5);
    data->trigger_r     = !gpio_get(PGPIO_PUSH_B);
    data->trigger_l     = !gpio_get(PGPIO_PUSH_D);
    data->trigger_zl    = !gpio_get(PGPIO_PUSH_A);
    data->trigger_zr    = !gpio_get(PGPIO_PUSH_C);
    gpio_put(PGPIO_SCAN_D, true);

    data->button_stick_right = !gpio_get(PGPIO_BUTTON_RS);
    data->button_stick_left = !gpio_get(PGPIO_BUTTON_LS);

    data->button_safemode = !gpio_get(PGPIO_BUTTON_MODE);
    data->button_shipping = data->button_stick_right && data->button_stick_left;
    data->button_sync = data->button_plus;
}

int main()
{
    stdio_init_all();
    sleep_ms(100);

    //btinput_capability_reset_flag();

    cb_hoja_hardware_setup();

    gpio_init(PGPIO_ESP_EN);
    cb_hoja_set_bluetooth_enabled(false);

    gpio_init(PGPIO_BUTTON_USB_EN);
    gpio_set_dir(PGPIO_BUTTON_USB_EN, GPIO_OUT);
    gpio_put(PGPIO_BUTTON_USB_EN, 0);

    gpio_init(PGPIO_BUTTON_USB_SEL);
    gpio_set_dir(PGPIO_BUTTON_USB_SEL, GPIO_OUT);
    gpio_put(PGPIO_BUTTON_USB_SEL, 0);

    gpio_init(PGPIO_BUTTON_MODE);
    gpio_set_dir(PGPIO_BUTTON_MODE, GPIO_IN);
    gpio_pull_up(PGPIO_BUTTON_MODE);

    button_data_s tmp = {0};
    cb_hoja_read_buttons(&tmp);

    hoja_config_t _config = {
            .input_method   = INPUT_METHOD_AUTO,
            .input_mode     = INPUT_MODE_LOAD,
        };

    if(!gpio_get(PGPIO_BUTTON_MODE) && tmp.trigger_l)
    {
        reset_usb_boot(0, 0);
    }
    else if (tmp.trigger_r && !gpio_get(PGPIO_BUTTON_MODE))
    {
        _config.input_method = INPUT_METHOD_BLUETOOTH;
        _config.input_mode = INPUT_MODE_BASEBANDUPDATE;
    }
    
    hoja_init(&_config);

}
