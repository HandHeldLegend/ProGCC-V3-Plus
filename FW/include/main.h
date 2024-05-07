#define X_AXIS_CONFIG 0xD0
#define Y_AXIS_CONFIG 0xF0
#define BUFFER_TO_UINT16(buffer) (uint16_t)(((buffer[0] & 0x07) << 9) | buffer[1] << 1 | buffer[2] >> 7)

#define PGPIO_BUTTON_RS   0   // BTN_PWR ok
#define PGPIO_BUTTON_LS   15  // ok
#define PGPIO_BUTTON_MODE 2   // ok

#define PGPIO_SCAN_A    13  // ok
#define PGPIO_SCAN_B    10  // ok
#define PGPIO_SCAN_C    7   // ok
#define PGPIO_SCAN_D    9   // ok

#define PGPIO_PUSH_A    6   // ok
#define PGPIO_PUSH_B    8   // ok
#define PGPIO_PUSH_C    11  // ok
#define PGPIO_PUSH_D    12  // ok


#if (HOJA_DEVICE_ID == 0xA002)
  #define PGPIO_RUMBLE_MAIN   24
  #define PGPIO_RUMBLE_BRAKE  25
#endif

// SPI ADC CLK pin
#define PGPIO_SPI_CLK 18    // ok
// SPI ADC TX pin
#define PGPIO_SPI_TX  19    // ok
// SPI ADC RX pin
#define PGPIO_SPI_RX  20    // ok

// Left stick ADC Chip Select
#define PGPIO_LS_CS   16    // ok
// Right stick ADC Chip Select
#define PGPIO_RS_CS   22    // ok

#define PGPIO_IMU0_CS 17    // ok
#if ( (HOJA_DEVICE_ID == 0xA002) || (HOJA_DEVICE_ID== 0xA003) )
  #define PGPIO_IMU1_CS 21    // ok
#elif (HOJA_DEVICE_ID == 0xA004)
  #define PGPIO_IMU1_CS 25    // ok
#endif

#define PGPIO_BUTTON_USB_SEL 1    // ok
#define PGPIO_BUTTON_USB_EN 27    // ok
#define PGPIO_ESP_EN 14           // ok


void _gpio_put_od(uint gpio, bool level);
  