#define X_AXIS_CONFIG 0xD0
#define Y_AXIS_CONFIG 0xF0
#define BUFFER_TO_UINT16(buffer) (uint16_t)(((buffer[0] & 0x07) << 9) | buffer[1] << 1 | buffer[2] >> 7)

#define PGPIO_BUTTON_RS   0
#define PGPIO_BUTTON_LS   15
#define PGPIO_BUTTON_MODE 2

#define PGPIO_SCAN_A    13
#define PGPIO_SCAN_B    10
#define PGPIO_SCAN_C    7
#define PGPIO_SCAN_D    9

#define PGPIO_PUSH_A    6
#define PGPIO_PUSH_B    8
#define PGPIO_PUSH_C    11
#define PGPIO_PUSH_D    12

// Analog L Trigger ADC
#define PADC_LT 1
// Analog R Trigger ADC
#define PADC_RT 0

// Analog L Trigger GPIO
#define PGPIO_LT 27
// Analog R Trigger GPIO
#define PGPIO_RT 26

#define PGPIO_RUMBLE_MAIN   24
#define PGPIO_RUMBLE_BRAKE  25

// SPI ADC CLK pin
  #define PGPIO_SPI_CLK 18
  // SPI ADC TX pin
  #define PGPIO_SPI_TX  19
  // SPI ADC RX pin
  #define PGPIO_SPI_RX  20

  // Left stick ADC Chip Select
  #define PGPIO_LS_CS   16
  // Right stick ADC Chip Select
  #define PGPIO_RS_CS   22

  #define PGPIO_IMU0_CS 17
  #define PGPIO_IMU1_CS 21

  #define PGPIO_BUTTON_USB_SEL 1
  #define PGPIO_BUTTON_USB_EN 27
  #define PGPIO_ESP_EN 14

  