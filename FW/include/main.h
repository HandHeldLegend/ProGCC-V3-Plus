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

#define PGPIO_BUTTON_USB_SEL 1    // ok
#define PGPIO_BUTTON_USB_EN 27    // ok
#define PGPIO_ESP_EN 14           // ok

  