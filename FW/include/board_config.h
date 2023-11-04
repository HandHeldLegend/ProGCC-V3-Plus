#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// Device stuff
#define HOJA_DEVICE_ID  0xA002
#define HOJA_FW_VERSION 0x0A01
#define HOJA_SETTINGS_VERSION 0xA000

// RGB Stuff
#define HOJA_RGB_PIN 23
#define HOJA_RGB_COUNT 20
#define HOJA_RGBW_EN 0

// GPIO definitions
#define HOJA_SERIAL_PIN 3
#define HOJA_CLOCK_PIN 4
#define HOJA_LATCH_PIN 5

// If we do not have native analog triggers
// set this to zero
#define HOJA_ANALOG_TRIGGERS 0

// Sets the analog light trigger level for SP function
#define HOJA_ANALOG_LIGHT 50

// URL that will display to open a config tool
#define HOJA_WEBUSB_URL     "handheldlegend.com"
#define HOJA_MANUFACTURER   "HHL"
#define HOJA_PRODUCT        "ProGCC 3+"

#define HOJA_CAPABILITY_ANALOG_STICK_L 1
#define HOJA_CAPABILITY_ANALOG_STICK_R 1
#define HOJA_CAPABILITY_ANALOG_TRIGGER_L 0
#define HOJA_CAPABILITY_ANALOG_TRIGGER_R 0

#define HOJA_CAPABILITY_BLUETOOTH 1
#define HOJA_CAPABILITY_BATTERY 1
#define HOJA_CAPABILITY_RGB 1
#define HOJA_CAPABILITY_GYRO 1

#define HOJA_CAPABILITY_NINTENDO_SERIAL 0
#define HOJA_CAPABILITY_NINTENDO_JOYBUS 1

#define HOJA_CAPABILITY_RUMBLE 1

#define HOJA_RGB_GROUP_RS       {0, 1, 2, 3}
#define HOJA_RGB_GROUP_LS       {4, 5, 6, 7}
#define HOJA_RGB_GROUP_DPAD     {8, 9, 10, 11}
#define HOJA_RGB_GROUP_MINUS    {12}
#define HOJA_RGB_GROUP_CAPTURE  {13}
#define HOJA_RGB_GROUP_HOME     {14}
#define HOJA_RGB_GROUP_PLUS     {15}
#define HOJA_RGB_GROUP_Y        {16}
#define HOJA_RGB_GROUP_X        {17}
#define HOJA_RGB_GROUP_A        {18}
#define HOJA_RGB_GROUP_B        {19}

#define HOJA_I2C_BUS i2c0
#define HOJA_I2CINPUT_ADDRESS 0x76
#define HOJA_I2C_SDA 28
#define HOJA_I2C_SCL 29

#endif
