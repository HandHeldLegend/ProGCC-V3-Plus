#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#define HOJA_BT_LOGGING_DEBUG 0

// Device stuff
//#define HOJA_DEVICE_ID  0xA004 // A002 is 3+, A003 is 3+ (Haptic), A004 is 3.1

// GPIO definitions
#define HOJA_SERIAL_PIN 3
#define HOJA_CLOCK_PIN 4
#define HOJA_LATCH_PIN 5

#define HOJA_FW_VERSION 0x0A30

// Haptic
#if (HOJA_DEVICE_ID == 0xA002)
    #define HOJA_PRODUCT        "ProGCC 3+"
    #define HOJA_CAPABILITY_RUMBLE_ERM 1
    #define HOJA_CAPABILITY_RUMBLE_LRA 0
#elif (HOJA_DEVICE_ID == 0xA004)
    #define HOJA_PRODUCT        "ProGCC 3.1"
    #define HOJA_CAPABILITY_RUMBLE_ERM 0
    #define HOJA_CAPABILITY_RUMBLE_LRA 1
#elif (HOJA_DEVICE_ID == 0xA005)
    #define HOJA_PRODUCT        "ProGCC 3.2"
    #define HOJA_CAPABILITY_RUMBLE_ERM 0
    #define HOJA_CAPABILITY_RUMBLE_LRA 1
#endif

    // RGB Stuff
    #define HOJA_RGB_PIN 23
    #define HOJA_RGB_COUNT 32
    #define HOJA_RGBW_EN 0

    // If we do not have native analog triggers
    // set this to zero
    #define HOJA_ANALOG_TRIGGERS 0

    // Sets the analog light trigger level for SP function
    #define HOJA_ANALOG_LIGHT 50

    // URL that will display to open a config tool
    #define HOJA_WEBUSB_URL     "handheldlegend.github.io/hoja_config"
    #define HOJA_MANUFACTURER   "HHL"
    
    #define HOJA_CAPABILITY_ANALOG_STICK_L 1
    #define HOJA_CAPABILITY_ANALOG_STICK_R 1
    #define HOJA_CAPABILITY_ANALOG_TRIGGER_L 0
    #define HOJA_CAPABILITY_ANALOG_TRIGGER_R 0

    #define HOJA_CAPABILITY_BLUETOOTH 1
    #define HOJA_CAPABILITY_BATTERY 1
    #define HOJA_CAPABILITY_RGB 1
    #define HOJA_CAPABILITY_GYRO 1

    #define HOJA_CAPABILITY_NINTENDO_SERIAL 1
    #define HOJA_CAPABILITY_NINTENDO_JOYBUS 1

    #define HOJA_POWER_CONSUMPTION_RATE 150 // mA
    #define HOJA_POWER_CONSUMPTION_SOURCE 1000 // mAh

    #if ( (HOJA_DEVICE_ID == 0xA002) | (HOJA_DEVICE_ID== 0xA003) )
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

        #define HOJA_RGB_GROUP_L {-1}
        #define HOJA_RGB_GROUP_ZL {-1}
        #define HOJA_RGB_GROUP_R {-1}
        #define HOJA_RGB_GROUP_ZR {-1}
        #define HOJA_RGB_GROUP_PLAYER {-1}
        
    #elif (HOJA_DEVICE_ID == 0xA004) || (HOJA_DEVICE_ID == 0xA005) 
        #define HOJA_CAPABILITY_BLUETOOTH_OPTIONAL 1
    
        #define HOJA_RGB_GROUP_RS       {0, 1, 2, 3}
        #define HOJA_RGB_GROUP_PLAYER   {7, 6, 5, 4}
        #define HOJA_RGB_GROUP_LS       {8, 9, 10, 11}
        #define HOJA_RGB_GROUP_DPAD     {12, 13, 14, 15}
        #define HOJA_RGB_GROUP_MINUS    {16}
        #define HOJA_RGB_GROUP_CAPTURE  {17}
        #define HOJA_RGB_GROUP_HOME     {18}
        #define HOJA_RGB_GROUP_PLUS     {19}
        #define HOJA_RGB_GROUP_Y        {20}
        #define HOJA_RGB_GROUP_X        {21}
        #define HOJA_RGB_GROUP_A        {22}
        #define HOJA_RGB_GROUP_B        {23}

        #define HOJA_RGB_GROUP_L        {26}
        #define HOJA_RGB_GROUP_ZL       {27}
        #define HOJA_RGB_GROUP_R        {25}
        #define HOJA_RGB_GROUP_ZR       {24}
    #endif

    #define HOJA_I2C_BUS i2c0
    
    #define HOJA_I2C_SDA 28     // ok
    #define HOJA_I2C_SCL 29     // ok

    // ---------------------------------
    // ---------------------------------

    // SPI HAL Setup
    #define HOJA_SPI_0_ENABLE     1
    #define HOJA_SPI_0_GPIO_CLK   18
    #define HOJA_SPI_0_GPIO_MOSI  19
    #define HOJA_SPI_0_GPIO_MISO  20
    #define SPI_INSTANCE_0        0

    // I2C HAL Setup
    #define HOJA_I2C_0_ENABLE       1
    #define HOJA_I2C_0_GPIO_SDA     28
    #define HOJA_I2C_0_GPIO_SCL     29
    #define I2C_INSTANCE_0          0

    // IMU Driver Setup
    #define IMU_DRIVER_LSM6DSR 2     // 2 Sensors
    #define HOJA_IMU_CHAN_A_DRIVER IMU_DRIVER_LSM6DSR
    #define HOJA_IMU_CHAN_B_DRIVER IMU_DRIVER_LSM6DSR

    #define HOJA_IMU_CHAN_A_CS_PIN          17
    #define HOJA_IMU_CHAN_A_SPI_INSTANCE    0
    #define HOJA_IMU_CHAN_A_INVERT_FLAGS    0b100100

    #if ( (HOJA_DEVICE_ID == 0xA002) | (HOJA_DEVICE_ID == 0xA003) )
        #define HOJA_IMU_CHAN_B_CS_PIN 21    // ok
    #elif (HOJA_DEVICE_ID == 0xA004) | (HOJA_DEVICE_ID == 0xA005)
        #define HOJA_IMU_CHAN_B_CS_PIN 25    // ok
    #endif

    #define HOJA_IMU_CHAN_B_SPI_INSTANCE    0
    #define HOJA_IMU_CHAN_B_INVERT_FLAGS    0b010010
    // ---------------------------------
    // ---------------------------------

    // ADC Driver Setup
    #define ADC_DRIVER_MCP3002          2   // 2 MCP3002 chips

    #if(HOJA_DEVICE_ID == 0xA004)
        #define ADC_SMOOTHING_STRENGTH      4
    #else 
        #define ADC_SMOOTHING_STRENGTH      0 
    #endif

    #define HOJA_ADC_LX_DRIVER          ADC_DRIVER_MCP3002
    #define HOJA_ADC_LX_CHANNEL         0
    #define HOJA_ADC_LX_SPI_INSTANCE    0
    #define HOJA_ADC_LX_CS_PIN          16 

    #define HOJA_ADC_LY_DRIVER          ADC_DRIVER_MCP3002
    #define HOJA_ADC_LY_CHANNEL         1
    #define HOJA_ADC_LY_SPI_INSTANCE    0
    #define HOJA_ADC_LY_CS_PIN          16 

    #define HOJA_ADC_RX_DRIVER          ADC_DRIVER_MCP3002
    #define HOJA_ADC_RX_CHANNEL         0
    #define HOJA_ADC_RX_SPI_INSTANCE    0
    #define HOJA_ADC_RX_CS_PIN          22 

    #define HOJA_ADC_RY_DRIVER          ADC_DRIVER_MCP3002
    #define HOJA_ADC_RY_CHANNEL         1
    #define HOJA_ADC_RY_SPI_INSTANCE    0
    #define HOJA_ADC_RY_CS_PIN          22 
    // ---------------------------------
    // ---------------------------------

    // Haptic Driver Setup
    #if (HOJA_DEVICE_ID == 0xA004) // ProGCC 3.1
        #define HOJA_CONFIG_HDRUMBLE    1
        #define HAPTIC_DRIVER_DRV2605L  1
        #define HAPTIC_DRIVER_DRV2605L_I2C_INSTANCE 0

        #define HOJA_HDRUMBLE_CHAN_A_PIN 21
        // UNUSED #define HOJA_HDRUMBLE_CHAN_B_PIN 24
    #elif (HOJA_DEVICE_ID == 0xA005) // ProGCC 3.2
        // No external driver used
        #define HOJA_CONFIG_HDRUMBLE     1
        #define HOJA_HDRUMBLE_CHAN_A_PIN 21
        #define HOJA_HDRUMBLE_CHAN_B_PIN 24
    #endif
    // ---------------------------------
    // ---------------------------------

    // USB Mux Driver Setup
    #define USB_MUX_DRIVER_PI3USB4000A  1
    #define HOJA_USB_MUX_DRIVER         USB_MUX_DRIVER_PI3USB4000A
    #define USB_MUX_DRIVER_ENABLE_PIN   0
    #define USB_MUX_DRIVER_SELECT_PIN   0

    // Bluetooth Driver Setup
    #define BLUETOOTH_DRIVER_ESP32HOJA      1
    #define HOJA_BLUETOOTH_DRIVER           BLUETOOTH_DRIVER_ESP32HOJA
    #define BLUETOOTH_DRIVER_I2C_INSTANCE   0
    #define BLUETOOTH_DRIVER_ENABLE_PIN     0
    // ---------------------------------
    // ---------------------------------

    // Battery Driver Setup
    #define BATTERY_DRIVER_BQ25810      1
    #define HOJA_BATTERY_DRIVER         BATTERY_DRIVER_BQ25810
    #define HOJA_BATTERY_I2C_INSTANCE   0
    // ---------------------------------
    // ---------------------------------

    // Device Information Setup 
    #define HOJA_DEVICE_NAME    HOJA_PRODUCT
    #define HOJA_DEVICE_MAKER   HOJA_MANUFACTURER
    // ---------------------------------
    // ---------------------------------

    // Static memory elements setup
    #define HOJA_BUTTONS_SUPPORTED_MAIN     0b1111111111111111 // ALL buttons supported
    #define HOJA_BUTTONS_SUPPORTED_SYSTEM   0b111 // Home, Capture, Sync

    // RGB Setup
    #if ( (HOJA_DEVICE_ID == 0xA002) | (HOJA_DEVICE_ID== 0xA003) )
        #define HOJA_RGB_GROUPS_NUM 15
        #define HOJA_RGB_GROUP_NAMES { \
            {"A"}, {"B"}, {"X"}, {"Y"}, \
            {"D-Pad"}, {"L Stick"}, {"R Stick"}, \
            {"L"}, {"R"}, {"ZL"}, {"ZR"}, \
            {"Home"}, {"Capture"}, \
            {"Plus"}, {"Minus"} \
        }

        // Corresponds to the group names in order
        // Filled out with any LED index corresponding to a group
        #define HOJA_RGB_GROUPINGS { \
            {18}, {19}, {17}, {16}, \
            {8, 9, 10, 11}, {4, 5, 6, 7}, {0, 1, 2, 3}, \
            {20}, {22}, {21}, {23}, \
            {14}, {13}, \
            {15}, {12} \
        }
    #elif (HOJA_DEVICE_ID == 0xA004) || (HOJA_DEVICE_ID == 0xA005)
        #define HOJA_RGB_GROUPS_NUM 16
        #define HOJA_RGB_GROUP_NAMES { \
            {"A"}, {"B"}, {"X"}, {"Y"}, \
            {"D-Pad"}, {"L Stick"}, {"R Stick"}, \
            {"L"}, {"R"}, {"ZL"}, {"ZR"}, \
            {"Home"}, {"Capture"}, \
            {"Plus"}, {"Minus"}, {"Player"} \
        }
        #define HOJA_RGB_PLAYER_GROUP_IDX 15

        // Corresponds to the group names in order
        // Filled out with any LED index corresponding to a group
        #define HOJA_RGB_GROUPINGS { \
            {22}, {23}, {21}, {20}, \
            {12,13,14,15}, {8,9,10,11}, {0,1,2,3}, \
            {26}, {25}, {27}, {24}, \
            {18}, {17}, \
            {19}, {16}, {7,6,5,4} \
        }
    #endif
    // ---------------------------------
    // ---------------------------------

#endif
