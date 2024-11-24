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
#if (HOJA_DEVICE_ID == 0xA003)
    #define HOJA_PRODUCT        "ProGCC 3H"
    #define HOJA_CAPABILITY_RUMBLE_LRA 1
    #define HOJA_CAPABILITY_RUMBLE_ERM 0
// Normal
#elif (HOJA_DEVICE_ID == 0xA002)
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
    #define HOJA_I2CINPUT_ADDRESS 0x76
    #define HOJA_I2C_SDA 28     // ok
    #define HOJA_I2C_SCL 29     // ok


    // SPI HAL Setup
    #define HOJA_SPI_0_ENABLE     1
    #define HOJA_SPI_0_GPIO_CLK   18
    #define HOJA_SPI_0_GPIO_MOSI  19
    #define HOJA_SPI_0_GPIO_MISO  20
    #define SPI_INSTANCE_0        0

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

    //#define HOJA_ADC_CHAN_LT_READ() 0
    //#define HOJA_ADC_CHAN_RT_READ() 0
    //#define HOJA_ADC_CHAN_BATTERY_READ()

    //#define HOJA_IMU_CHAN_A_READ() void()
    //#define HOJA_IMU_CHAN_B_READ() void()

// Defined by cmake #define HOJA_RUMBLE_TYPE HOJA_RUMBLE_TYPE_HAPTIC or HOJA_RUMBLE_TYPE_ERM

#endif
