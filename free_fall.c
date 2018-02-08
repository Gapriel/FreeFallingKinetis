/*
 * free_fall.c
 *
 *  Created on: Feb 2, 2018
 *      Author: Francisco Avelar, Gabriel Santamaria
 */

#include "free_fall.h"
#include "clock_config.h"
#include "fsl_i2c.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "DataTypeDefinitions.h"

#define SENSISITIVITY_2G 0.000244   /**+- 2g acceleration sensitivity value as seen in table 61 from IMU datasheet */
#define I2C_IMU_BAUDRATE 100000
#define I2C0_SCL_PIN 24
#define I2C0_SDA_PIN 25
#define I2C0_SCL_PORT PORTE
#define I2C0_SDA_PORT PORTE
#define RED_LED_PIN 22
#define RED_LED_GPIO GPIOB
#define IMU_VALUE_AROUND_FREE_FALL 0.2
#define IMU_VALUE_SQUARED (IMU_VALUE_AROUND_FREE_FALL * IMU_VALUE_AROUND_FREE_FALL)
#define RED_LED_MASK (1<<RED_LED_PIN)

#define IMU_SLAVE_ADDRESS 0x1D
#define IMU_CONFIG_REGISTER_ADDRESS 0x2A
#define IMU_CONFIG_REGISTER_SIZE 1
#define IMU_REGISTER_DATA_SIZE 1

#define BYTES_TO_ALL_AXES 6
#define IMU_ACCELEROMETER_X_HIGH_REGISTER 0x01
#define ACCELEROMETER_X_DATA 0
#define ACCELEROMETER_Y_DATA 1
#define ACCELEROMETER_Z_DATA 2
#define ACCELEROMETER_X_HIGH 0
#define ACCELEROMETER_X_LOW 1
#define ACCELEROMETER_Y_HIGH 2
#define ACCELEROMETER_Y_LOW 3
#define ACCELEROMETER_Z_HIGH 4
#define ACCELEROMETER_Z_LOW 5
#define ACCELEROMETER_REGISTER_BITS_SIZE 8
#define ACCELEROMETER_2_BIT_ADJUSTMENT 4

#define OUTPUT_HIGH 1
#define IMU_ENABLE 0x01

static i2c_master_transfer_t masterXfer; /**I2C_0 master transfer structure declaration*/
static bool g_MasterCompletionFlag = false; /**I2C_0 master transference completion flag*/
static i2c_master_handle_t g_m_handle;  /**I2C_0 master handler declared*/
static const uint8_t gFrecuency = 8;    /**period corresponding to 4Hz*/
static uint8_t gKinetisFalling = FALSE; /**variable used to know if the Kinetis is falling*/
static int16_t accelerometer[3]; /**where the accelerometer will receive its readings*/
static float x_accelerometer;   /**X axis accelerometer reading variable*/
static float y_accelerometer;   /**Y axis accelerometer reading variable*/
static float z_accelerometer;   /**Z axis accelerometer reading variable*/

/**PIT_0 interrupt service routine, here the PIT assures that the LED blinks @ 4Hz*/
void PIT0_IRQHandler() {
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag); /**pit0 interrupt flag cleared*/
    if (TRUE == gKinetisFalling)    /**if the Kinetis if falling*/
    {
        GPIO_TogglePinsOutput(RED_LED_GPIO, RED_LED_MASK); /**toggles RED LED state*/
    }
}

/**I2C_0 master callback, here the programmmer can add code without messing with the actual I2C_0 ISR*/
static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
                                status_t status, void * userData) {

    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}

/**this function initializes the I2C module*/
static void freeFall_I2Cinit() {
    /**clock enabling*/
    CLOCK_EnableClock(kCLOCK_PortE); /**I2C_0 pins port clock enabling*/
    CLOCK_EnableClock(kCLOCK_I2c0); /**I2C_0 clock enabling*/

    /**I2C_0 pins configuration*/
    port_pin_config_t config_i2c = { kPORT_PullUp, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAlt5, kPORT_UnlockRegister, };
    PORT_SetPinConfig(I2C0_SCL_PORT, I2C0_SCL_PIN, &config_i2c);/**I2C_0 SCL pin configuration*/
    PORT_SetPinConfig(I2C0_SDA_PORT, I2C0_SDA_PIN, &config_i2c);/**I2C_0 DSA pin configuration*/

    /**I2C_0 master configuration*/
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig); /**I2C_0 master default config. obtanined*/
    masterConfig.baudRate_Bps = I2C_IMU_BAUDRATE;
    I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

    /**I2C_0 master handler creation*/
    I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback,
                                   NULL);
}

/**this function initializes the PIT module*/
static void freeFall_pitInit() {
    /**clock enabling*/
    CLOCK_EnableClock(kCLOCK_Pit0); /**PIT_0 module clock enabling*/

    /**PIT_0 module configuration*/
    pit_config_t config_pit;
    PIT_GetDefaultConfig(&config_pit);
    PIT_Init(PIT, &config_pit);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, CLOCK_GetBusClkFreq() / gFrecuency); /**pit timer set to interrupt every second*/

    /**PIT_0 module interrupts enabling*/
    NVIC_EnableIRQ(PIT0_IRQn);      /**PIT 0 interrupt enabled*/
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

    /**PIT_0 timer start*/
    PIT_StartTimer(PIT, kPIT_Chnl_0);
}

/**this function initializes the GPIO module*/
static void freeFall_gpioLedsInit() {
    /**led port clock gating*/
    CLOCK_EnableClock(kCLOCK_PortB);

    /**led ports configuration*/
    port_pin_config_t config_led = { kPORT_PullDisable, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };
    PORT_SetPinConfig(PORTB, RED_LED_PIN, &config_led);  /**Red led configuration*/

    /**leds gpio configuration*/
    gpio_pin_config_t led_config_gpio = { kGPIO_DigitalOutput, OUTPUT_HIGH };
    GPIO_PinInit(GPIOB, RED_LED_PIN, &led_config_gpio);
}

/**this function enables the IMU working*/
static void freeFall_IMUinit() {
    /**I2C_0 data block definition*/
    uint8_t data_buffer = IMU_ENABLE;
    masterXfer.slaveAddress = IMU_SLAVE_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = IMU_CONFIG_REGISTER_ADDRESS;
    masterXfer.subaddressSize = IMU_CONFIG_REGISTER_SIZE;
    masterXfer.data = &data_buffer;
    masterXfer.dataSize = IMU_REGISTER_DATA_SIZE;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /**I2C_0 data block transmission*/
    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
    while (!g_MasterCompletionFlag)
    {
    }
    g_MasterCompletionFlag = false;
}

/**this function reads the IMU accelerometer (x,y,z LS and MS registers)*/
static void freeFall_readAccelerometer() {
    uint8_t buffer[BYTES_TO_ALL_AXES]; /**here the read register will hold the values obtained subsequently*/

    /**I2C_0 data block definition*/
    masterXfer.slaveAddress = IMU_SLAVE_ADDRESS;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = IMU_ACCELEROMETER_X_HIGH_REGISTER;
    masterXfer.subaddressSize = IMU_REGISTER_DATA_SIZE;
    masterXfer.data = buffer;
    masterXfer.dataSize = BYTES_TO_ALL_AXES;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    /**I2C_0 data block transmission*/
    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
    while (!g_MasterCompletionFlag)
    {
    }
    g_MasterCompletionFlag = false;

    /**MSB and LSB bytes joined together to form the true axis reading*/
    accelerometer[ACCELEROMETER_X_DATA] = buffer[ACCELEROMETER_X_HIGH]
            << ACCELEROMETER_REGISTER_BITS_SIZE | buffer[ACCELEROMETER_X_LOW]; /**X axis*/
    accelerometer[ACCELEROMETER_Y_DATA] = buffer[ACCELEROMETER_Y_HIGH]
            << ACCELEROMETER_REGISTER_BITS_SIZE | buffer[ACCELEROMETER_Y_LOW]; /**Y axis*/
    accelerometer[ACCELEROMETER_Z_DATA] = buffer[ACCELEROMETER_Z_HIGH]
            << ACCELEROMETER_REGISTER_BITS_SIZE | buffer[ACCELEROMETER_Z_LOW]; /**Z axis*/
}

/**this function converts the read value to an actual functional value*/
static void IMU_format() {

    /*
     * The value is adjusted according to the configured sensitivity
     */
    x_accelerometer = accelerometer[ACCELEROMETER_X_DATA] * SENSISITIVITY_2G;
    y_accelerometer = accelerometer[ACCELEROMETER_Y_DATA] * SENSISITIVITY_2G;
    z_accelerometer = accelerometer[ACCELEROMETER_Z_DATA] * SENSISITIVITY_2G;

    /*
     * 2 exceeding bits adjustment because of miss-alinment
     */
    x_accelerometer /= ACCELEROMETER_2_BIT_ADJUSTMENT;
    y_accelerometer /= ACCELEROMETER_2_BIT_ADJUSTMENT;
    z_accelerometer /= ACCELEROMETER_2_BIT_ADJUSTMENT;

}

/**this function calls the modules initialization functions*/
void freeFall_modulesInit() {
    freeFall_I2Cinit();         /*I2C module initialization*/
    freeFall_IMUinit();         /**accelerometer enabling*/
    freeFall_gpioLedsInit();    /**PIT module initialization*/
    freeFall_pitInit();         /**LED GPIO module initialization*/
}

/**this function bears the logic responsible for detecting if the Kinetis is falling*/
void freeFall_fallDetection() {

    /**the axis values are read and properly formated*/
    freeFall_readAccelerometer();
    IMU_format();

    /**sum of axis magnitudes in order to obtain the vector magnitude*/
    float magnitude_squared = x_accelerometer * x_accelerometer
            + y_accelerometer * y_accelerometer
            + z_accelerometer * z_accelerometer;

    /**if the accelerations approximates zero,*/
    if ((IMU_VALUE_SQUARED) > magnitude_squared)
    {
        /**then the LED starts to blink*/
        GPIO_WritePinOutput(RED_LED_GPIO, RED_LED_PIN, LED_ON);
        gKinetisFalling = TRUE;
    } else
    {
        /**if it's not falling, the LED stops blinking*/
        GPIO_WritePinOutput(RED_LED_GPIO, RED_LED_PIN, LED_OFF);
        gKinetisFalling = FALSE;
    }
}
