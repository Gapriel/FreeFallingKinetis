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

#define SENSISITIVITY_2G 0.000244   //+- 2g acceleration sensitivity value as seen in table 61 from IMU datasheet

static i2c_master_transfer_t masterXfer; //I2C_0 master transfer structure declaration
static bool g_MasterCompletionFlag = false; //I2C_0 master transference completion flag
static i2c_master_handle_t g_m_handle;  //I2C_0 master handler declared
static const uint8_t gFrecuency = 8;    //period corresponding to 4Hz
static uint8_t gKinetisFalling = FALSE; //variable used to know if the Kinetis is falling
static int16_t accelerometer[3]; //where the accelerometer will receive its readings
static float x_accelerometer;   //X axis accelerometer reading variable
static float y_accelerometer;   //Y axis accelerometer reading variable
static float z_accelerometer;   //Z axis accelerometer reading variable

//PIT_0 interrupt service routine, here the PIT assures that the LED blinks @ 4Hz
void PIT0_IRQHandler() {
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag); //pit0 interrupt flag cleared
    if (TRUE == gKinetisFalling)    //if the Kinetis if falling
    {
        GPIO_TogglePinsOutput(GPIOB, 1 << 22);  //toggles RED LED state
    }
}

//I2C_0 master callback, here the programmmer can add code without messing with the actual I2C_0 ISR
static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
                                status_t status, void * userData) {

    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}

//this function initializes the I2C module
static void freeFall_I2Cinit() {
    //clock enabling
    CLOCK_EnableClock(kCLOCK_PortE); //I2C_0 pins port clock enabling
    CLOCK_EnableClock(kCLOCK_I2c0); //I2C_0 clock enabling

    //I2C_0 pins configuration
    port_pin_config_t config_i2c = { kPORT_PullUp, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAlt5, kPORT_UnlockRegister, };
    PORT_SetPinConfig(PORTE, 24, &config_i2c);	//I2C_0 SCL pin configuration
    PORT_SetPinConfig(PORTE, 25, &config_i2c);	//I2C_0 DSA pin configuration

    //I2C_0 master configuration
    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig(&masterConfig); //I2C_0 master default config. obtanined
    masterConfig.baudRate_Bps = 100000;
    I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

    //I2C_0 master handler creation
    I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback,
    NULL);
}

//this function initializes the PIT module
static void freeFall_pitInit() {
    //clock enabling
    CLOCK_EnableClock(kCLOCK_Pit0); //PIT_0 module clock enabling

    //PIT_0 module configuration
    pit_config_t config_pit;
    PIT_GetDefaultConfig(&config_pit);
    PIT_Init(PIT, &config_pit);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, CLOCK_GetBusClkFreq() / gFrecuency); //pit timer set to interrupt every second

    //PIT_0 module interrupts enabling
    NVIC_EnableIRQ(PIT0_IRQn);      //PIT 0 interrupt enabled
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

    //PIT_0 timer start
    PIT_StartTimer(PIT, kPIT_Chnl_0);
}

//this function initializes the GPIO module
static void freeFall_gpioLedsInit() {
    //led port clock gating
    CLOCK_EnableClock(kCLOCK_PortB);

    //led ports configuration
    port_pin_config_t config_led = { kPORT_PullDisable, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAsGpio, kPORT_UnlockRegister };
    PORT_SetPinConfig(PORTB, 22, &config_led);  //Red led configuration

    //leds gpio configuration
    gpio_pin_config_t led_config_gpio = { kGPIO_DigitalOutput, 1 };
    GPIO_PinInit(GPIOB, 22, &led_config_gpio);
}

//this function enables the IMU working
static void freeFall_IMUinit() {
    //I2C_0 data block definition
    uint8_t data_buffer = 0x01;
    masterXfer.slaveAddress = 0x1D;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0x2A;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &data_buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    //I2C_0 data block transmission
    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
    while (!g_MasterCompletionFlag)
    {
    }
    g_MasterCompletionFlag = false;
}

//this function reads the IMU accelerometer (x,y,z LS and MS registers)
static void freeFall_readAccelerometer() {
    uint8_t buffer[6];  //here the read register will hold the values obtained subsequently

    //I2C_0 data block definition
    masterXfer.slaveAddress = 0x1D;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = 0x01;
    masterXfer.subaddressSize = 1;
    masterXfer.data = buffer;
    masterXfer.dataSize = 6;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    //I2C_0 data block transmission
    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
    while (!g_MasterCompletionFlag)
    {
    }
    g_MasterCompletionFlag = false;

    //MSB and LSB bytes joined together to form the true axis reading
    accelerometer[0] = buffer[0] << 8 | buffer[1];  //X axis
    accelerometer[1] = buffer[2] << 8 | buffer[3];  //Y axis
    accelerometer[2] = buffer[4] << 8 | buffer[5];  //Z axis
}

//this function converts the read value to an actual functional value
static void IMU_format() {

    /*
     * The value is adjusted according to the configured sensitivity
     */
    x_accelerometer = accelerometer[0] * SENSISITIVITY_2G;
    y_accelerometer = accelerometer[1] * SENSISITIVITY_2G;
    z_accelerometer = accelerometer[2] * SENSISITIVITY_2G;

    /*
     * 2 exceeding bits adjustment because of miss-alinment
     * Ajuste de 2 bits sobrantes debido al acomodo de la parte baja de la información.
     */
    x_accelerometer /= 4;
    y_accelerometer /= 4;
    z_accelerometer /= 4;

}

//this function calls the modules initialization functions
void freeFall_modulesInit() {
    freeFall_I2Cinit();         //I2C module initialization
    freeFall_IMUinit();         //accelerometer enabling
    freeFall_gpioLedsInit();    //PIT module initialization
    freeFall_pitInit();         //LED GPIO module initialization
}

//this function bears the logic responsible for detecting if the Kinetis is falling
void freeFall_fallDetection() {

    //the axis values are read and properly formated
    freeFall_readAccelerometer();
    IMU_format();

    //sum of axis magnitudes in order to obtain the vector magnitude
    float magnitude = x_accelerometer * x_accelerometer
            + y_accelerometer * y_accelerometer
            + z_accelerometer * z_accelerometer;

    //if the accelerations approximates zero,
    if ((0.04) > magnitude)
    {
        //then the LED starts to blink
        GPIO_WritePinOutput(GPIOB, 22, 0);
        gKinetisFalling = TRUE;
    } else
    {
        //if it's not falling, the LED stops blinking
        GPIO_WritePinOutput(GPIOB, 22, 1);
        gKinetisFalling = FALSE;
    }
}
