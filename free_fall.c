/*
 * free_fall.c
 *
 *  Created on: Feb 2, 2018
 *      Author: gabrielpc
 */

#include "free_fall.h"
#include "clock_config.h"
#include "fsl_i2c.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "DataTypeDefinitions.h"

static i2c_master_transfer_t masterXfer;
static uint8_t data_buffer = 0x01;
static volatile bool g_MasterCompletionFlag = false;
static const uint8_t gFrecuency = 4;    //period corresponding to 4Hz
static const uint8_t gKinetisFalling = FALSE;   //variable used to know if the Kinetis is falling

void PIT0_IRQHandler() {
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag); //pit0 interrupt flag cleared
    if (TRUE == gKinetisFalling)    //if the Kinetis if falling
    {
        GPIO_TogglePinsOutput(GPIOB, 1 << 22);  //toggles RED LED state
    }
}

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
                                status_t status, void * userData) {

    if (status == kStatus_Success)
    {
        g_MasterCompletionFlag = true;
    }
}

static void freeFall_I2Cinit() {
    //clock enabling
    CLOCK_EnableClock(kCLOCK_PortE); //I2C_0 pins port clock enabling
    CLOCK_EnableClock(kCLOCK_I2c0); //I2C_0 clock enabling

    //I2C_0 pins configuration
    port_pin_config_t config_i2c = { kPORT_PullDisable, kPORT_SlowSlewRate,
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
    i2c_master_handle_t g_m_handle;
    I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback,
                                   NULL);
}

static void freeFall_pitInit() {
    //clock enabling
    CLOCK_EnableClock(kCLOCK_Pit0); //PIT_0 module clock enabling

    //PIT_0 module configuration
    pit_config_t config_pit;
    PIT_GetDefaultConfig(&config_pit);
    PIT_Init(PIT, &config_pit);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, CLOCK_GetBusClkFreq()/gFrecuency); //pit timer set to interrupt every second

    //PIT_0 module interrupts enabling
    NVIC_EnableIRQ(PIT0_IRQn);      //PIT 0 interrupt enabled
    PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);

    //PIT_0 timer start
    PIT_StartTimer(PIT, kPIT_Chnl_0);
}

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

void freeFall_modulesInit() {
    freeFall_I2Cinit();         //I2C module initialization
    freeFall_gpioLedsInit();    //PIT module initialization
    freeFall_pitInit();         //LED GPIO module initialization
}

void freeFall_fallDetection() {

}
