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
#include <stdio.h>

static i2c_master_transfer_t masterXfer;
static volatile bool g_MasterCompletionFlag = false;
static i2c_master_handle_t g_m_handle;
static const uint8_t gFrecuency = 8;    //period corresponding to 4Hz
static uint8_t gKinetisFalling = FALSE; //variable used to know if the Kinetis is falling
static int16_t accelerometer[3]; //where the accelerometer will receive its readings
static bool FreeFall = FALSE;
#define SENSISITIVITY_2G 0.000244
#define A_FFMT_SRC 0x16
#define FREE_FALL_PIN_SRC 7
#define FREE_FALL_PIN_MASK (1 << FREE_FALL_PIN_SRC)

float x_accelerometer ;
float y_accelerometer ;
float z_accelerometer ;

#define DEBUG 0


static void formato_IMU();

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
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, CLOCK_GetBusClkFreq() / gFrecuency); //pit timer set to interrupt every second

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


static void freeFall_IMUinit() {
    uint8_t data_buffer = 0x01;
    masterXfer.slaveAddress = 0x1D;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddress = 0x2A;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &data_buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
    while (!g_MasterCompletionFlag)
    {
    }
    g_MasterCompletionFlag = false;


}

static void freeFall_readAccelerometer() {
#if DEBUG
    uint8_t buffer[6];
    masterXfer.slaveAddress = 0x1D;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = 0x01;
    masterXfer.subaddressSize = 1;
    masterXfer.data = buffer;
    masterXfer.dataSize = 6;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
    while (!g_MasterCompletionFlag)
    {
    }
    g_MasterCompletionFlag = false;

    accelerometer[0] = buffer[0] << 8 | buffer[1];
    accelerometer[1] = buffer[2] << 8 | buffer[3];
    accelerometer[2] = buffer[4] << 8 | buffer[5];
#else
    uint8_t data_buffer;
    masterXfer.slaveAddress = 0x1D;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddress = A_FFMT_SRC;
    masterXfer.subaddressSize = 1;
    masterXfer.data = &data_buffer;
    masterXfer.dataSize =1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_m_handle, &masterXfer);
    while (!g_MasterCompletionFlag)
    {
    }
    g_MasterCompletionFlag = false;
    FreeFall = (bool) ((data_buffer & FREE_FALL_PIN_MASK) >> FREE_FALL_PIN_SRC );
#endif
}

void freeFall_modulesInit() {
    freeFall_I2Cinit();         //I2C module initialization
    freeFall_IMUinit();         //accelerometer enabling
    freeFall_gpioLedsInit();    //PIT module initialization
    freeFall_pitInit();         //LED GPIO module initialization
}



void freeFall_fallDetection() {
    freeFall_readAccelerometer();
    formato_IMU();
    if(TRUE ==  FreeFall){
    	gKinetisFalling = TRUE;
    }else
    {
    	gKinetisFalling = FALSE;
    	GPIO_WritePinOutput(GPIOB, 1<<22, 1);
    }
#if DEBUG
    printf("Hola aaa  %f \n", y_accelerometer);
#endif
}


static void formato_IMU(){

	/*
	 * Se ajusta el el valor tomado de acuerdo a la sensibilidad configurada.
	 */
	x_accelerometer = accelerometer[0] * SENSISITIVITY_2G;
	y_accelerometer = accelerometer[1] * SENSISITIVITY_2G;
	z_accelerometer = accelerometer[2] * SENSISITIVITY_2G;

	/*
	 * Ajuste de 2 bits sobrantes debido al acomodo de la parte baja de la información.
	 */
	x_accelerometer /=4;
	y_accelerometer /=4;
	z_accelerometer /=4;
}
