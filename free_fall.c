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

static i2c_master_transfer_t masterXfer;
static uint8_t data_buffer = 0x01;
static volatile bool g_MasterCompletionFlag = false;

static void i2c_master_callback(I2C_Type *base, i2c_master_handle_t *handle,
		status_t status, void * userData) {

	if (status == kStatus_Success) {
		g_MasterCompletionFlag = true;
	}
}

void freeFall_I2Cinit() {
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
	I2C_MasterGetDefaultConfig(&masterConfig);//I2C_0 master default config. obtanined
	masterConfig.baudRate_Bps = 100000;
	I2C_MasterInit(I2C0, &masterConfig, CLOCK_GetFreq(kCLOCK_BusClk));

	//I2C_0 master handler creation
	i2c_master_handle_t g_m_handle;
	I2C_MasterTransferCreateHandle(I2C0, &g_m_handle, i2c_master_callback,
			NULL);
}

void freeFall_fallDetection() {

}
