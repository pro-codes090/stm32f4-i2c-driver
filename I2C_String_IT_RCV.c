/*
 * I2C_String_IT_RCV.c
 *
 *  Created on: 31-Dec-2021
 *      Author: pro
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_i2c_driver.h"

I2C_Handle_t I2Chandle;

void I2C2_GPIOInits(void) {
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//SCLK
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11 ;

	GPIO_Init(&I2CPins);

}

void I2C2_Inits(void) {
	I2Chandle.pI2Cx = I2C2;
	I2Chandle.I2CConfig.I2C_ACKControl = I2C_ACK_ENABLE;
	I2Chandle.I2CConfig.I2C_DeviceAddress = 0;
	I2Chandle.I2CConfig.I2C_SCLKSpeed = I2C_SCLK_SPEED_SM;
	I2Chandle.I2CConfig.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C_Init(&I2Chandle);
}

void GPIO_ButtonInit(void) {
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOPType = 0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPUPDControl = GPIO_NO_PUPD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinAltFunMode = 0;

	GPIO_Init(&GPIOBtn);

}
int main(void) {

	uint8_t commandcode;

	uint8_t len;
//
//	//rcv buffer
	uint8_t rcv_buf[32]= "pppppppppprrrrrrrrrroooooooooo@#";

	GPIO_ButtonInit();

	I2C2_GPIOInits();

	I2C2_Inits();

	// I2C interrupt configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_EV,ENABLE) ;
	I2C_IRQInterruptConfig(IRQ_NO_I2C2_ER,ENABLE) ;

	// enable the spi peripheral
	I2C_PeripheralControl(I2C2, ENABLE);

	// ack bit has to be enabled after the peripheral is active(PE = 1)
	I2Chandle.pI2Cx->CR1 |= ( I2C_ACK_ENABLE << I2C_CR1_ACK ) ;

	printf("Application starts ") ;

	while (1) {

		//wait till button is pressed
		while (!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

		for (int i = 0; i < 50000; i++) {

		}

		commandcode = 0x51 ;

		while( I2C_SendIT(&I2Chandle,&commandcode , 1, 0x68 , I2C_DISABLE_SR ) != I2C_READY ) ;

		while( I2C_ReadIT(&I2Chandle,&len, 1, 0x68 , I2C_DISABLE_SR) != I2C_READY ) ;

	}
	//close the communication by disabling the peripherals
	I2C_PeripheralControl(I2C2, DISABLE);
	while (1);

	return 0;
}

void I2C2_EV_IRQHandler(){
I2C_EV_IRQHandling(&I2Chandle) ;
}

void I2C2_ER_IRQHandler(){
I2C_ER_IRQHandling(&I2Chandle) ;
}


