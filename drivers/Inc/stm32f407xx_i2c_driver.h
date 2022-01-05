/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 13-Nov-2021
 *      Author: pro
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 * */

typedef struct{

	uint32_t I2C_SCLKSpeed ;	// refer @I2C_SCLK_SPEED
	uint8_t	I2C_DeviceAddress ; // mentioned by the user (no refrence)
	uint8_t	I2C_ACKControl    ;	// refer @I2C_ACK_CONTROL
	uint8_t	I2C_FMDutyCycle   ;	// refer @I2C_DUTY_CYCLE


}I2C_Config_t ;


/*
 * Handle structure for I2Cx peripheral
 * */

typedef struct{
	I2C_RegDef_t   *pI2Cx 	 ;		// this holds the base address of i2cx(1 ,2,3,4) peripheral
	I2C_Config_t   I2CConfig ;		// configuration structure reference
	uint8_t			*pTxBuffer ; 	// to store the Tx buffer address
	uint8_t			*pRxBuffer ; 	// to store the Rx buffer address
	uint32_t		 TxLen     ; 	// to store the Tx len
	uint32_t		 RxLen     ;  	// to store the Rx len
	uint8_t			 TxRxState ;	// to store communicatoopn state
	uint8_t			 DevAddr   ; 	// to store s;ave / Device address
	uint32_t		 RxSize    ;    // to store Rx Size
	uint8_t			 Sr ; 			// to store Repeated start value

}I2C_Handle_t;


// I2C Configuration macros

	// I2C_SCLK_SPEED

#define I2C_SCLK_SPEED_SM	100000
#define I2C_SCLK_SPEED_FM4K	400000
#define I2C_SCLK_SPEED_FM2K	200000
#define I2C_SCLK_SPEED_FM3K	300000

	 // I2C_ACK_CONTROL

#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE 	0

	// I2C_DUTY_CYCLE

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

#define  I2C_ENABLE_SR		ENABLE
#define  I2C_DISABLE_SR		DISABLE

/*I2C driver apis */

void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi) ;

// init and deinit apis
void I2C_Init(I2C_Handle_t *pI2CHandle) ;
void I2C_DeInit(I2C_RegDef_t *pI2Cx) ;
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) ;
uint8_t I2C_GetFlagStatus(I2C_RegDef_t * pI2Cx ,uint32_t I2C_FLAG) ;
// i2c send and receive apis
void I2C_Send(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer , uint32_t length , uint8_t SlaveAddress , uint8_t rep_start) ;
void I2C_Recieve(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer , uint32_t length , uint8_t SlaveAddress ,uint8_t rep_start) ;

// i2c interrupt configuration

void I2C_IRQInterruptConfig(uint8_t IRQ_Number , uint8_t EnorDi ) ;
void I2C_IRQPriorityConfig(uint8_t IRQ_Number , uint32_t Interrupt_Prority ) ;
void I2C_EV_IRQHandling (I2C_Handle_t *pI2CHandle) ;
void I2C_ER_IRQHandling (I2C_Handle_t *pI2CHandle) ;

// i2c send and receive interrupt mode apis

uint8_t I2C_SendIT(I2C_Handle_t *pI2CHandle , uint8_t *pTxBuffer , uint32_t length , uint8_t SlaveAddr , uint8_t rep_start) ;
uint8_t I2C_ReadIT(I2C_Handle_t *pI2CHandle , uint8_t *pRxBuffer , uint32_t length , uint8_t SlaveAddr , uint8_t rep_start) ;
void I2C_CloseTransmissionIT(I2C_Handle_t *pI2CHandle ) ;
void I2C_CloseRecptionIT(I2C_Handle_t *pI2CHandle ) ;
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle ) ;
void I2C_Close_recieve(I2C_Handle_t *pI2CHandle ) ;
// Application event complete callback api
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle , uint8_t eventcode) ;

/*
 * I2C status flag definition
 */

#define I2C_SB_FLAG			(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG		(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG		(1 << I2C_SR1_BTF)
#define I2C_ADD10_FLAG		(1 << I2C_SR1_ADD10)
#define I2C_STOPF_FLAG		(1 << I2C_SR1_STOPF)
#define I2C_RxNE_FLAG		(1 << I2C_SR1_RxNE)
#define I2C_TxE_FLAG		(1 << I2C_SR1_TxE)
#define I2C_BERR_FLAG		(1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG		(1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG			(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG		(1 << I2C_SR1_OVR)
#define I2C_PECERR_FLAG		(1 << I2C_SR1_PECERR)
#define I2C_TIMEOUT_FLAG	(1 << I2C_SR1_TIMEOUT)
#define I2C_SMBALERT_FLAG 	(1 << I2C_SR1_SMBALERT)


/* I2C status definition  */

#define I2C_READY	 		0
#define I2C_BUSY_IN_TX 		1
#define I2C_BUSY_IN_RX 		2

/* I2C application event complete definition  */

#define I2C_EVENT_TX_CMPLT		0
#define I2C_EVENT_RX_CMPLT		1
#define I2C_EVENT_STOP			2

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
