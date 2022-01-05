/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 13-Nov-2021
 *      Author: pro
 */


#include "stm32f407xx_i2c_driver.h"

static void I2C_Generate_start_condition(I2C_RegDef_t *pI2Cx) {

	pI2Cx->CR1 |= (1 << I2C_CR1_START) ;

}

static void I2C_Generate_Stop_Condition(I2C_RegDef_t *pI2Cx) {

	pI2Cx->CR1 |= (1 << I2C_CR1_STOP ) ;

}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t * pI2Cx ,uint32_t I2C_FLAG) {

	if (pI2Cx->SR1 & I2C_FLAG) {

		return FLAG_SET ;
	} else {

		return FLAG_RESET ;
	}
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle ){

	printf("closing data transfer in interrupt mode \n") ;
	// disable all the interrupts
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN) ;
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN) ;
	printf("interrupts disabled transmission \n") ;
	pI2CHandle->TxRxState = I2C_READY ;
	pI2CHandle->pTxBuffer = NULL ;
	pI2CHandle->TxLen = 0  ;
	printf("handle variables reset transmission \n") ;
}

void I2C_Close_recieve(I2C_Handle_t *pI2CHandle ) {
	printf("closing data reception in interrupt mode \n") ;
	// disable all the interrupts
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN) ;
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN) ;
	printf("interrupts disabled reception \n") ;
	pI2CHandle->TxRxState = I2C_READY ;
	pI2CHandle->pRxBuffer = NULL ;
	pI2CHandle->RxLen = 0  ;
	pI2CHandle->RxSize = 0  ;
	printf("handle variables reset reception \n") ;
	// re-enable the acking
	pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK) ;
	printf("Ack enabled interrupt \n") ;

}

static void I2C_executeAddresPhase(I2C_RegDef_t* pI2Cx , uint8_t SlaveAddress , uint8_t ReadORRight) {

	SlaveAddress = SlaveAddress << 1 ;

	if (ReadORRight == 0) {
		SlaveAddress &= ~(1 << 0 ) ;	// the slave address is write bit combined
	}else if (ReadORRight == 1) {
		SlaveAddress |= (1 << 0 ) ;		// the slave address is Read bit combined
	}
	pI2Cx->DR = SlaveAddress ;
}

static void Clear_ADDR_FLAG(I2C_Handle_t * pI2CHandle){
	uint32_t DummyRead  = 0 ;

	// check for device mode
	if ( (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL) ) ) {
			// device is in master mode
	  if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

		  if (pI2CHandle->RxSize == 1 ) {
			// disable the acking first
			pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK) ;

			// clear the addr flag
			DummyRead =  pI2CHandle->pI2Cx->SR1 ;
			DummyRead =  pI2CHandle->pI2Cx->SR2 ;
			(void)DummyRead ;

		  }
	}  else{
		// device is in transmit mode or don't know
		// clear the addr flag
		DummyRead =  pI2CHandle->pI2Cx->SR1 ;
		DummyRead =  pI2CHandle->pI2Cx->SR2 ;
		(void)DummyRead ;
	}
	}else{
	  // device is in slave mode
		// clear the addr flag straight away
		DummyRead =  pI2CHandle->pI2Cx->SR1 ;
		DummyRead =  pI2CHandle->pI2Cx->SR2 ;
		(void)DummyRead ;
	}
}
void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx , uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_CLOCK_ENABLE();
		}else if (pI2Cx == I2C2) {
			I2C2_CLOCK_ENABLE();
		}else if (pI2Cx == I2C3) {
			I2C3_CLOCK_ENABLE();
		}
	}else{
		if (pI2Cx == I2C1) {
			I2C1_CLOCK_DISABLE();
		}else if (pI2Cx == I2C2){
			I2C2_CLOCK_DISABLE();
		}else if (pI2Cx == I2C3) {
			I2C3_CLOCK_DISABLE();
		}
	}
}

uint32_t RCC_GetPllClk(void ){
	return 0 ;
}

uint32_t RCC_GetPCLK1Vlaue(void ){
	uint16_t AHB_PreScaler[9] = {2,4,8,16,32,64,128,256,512} ;
	uint8_t APB1_PreScaler[4] = {2,4,8,16} ;
	uint32_t psclk ;
	uint8_t clksrc ,  temp  , ahbp , apb1p ;
	uint32_t SystemClk = 0 ;
	clksrc = ((RCC->CFGR >> 2) & 0x3 ); 	// check the clock source

	if (clksrc == 0 ) {
		SystemClk = 16000000 ;	// hsi is selected
	}else if (clksrc == 1 ){
		SystemClk = 8000000 ;	// hse is selected
	}else if (clksrc == 2 ){
		SystemClk = RCC_GetPllClk(); // pll is selected
	}

	// get the value of ahb prescalar value
	temp = ((RCC->CFGR >> 4 ) & 0xf) ;
	if (temp <  8 ) {
		ahbp = 1 ;
	}else  {
		ahbp = AHB_PreScaler[temp - 8] ;
	}
	// get the value of apb1 prescalar value
	temp = ((RCC->CFGR >> 10) & 0x7) ;

	if (temp <  4 ) {
		apb1p = 1 ;
	}else  {
		apb1p = APB1_PreScaler[temp - 4] ;
	}
	psclk =	( (SystemClk / ahbp) / apb1p) ;
	return psclk ;

}

// init and deinit apis
void I2C_Init(I2C_Handle_t *pI2CHandle) {
I2C_PeripheralClockControl(pI2CHandle->pI2Cx, ENABLE) ;

	uint32_t tempreg = 0 ;

	// configuring the cr1 register
	tempreg |=  ( pI2CHandle->I2CConfig.I2C_ACKControl << 10 ) ; // ack control
	pI2CHandle->pI2Cx->CR1 = tempreg ;

	// configuring the freq field of CR2 register
	tempreg = 0 ;
	tempreg |= (RCC_GetPCLK1Vlaue() / 1000000U);
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3f);

	// program the device own address
	tempreg |=  (pI2CHandle->I2CConfig.I2C_DeviceAddress << 1) ;
	tempreg |= (1 << 14 ) ; 	// as per the manual the 14th bit position should be kept at 1
	pI2CHandle->pI2Cx->OAR1 = tempreg ;

	// configure the CCR register
	uint16_t CCRValue = 0 ;
	tempreg = 0 ;
	if (pI2CHandle->I2CConfig.I2C_SCLKSpeed <= I2C_SCLK_SPEED_SM) {

		// i2c is in standard mode
			// by default i2c is configured in stanndard mode
		tempreg |= ( pI2CHandle->I2CConfig.I2C_FMDutyCycle << I2C_CCR_DUTY) ; // select the duty cycle
		CCRValue = RCC_GetPCLK1Vlaue() / (2 * pI2CHandle->I2CConfig.I2C_SCLKSpeed ) ;
		tempreg = (CCRValue & (0xFFF));
		pI2CHandle->pI2Cx->CCR = tempreg ;

	}else {

		// i2c is in fast mode
			// select the fast mode in ccr register
			tempreg |= ( 1 << I2C_CCR_FS) ;
			tempreg |= ( pI2CHandle->I2CConfig.I2C_FMDutyCycle << I2C_CCR_DUTY) ; // select the duty cycle

		// check the duty cycle selected and use 16:9 duty cycle for 400khz

		if (pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_2) {

			CCRValue = RCC_GetPCLK1Vlaue() / (3 * pI2CHandle->I2CConfig.I2C_SCLKSpeed ) ;

		}else if (pI2CHandle->I2CConfig.I2C_FMDutyCycle == I2C_FM_DUTY_16_9) {

			CCRValue = RCC_GetPCLK1Vlaue() / (25 * pI2CHandle->I2CConfig.I2C_SCLKSpeed ) ;

		}

		 tempreg |= ( CCRValue & 0xFFF);
	}

	pI2CHandle->pI2Cx->CCR = tempreg ;

	// trise configurations
	if (pI2CHandle->I2CConfig.I2C_SCLKSpeed == I2C_SCLK_SPEED_SM) {
		// mode is standard mode
		tempreg = ( RCC_GetPCLK1Vlaue() / 1000000U ) + 1 ;

	}else {
		// mode is fast mode
		tempreg = ((RCC_GetPCLK1Vlaue() * 300) / 1000000000U) + 1 ;
	}

	pI2CHandle->pI2Cx->TRISE = ( tempreg & 0x3F) ;

}
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {

	if (pI2Cx == I2C1) {
		I2C1_REG_RESET() ;
	}else if (pI2Cx == I2C2) {
		I2C2_REG_RESET() ;
	}else if (pI2Cx == I2C3) {
		I2C3_REG_RESET() ;
	}

}
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {

	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |=  (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &=  ~(1 << I2C_CR1_PE);
	}


}
// i2c send and receive apis
void I2C_Send(I2C_Handle_t *pI2CHandle , uint8_t *pTxBuffer , uint32_t length , uint8_t SlaveAddress , uint8_t rep_start) {

	// Generate the start condition
	I2C_Generate_start_condition(pI2CHandle->pI2Cx) ;

	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_SB_FLAG))) ; // wait for the SR1 to SET

	// write the address of the slave to the DR
	I2C_executeAddresPhase(pI2CHandle->pI2Cx , SlaveAddress , 0) ;

	// check wether the address phase is completed or not
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_ADDR_FLAG))) ;

	// clear ADDR flag
	Clear_ADDR_FLAG(pI2CHandle) ;
	// send the data until length become zero

	while( length > 0 ){

		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG)) ;
		pI2CHandle->pI2Cx->DR = *pTxBuffer ; // Transferring  1 byte of data to dr register
		pTxBuffer ++ ;
		length -- ;
	}

	// when the length becomes zero wait for the txe and btf flag to become 1 and then end the communication

	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG)) {}
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG)) {}

	if (rep_start == DISABLE) {
		// Generate the Stop condition
		I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx) ;
	}
}
void I2C_Recieve(I2C_Handle_t *pI2CHandle , uint8_t *pRxBuffer , uint32_t length ,  uint8_t SlaveAddress , uint8_t rep_start) {

	// Generate the start condition
	I2C_Generate_start_condition(pI2CHandle->pI2Cx) ;
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_SB_FLAG))) ; // wait for the SR1 to SET
	// write the address of the slave to the DR
	I2C_executeAddresPhase(pI2CHandle->pI2Cx , SlaveAddress , 1) ;
	// check weather the address phase is completed or not
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx , I2C_ADDR_FLAG))) ;

	if (length == 1 ) {
		// disable the acking
		pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK) ;

		// clear the addr flag
		Clear_ADDR_FLAG(pI2CHandle) ;

		// wait until RxNE becomes 1
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG)) ;

		// check for repeated start
		if (rep_start == DISABLE) {
			// generate stop condition
			I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx) ;

		}

		// read the data buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR ;
	}
	if (length >  1) {
		// clear the addr flag
		Clear_ADDR_FLAG(pI2CHandle) ;

		for (uint32_t i = length; i > 0 ; i -- ) {
			// wait until RxNE becomes 1
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG)) ;
			// if last 2 bytes are remaining
			if (i == 2 ) {
				// clear the ack bit
					// disable the acking
				pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK) ;

				// check for repeated start
				if (rep_start == DISABLE) {
					// generate stop condition
					I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx) ;

				}
			}
			// read the data from the dr into the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR ;

			// increment the address
			pRxBuffer++ ;
		}
	}
	if (pI2CHandle->I2CConfig.I2C_ACKControl == I2C_ACK_ENABLE) {
		// enable the acking
		pI2CHandle->pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK) ;
	}
}
// i2c interrupt configuration

void I2C_IRQInterruptConfig(uint8_t IRQ_Number , uint8_t EnorDi ) {
	if (EnorDi == ENABLE ) {
		if (IRQ_Number <= 31) {
			// configure iser 0 register
			*NVIC_ISER0 |= ( 1 << IRQ_Number) ;
		} else if (IRQ_Number > 31 && IRQ_Number < 64){
			// configure iser 1 register
			*NVIC_ISER1 |= ( 1 << ( IRQ_Number % 32 )) ;
		} else if (IRQ_Number >= 64 && IRQ_Number < 96){
			// configure iser 2 register
			*NVIC_ISER2 |= ( 1 << ( IRQ_Number % 64 )) ;
	 }
   }else {
		if (IRQ_Number <= 31) {
			// configure iser 0 register
		*NVIC_ICER0 |= ( 1 << IRQ_Number) ;
		} else if (IRQ_Number > 31 && IRQ_Number < 64){
			// configure iser 1 register
			*NVIC_ICER1 |= ( 1 << ( IRQ_Number % 32 ) ) ;
		} else if (IRQ_Number > 64 && IRQ_Number < 96){
			// configure iser 2 register
			*NVIC_ICER2 |= ( 1 << ( IRQ_Number % 64 )) ;
	}
  }
}
void I2C_IRQPriorityConfig(uint8_t IRQ_Number , uint32_t Interrupt_Prority ) {
	// find the appropriate ipr register
	uint8_t iprx = IRQ_Number / 4 ;
	uint8_t iprx_section = IRQ_Number % 4 ;
	uint8_t shift_ammount = (8 * iprx_section) + NO_PR_BITS_IMPLEMENTED ;
	*(NVIC_IPR_BASE_ADDR + iprx) |= (Interrupt_Prority << shift_ammount) ;

}

// i2c send and receive interrupt mode apis

uint8_t I2C_SendIT(I2C_Handle_t *pI2CHandle , uint8_t *pTxBuffer , uint32_t length , uint8_t SlaveAddr , uint8_t rep_start) {

	uint8_t busystate = pI2CHandle->TxRxState ;

	if ((busystate != I2C_BUSY_IN_TX ) && (busystate != I2C_BUSY_IN_RX ) ) {

		pI2CHandle->DevAddr = SlaveAddr ;
		pI2CHandle->TxLen = length ;
		pI2CHandle->pTxBuffer = pTxBuffer ;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX ;
		pI2CHandle->Sr = rep_start ;
		// generate the start condition
		I2C_Generate_start_condition(pI2CHandle->pI2Cx) ;

	}

	// enable the event interrupt for i2c in cr1 register
	pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN) ;
	pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN) ;
	pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN) ;

	return busystate ;
}

uint8_t I2C_ReadIT(I2C_Handle_t *pI2CHandle , uint8_t *pRxBuffer , uint32_t length , uint8_t SlaveAddr , uint8_t rep_start) {

	uint8_t busystate = pI2CHandle->TxRxState ;

	if ((busystate != I2C_BUSY_IN_TX ) && (busystate != I2C_BUSY_IN_RX ) ) {

		pI2CHandle->DevAddr = SlaveAddr ;
		pI2CHandle->RxLen = length ;
		pI2CHandle->RxSize = length ;
		pI2CHandle->pRxBuffer = pRxBuffer ;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX ;
		pI2CHandle->Sr = rep_start ;
		// generate the start condition
			I2C_Generate_start_condition(pI2CHandle->pI2Cx) ;

	}
	// enable the event and error interrupt for i2c in cr1 register
	pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN) ;
	pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN) ;
	pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN) ;

	return busystate ;
}

//void I2C_CloseTransmissionIT(I2C_Handle_t *pI2CHandle ) {
//
//}
//void I2C_CloseRecptionIT(I2C_Handle_t *pI2CHandle ) {
//
//}


void I2C_EV_IRQHandling (I2C_Handle_t *pI2CHandle) {
	uint32_t temp1 , temp2 , temp3;
	// check for start bit event
	temp1 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_SB) ;
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;

	if (temp1 && temp2) {
		// start bit is enabled communication has started
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {

		I2C_executeAddresPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr, 0) ;

		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

		I2C_executeAddresPhase(pI2CHandle->pI2Cx,pI2CHandle->DevAddr, 1) ;

		}
	}

	// check for ADDR event
	// ADDR event also occurs in slave mode when address math is successful

	temp1 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADDR) ;
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;

	if (temp1 && temp2) {
		// address phase has been completed

		Clear_ADDR_FLAG(pI2CHandle) ;
	}

	// check for ADDR10 event

	temp1 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_ADD10) ;
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;

	if (temp1 && temp2) {
		// 10 bit address has been selected
		// 10 bit address has been sent to the slave by master it is same as ADDR event
		printf("10 bit addr mode not supported please switch to 7 bits addr mode \n") ;
		printf("restarting the communication with 7 bit addr mode \n") ;
		printf("demo mode \n");
		}

	// check for BTF event

	temp1 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_BTF) ;
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;

	if (temp1 && temp2) {
		// BTF event has occurred which is when data byte has been transfered

		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {

			if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG)) {

				// BTF  and TXE  both are set
				if (pI2CHandle->TxLen == 0) {

				// generate the stop condition
					// generate the stop condition only if the repeated start is enabled
					if (pI2CHandle->Sr == DISABLE) {
						I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx) ;
					}

				// reset all the contents of the member element
					I2C_CloseSendData(pI2CHandle) ;	// close send data function call for interrupt based implementation

				// notify the application about the i2c transmission complete
				I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_TX_CMPLT) ;
				}
			}
		}else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {
			// when new byte is recieved and dr hasn't been read yet
			printf("new byte recieved \n ") ;
		}
	}

	// check for STOPF event
	temp1 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_STOPF) ;
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;

	if (temp1 && temp2) {
		// stop condition has been detected by slave when master generates stop condition
			// this event only occurs in slave mode when stop condition is detected
		uint32_t DummyRead = 0 ;

		DummyRead = pI2CHandle->pI2Cx->SR1 ;
		DummyRead = 0 ;
		pI2CHandle->pI2Cx->CR1 |= DummyRead ;

		// notify the application about the i2c stop condition has been detected
		I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_STOP) ;

		}

	// check for RxNE event

	temp1 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_RxNE) ;
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;
	temp3 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN) ;

	if (temp1 && temp2 && temp3) {
	// RxNE event has occurred the rx buffer is not
	// empty and there is data in rx buffer and is to be read

			// Check for device Mode
	 if ( ( pI2CHandle->pI2Cx->SR2 & (  1 <<  I2C_SR2_MSL))) {
		 // The Device is in master mode
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX) {

			if (pI2CHandle->RxSize == 1) {
				printf("Receiving data 1byte interrupt mode \n") ;
				*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR ;
				pI2CHandle->RxLen -- ;
			}

		if (pI2CHandle->RxSize > 1) {
			printf("Receiving data interrupt mode \n") ;
			if (pI2CHandle->RxLen == 2 ) {
			// turn off the Acking
			pI2CHandle->pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK) ;
			}

			*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR ;
			 pI2CHandle->RxLen -- ;
			 pI2CHandle->pRxBuffer ++;
		}
		if (pI2CHandle->RxLen == 0 ) {
			// generate the stop condition and notify the application

			  // generate the stop condition
			if (pI2CHandle->Sr == I2C_DISABLE_SR) {
				I2C_Generate_Stop_Condition(pI2CHandle->pI2Cx) ;
			}

			// close the I2C rx
			I2C_Close_recieve(pI2CHandle) ;

			// notify the application
			I2C_ApplicationEventCallBack(pI2CHandle, I2C_EVENT_RX_CMPLT) ;

	  	    }
     	 }
   	  }
  }

	// check for TxE event

	temp1 = pI2CHandle->pI2Cx->SR1 & ( 1 << I2C_SR1_TxE) ;
	temp2 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITEVTEN) ;
	temp3 = pI2CHandle->pI2Cx->CR2 & ( 1 << I2C_CR2_ITBUFEN) ;

	if (temp1 && temp2 && temp3) {
		// TxE event has occurred the tx buffer is empty
			// transmitt the data
		// do this only when device is in master mode
		if ( (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL )) ) {
			 if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX) {
				 if (pI2CHandle->TxLen > 0 ) {
					printf("TransIT \n") ;
				// load the data into DR
				pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer) ;
				// decrement the TxLen
				pI2CHandle->TxLen -- ;
				// Increment the buffer address
				pI2CHandle->pTxBuffer++ ;
		}
	  }
	}
  }	/* TXE event function end */

}/*ISR Function end */

void I2C_ER_IRQHandling (I2C_Handle_t *pI2CHandle) {
	printf("error has occurred check the status flags \n") ;

}


// Application event complete callback api
void I2C_ApplicationEventCallBack(I2C_Handle_t *pI2CHandle , uint8_t eventcode) {
	if (eventcode == I2C_EVENT_RX_CMPLT) {
		printf("Reception has completed \n ") ;
	}else if (eventcode == I2C_EVENT_TX_CMPLT){
		printf("transmission has completed \n ") ;
	}else if (eventcode == I2C_EVENT_STOP){
		printf("stopped has taken place  \n ") ;
	}else {
		printf("something happened \n ") ;
	}

}
