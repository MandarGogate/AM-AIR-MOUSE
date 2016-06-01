/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed "as is", without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. This file may only be built (assembled or compiled and linked)
**  using the Atollic TrueSTUDIO(R) product. The use of this file together
**  with other tools than Atollic TrueSTUDIO(R) is not permitted.
**
*****************************************************************************
*/

/* Includes */
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx_i2c.h"
/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */
#include "defines.h"
#include "tm_stm32f4_mpu6050.h"
/**
**===========================================================================
**
**  Abstract: main program
**
**===========================================================================
*/
#define BUFFER_SIZE 16

//notice the use of the volatile keyword this is important as without it the compiler may make
//optimisations assuming the value of this variable hasnt changed
//volatile
char received_buffer[BUFFER_SIZE+1];

static uint8_t DataReceivedCounter = 0; //tracks the number of characters received so far, reset after each command

//function prototypes
void USARTCommandReceived(char * command);
void ClearCommand();
void Delay(int nCount);
void ConfigureUsart(int baudrate);
void USART1_IRQHandler(void);
//Configures the USART using pin B6 as TX and B7 as RX and the passed in baudrate
void ConfigureUsart(int baudrate){

	//structures used configure the hardware
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	//enable the clocks for the GPIOB and the USART
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	//Initialise pins GPIOB 6 and GPIOB 7
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF; //we are setting the pin to be alternative function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//Connect the TX and RX pins to their alternate function pins
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_USART1); //
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	//configure USART
	USART_InitStruct.USART_BaudRate = baudrate;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx; //enable send and receive (Tx and Rx)
	USART_Init(USART1, &USART_InitStruct);

	//Enable the interupt
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// finally this enables the complete USART1 peripheral
	USART_Cmd(USART1, ENABLE);
}

//writes out a string to the passed in usart. The string is passed as a pointer
void SendData(USART_TypeDef* USARTx, volatile char *s){

	while(*s){
		// wait until data register is empty
		while( !(USARTx->SR & 0x00000040) );
		USART_SendData(USARTx, *s);
		*s++;
	}
}


int main(void)
{

	/* Init pins PD12, 13, 14, 15 */
	TM_GPIO_Init(GPIOE, GPIO_Pin_8 | GPIO_Pin_9, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_High);
	int p1,p2;
	long long int i =0;
	/*
	 * PUSH BUTTON TEST CASE
	while(1){


	}
	*/
  /* TODO - Add your application code here */
	TM_MPU6050_t MPU6050_Data0;
	uint8_t sensor1 = 0;
	char str[120];
	ConfigureUsart(9600); // setup usart 1 with a baudrate of 9600
	SendData(USART1,"AT+BAUD8");
	Delay(900000);
	ConfigureUsart(115200); // setup usart 1 with a baudrate of 9600

	Delay(900000);
	/* Initialize MPU6050 sensor 0, address = 0xD0, AD0 pin on sensor is low */
	if (TM_MPU6050_Init(&MPU6050_Data0, TM_MPU6050_Device_0, TM_MPU6050_Accelerometer_2G, TM_MPU6050_Gyroscope_2000s) == TM_MPU6050_Result_Ok) {
		sensor1 = 1;
	}




  /* Infinite loop */
  while (1)
  {
	  if (sensor1) {
	  				/* Read all data from sensor 1 */
	  				TM_MPU6050_ReadAll(&MPU6050_Data0);
	  				if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_8)==1){
	  							p1 = 1;
	  						}else{
	  						p1 = 0;
	  						}


	  						if (GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_9)==1){
	  							p2 = 1;
	  						}else{
	  						p2 = 0;
	  						}

	  				/* Format data */

	  				// Sending MPU data over Bluetooth
	  				sprintf(str, "%d,%d,%d,%d,%d\n",
	  					  					//MPU6050_Data0.Accelerometer_X,
	  					  					//MPU6050_Data0.Accelerometer_Y,
	  					  					//MPU6050_Data0.Accelerometer_Z,
	  					  					p1,
	  					  					p2,
	  					  					MPU6050_Data0.Gyroscope_X,
	  					  					MPU6050_Data0.Gyroscope_Y,

	  					  					MPU6050_Data0.Gyroscope_Z

	  					  				);
	  				SendData(USART1, str);
	  				for(i=0;i<18300;i++);

	  	  }

}

}
/*
 * Callback used by stm32f4_discovery_audio_codec.c.
 * Refer to stm32f4_discovery_audio_codec.h for more info.
 */
//This method is executed when data is received on the RX line (this is the interrupt), this method can process the
//data thats been received and decide what to do. It is executed for each character received, it reads each character
//and checks to see if it received the enter key (ascii code 13) or if the total number of characters received is greater
//that the buffer size.
//Note that there is no reference to this method in our setup code, this is because the name of this method is defined in the
//startup_stm32f4xx.S (you can find this in the startup_src folder). When listening for interrupts from USART 2 or 3 you would
//define methods named USART2_IRQHandler or USART3_IRQHandler
void USART1_IRQHandler(void){
	//check the type of interrupt to make sure we have received some data.
	if( USART_GetITStatus(USART1, USART_IT_RXNE) ){
		char t = USART1->DR; //Read the character that we have received

		if( (DataReceivedCounter < BUFFER_SIZE) && t != 13 ){
			received_buffer[DataReceivedCounter] = t;
			DataReceivedCounter++;
		}
		else{ // otherwise reset the character counter and print the received string
			DataReceivedCounter = 0;
			//only raise a command event if the enter key was pressed otherwise just clear
			if(t == 13){
				USARTCommandReceived(received_buffer);
			}

			ClearCommand();

		}
	}
}

//this method is called when a command is received from the USART, a command is only received when enter
//is pressed, if the buffer length is exceeded the buffer is cleared without raising a command
void USARTCommandReceived(char * command){
	SendData(USART1, received_buffer);

	}

void ClearCommand(){
	int i =0;
	for(i=0;i < BUFFER_SIZE; i++){
		received_buffer[i] = 0;
	}

}

void Delay(int nCount) {
  while(nCount--) {
  }
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

/*
 * Callback used by stm324xg_eval_audio_codec.c.
 * Refer to stm324xg_eval_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
