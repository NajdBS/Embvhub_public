#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"


#define MAX_SPEED 999     // Maximum value for CCR (maximum speed)
#define MIN_SPEED 300     // Minimum speed 
#define START_SPEED 500   // Starting speed 
#define STEP_SPEED 100    // Step size to increase/decrease speed
#define TURNING_SPEED 700       // Speed for turning (left or right)

// DS1621 I2C Address and commands for the temperature sensor
#define DS1621_ADDR_WRITE  (0x90)  // DS1621 write address
#define DS1621_ADDR_READ   (0x91)  // DS1621 read address

// DS1621 Registers
#define DS1621_TEMP_REG    (0xAA)  // Register to read temperature
#define DS1621_CTRL_REG    (0xAC)  // Register to write control settings
#define DS1621_STATUS_REG  (0xA8)  // Status register

// Commands for controlling the DS1621
#define DS1621_START_CONVERSION  (0xEE)  // Start temperature conversion
#define DS1621_STOP_CONVERSION   (0x22)  // Stop temperature conversion


// Variables for system management
int current_speed_forward = START_SPEED;
int current_speed_backward = START_SPEED;

float temperature;

char TX_Buffer[50];
char RX_Buffer[50];
short ADC_Value[3];

int i=0;
int systemState = 0;
int channelIndex = 0;  
int currentChannel = 0;   


void config_clock()
{
    RCC->CR |= 1 << 16;    // HSE on
    while (!(RCC->CR & (1 << 17))); // Wait until HSE ready
    RCC->CFGR |= 0x00000001;  // Select HSE as system clock
    while ((RCC->CFGR & 0x00000004) != 0x00000004); // Wait until HSE is used as system clock
}

void config_GPIOA(void)
{
    RCC->AHB1ENR |= 0x1;   // Enable GPIOA clock
    GPIOA->MODER |= 0xFC;  // PA1, PA2, PA3 as Analog Input
	
}

void config_TIMER6(void)
{
    RCC->APB1ENR |= (1 << 4);     // Enable TIM6 clock
    //TIM6->PSC = 7999;             // Set prescaler to 7999
	  TIM6->PSC = 15999;             // Set prescaler to 15999
    TIM6->ARR = 999;              // Set auto-reload to 999 (1 second interval)
    TIM6->DIER = 0x1;             // Enable update interrupt
    NVIC_EnableIRQ(TIM6_DAC_IRQn); // Enable TIM6 interrupt in NVIC
}

void config_EXTI(void)
{
    RCC->APB2ENR |= 1 << 14;     // Enable SYSCFG clock
    SYSCFG->EXTICR[0] &= 0xFFF0; // Select PA0 as EXTI0 source
    EXTI->RTSR |= 0x1;           // Trigger interrupt on rising edge of PA0
    EXTI->IMR |= 0x1;            // Enable interrupt for EXTI line 0
    NVIC_EnableIRQ(EXTI0_IRQn);  // Enable EXTI0 interrupt in NVIC
}

void config_ADC1(void)
{
    // Step 1: Enable ADC1 clock
    RCC->APB2ENR |= (1 << 8);    // Enable ADC1 clock

    // Step 2: Configure ADC1
    ADC1->CR1 |= (1 << 8);       // SCAN mode enabled (multi-channel)
    ADC1->CR1 |= (1 << 11);      // Discontinuous mode enabled (DISCEN)
		
		ADC1->CR1 &= ~(0b111 << 13); // Clear [15:13] bits, set DISCNUM = 000 (1 channel)
    
		ADC1->CR1 |= (1 << 5);       // Enable EOC interrupt
    ADC1->CR2 |= (1 << 10);      // Enable EOC flag
   
		//Step 3: Configure sequence
		ADC1->SQR1 &= ~(0xF << 20); // Clear [23:20] bits, set sequence length = 1 (L=0)

    // Step 4: Configure sampling time
	  ADC1->SMPR2 &= ~(0b111 << 3); // Clear [5:3] bits, N1 = 3 ,channel sampling time : tconv = (3+12) * (1/4) ~= 4µs
		
		// Step 5: Enable ADC interrupt
    NVIC_EnableIRQ(ADC_IRQn);    // Enable ADC global interrupt

    // Step 6: Enable ADC
    ADC1->CR2 |= 1;              // Turn on ADC
}

void USART3_Init()
{
    // Enable GPIOB clock
    RCC->AHB1ENR |= (1 << 1);  // Enable GPIOB clock(bit 1 AHB1ENR)

    // Set PB10 (TX) and PB11 (RX) to alternate function (AF)
    GPIOB->MODER &= ~( (3 << (2 * 10)) | (3 << (2 * 11)) );   // Clear mode bits for PB10 and PB11
    GPIOB->MODER |= ( (2 << (2 * 10)) | (2 << (2 * 11)) );    // Set PB10 and PB11 to alternate function (AF)

    // Set AF7 (USART3) for PB10 and PB11
    GPIOB->AFR[1] &= ~( (0xF << (4 * (10 - 8))) | (0xF << (4 * (11 - 8))) );  // Clear AF bits
    GPIOB->AFR[1] |= ( (7 << (4 * (10 - 8))) | (7 << (4 * (11 - 8))) );       // Set AF7 (USART3) for PB10 and PB11
    //AF7 is the alternate function assigned to USART3_TX and USART3_RX on PB10 (transmit) and PB11 (receive), respectively.
		
	  //Config USART3		
		RCC->APB1ENR |= 1<<18; // Enable USART3 clock
		//USART3->BRR=834; 	// for 8 MHZ and 9600 Baudrate 
		USART3->BRR=1667; 	// for 16 MHZ and 9600 Baudrate 
		USART3->CR1 |= 1<<3; 	// Enable TE
	  USART3->CR1 |= 1<<2; 	// Enable RE
		USART3->CR1 |= 1<<5; 	// Enable RXNEIE
		USART3->CR1 |= 1<<4; 	// Enable IDLEIE
	  USART3->CR1 |= 1<<13; // Enable USART3
	  NVIC_EnableIRQ(USART3_IRQn);    // Enable USART3 global interrupt
		
}


// Function to send a character via USART3
void SendChar_usart3(char C)
{
	while((USART3->SR & 0x80) == 0){}// Check if the Transmit Data Register (TDR) is empty
  USART3->DR=C;  // Load the character into the Data Register (DR) for transmission
   
}


// Function to send a string via USART3
void SendTxt_usart3(char *ADR)
{
		while(*ADR)
		{
		SendChar_usart3(*ADR);
		ADR++;
		}
}




// Function to clear a buffer
void ClearBuffer(char *buffer, int size) 
{
    for (int j = 0; j < size; j++) {

			buffer[j] = '\0';
    }
}	


// Function to transmit a formatted string via USART3 (used for communication with HC-06 Bluetooth module)
void Transmit_string_to_HC06()
{
		sprintf(TX_Buffer,"CH1=%d CH2=%d CH3=%d Temp_DS1621=%.2f C\n",ADC_Value[0],ADC_Value[1],ADC_Value[2],temperature);
		SendTxt_usart3(TX_Buffer);
		ClearBuffer(TX_Buffer, sizeof(TX_Buffer));
}



void config_TIMER3(void)
{
    // Enable clock for Timer 3 (APB1)
    RCC->APB1ENR |= (1 << 1); //enable TIM3

    // Enable clock for GPIOA
    RCC->AHB1ENR |= 0x1; //enable GPIOA (for PA6 and PA7)
    
    // Enable clock for GPIOB
    RCC->AHB1ENR |= (1 << 1);//enable GPIOB (for PB0 and PB1)

    // Configure PA6, PA7, PB0, and PB1 in alternate function mode (AF)
    GPIOA->MODER &= ~( (3 << (2 * 6)) | (3 << (2 * 7)) ); // Clear mode bits for PA6 and PA7
    GPIOA->MODER |= ( (2 << (2 * 6)) | (2 << (2 * 7)) );  // Set PA6 and PA7 to alternate function (AF)

    GPIOB->MODER &= ~( (3 << (2 * 0)) | (3 << (2 * 1)) ); // Clear mode bits for PB0 and PB1
    GPIOB->MODER |= ( (2 << (2 * 0)) | (2 << (2 * 1)) );  // Set PB0 and PB1 to alternate function (AF)

    // Configure the alternate function for the pins (AF2 for TIM3)
    // PA6 and PA7 correspond to Timer 3 channels 1 and 2
    GPIOA->AFR[0] &= ~( (0xF << (4 * 6)) | (0xF << (4 * 7)) );  // Clear AF bits for PA6 and PA7
    GPIOA->AFR[0] |= ( (2 << (4 * 6)) | (2 << (4 * 7)) );       // Select AF2 function for PA6 and PA7

    // PB0 and PB1 correspond to Timer 3 channels 3 and 4
    GPIOB->AFR[0] &= ~( (0xF << (4 * 0)) | (0xF << (4 * 1)) );  // Clear AF bits for PB0 and PB1
    GPIOB->AFR[0] |= ( (2 << (4 * 0)) | (2 << (4 * 1)) );       // Select AF2 function for PB0 and PB1

    // Configure Timer 3
    TIM3->CCER |= 0x1111; //enables CC1E, CC2E, CC3E, CC4E (channels 1, 2, 3, 4)
    TIM3->CCMR1 |= 0x6060; //channels 1 and 2 in PWM1 mode
    TIM3->CCMR2 |= 0x6060; //channels 3 and 4 in PWM1 mode

    // Configure the prescaler (PSC) and auto-reload value (ARR)
    TIM3->PSC = 15;         //timer frequency of 1 MHz 
    TIM3->ARR = 999;        // Auto-reload value (ARR) for a 1 MHz timer, giving a period of 1 ms
    
		// Start Timer 3
    TIM3->CR1 |= 0x1;
}

void I2C1_Init()
{
    // Enable clock for I2C1 (APB1)
    RCC->APB1ENR |= (1 << 21); 

    // Enable clock for GPIOB
    RCC->AHB1ENR |= (1 << 1);//enable GPIOA (for PB6 and PB7)

    // Configure PA6, PA7, PB0, and PB1 in alternate function mode (AF)
    GPIOB->MODER &= ~( (3 << (2 * 6)) | (3 << (2 * 7)) ); // Clear mode bits for PA6 and PA7
    GPIOB->MODER |= ( (2 << (2 * 6)) | (2 << (2 * 7)) );  // Set PB6 and PB7 to alternate function (AF)

    // Configure the alternate function for the pins (AF4 for I2C1)
    // PB6 and PB7 correspond to I2C1 SDA and SCL
    GPIOB->AFR[0] &= ~( (0xF << (4 * 6)) | (0xF << (4 * 7)) );  // Clear AF bits for PB6 and PB7
    GPIOB->AFR[0] |= ( (4 << (4 * 6)) | (4 << (4 * 7)) );       // Select AF4 function for PB6 and PB7
	  
    GPIOB->OTYPER |= ( (1 << 7) | (1 << 6) );       // Set PB6 and PB7 as Output open-drain
		// Reset the I2C 
		I2C1->CR1 |= (1<<15);
		I2C1->CR1 &= ~(1<<15);
		
		// Input clock in I2C_CR2 Register 
		I2C1->CR2 |= (16<<0);  // PCLK1 FREQUENCY in MHz
		
		// Configure the clock control registers
		I2C1->CCR = 80<<0;  
		
		// Configure the rise time register
		I2C1->TRISE = 17;  
		
		// Enable the peripheral
		I2C1->CR1 |= (1<<0);  // Enable I2C
}

void I2C_Start (void)
{
	I2C1->CR1 |= (1<<8);  // Generate START
	while (!(I2C1->SR1 & (1<<0)));  // Wait fror SB bit to set
}

void I2C_Address (uint8_t Address)
{
	I2C1->DR = Address;  //  send the address
	while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
	I2C1->SR2; //Clearing the ADDR bit
}

void I2C_Write (uint8_t data)
{
	while (!(I2C1->SR1 & (1<<7)));  // wait for TXE bit to set
	I2C1->DR = data;
	
}

void I2C_Read (uint8_t Address, uint8_t *buffer, uint8_t size)
{
	
	  int remaining = size;
/*	
	if (size == 1)
	{
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
		
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit 
		I2C1->SR2;  //Clearing the ADDR bit
		I2C1->CR1 |= (1<<9);  // Stop I2C

		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		
		buffer[size-remaining] = I2C1->DR;  // Read the data from the DATA REGISTER
		
	}
	
	else 
	{*/
	
		
		
		
		
		I2C1->DR = Address;  //  send the address
		while (!(I2C1->SR1 & (1<<1)));  // wait for ADDR bit to set
		I2C1->SR2;  //Clearing the ADDR bit
		
		I2C1->CR1 |= (1<<10);  // Enable the ACK

/*
		while (remaining>2)
		{
			
			while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
			
	
			buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer			
			

			I2C1->CR1 |= 1<<10;  // Set the ACK bit to Acknowledge the data received
			
			remaining--;
		}*/
		
		
		
		
		
		
		
		// Read the SECOND LAST BYTE
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;
		I2C1->CR1 |= (1<<9);  // Stop I2C
		
		remaining--;
		
		// Read the Last BYTE
		I2C1->CR1 &= ~(1<<10);  // clear the ACK bit 
		while (!(I2C1->SR1 & (1<<6)));  // wait for RxNE to set
		buffer[size-remaining] = I2C1->DR;  // copy the data into the buffer
	
	//}	
	
}

void I2C_Stop (void)
{
	I2C1->CR1 |= (1<<9);  // Stop I2C
}






/*void DS1621_Write (uint8_t Address, uint8_t Reg, uint8_t Data)
{
		I2C_Start ();
		I2C_Address (Address);
		I2C_Write (Reg);
		I2C_Write (Data);
		I2C_Stop ();
}*/
void DS1621_Command(uint8_t Address, uint8_t Command)
{
    I2C_Start();           
    I2C_Address(Address);  
    I2C_Write(Command);    
    I2C_Stop();            
}


void DS1621_Read (uint8_t Address, uint8_t Reg, uint8_t *buffer, uint8_t size)
{
		I2C_Start ();
		I2C_Address (Address);
		I2C_Write (Reg);
		I2C_Start ();  // repeated start
		I2C_Read (Address+0x01, buffer, size);
		I2C_Stop ();
}

void DS1621_Init (void)
{
		DS1621_Command(DS1621_ADDR_WRITE, DS1621_START_CONVERSION);
}



// Function to read temperature from the DS1621 sensor and return the value in Celsius)(9 bits resolution)
float DS1621_Read_Temp(void)
{
    uint8_t Rx_data[2];  // Array to store the raw data from the sensor
    int16_t val;         // Variable to hold the processed temperature value (signed 16-bit integer)
    float temp;          // Variable to store the final temperature in Celsius

    // Read 2 bytes of data from the DS1621 temperature register
    DS1621_Read(DS1621_ADDR_WRITE, DS1621_TEMP_REG, Rx_data, 2);

    // Combine the two received bytes to form a 9-bit value
    val = (int16_t)((Rx_data[0] << 1) | (Rx_data[1] >> 7));

    // If the most significant bit of the value is set, it indicates a negative temperature
    if (val & 0x0100) {
        val |= 0xFF00;  // Extend the sign for negative values
        temp = (float)val * 0.5;  // Convert the raw value to temperature (negative value)
    } else {
        temp = (float)val * 0.5;  // Convert the raw value to temperature (positive value)
    }

    return temp;  // Return the temperature in Celsius
}



int main(void)
{
		//config_clock();//8MHZ
    config_GPIOA();
	  USART3_Init();
    config_TIMER6();
    config_EXTI();
    config_ADC1();
	  config_TIMER3();
    I2C1_Init();
		DS1621_Init();
		// Interruptions
    NVIC_SetPriority(USART3_IRQn, 0);    
		NVIC_SetPriority(EXTI0_IRQn, 1);     
    NVIC_SetPriority(TIM6_DAC_IRQn, 2);  
    NVIC_SetPriority(ADC_IRQn, 2);       
		while(1){};
}

void TIM6_DAC_IRQHandler(void)
{
    if((TIM6->SR & 0x1) != 0) // If update flag is set (UIF=1)
    {
				channelIndex++;
        if (channelIndex == 1)
				{
           ADC1->SQR3 = (1 << 0) ; // select (channel 1)
				}
        if (channelIndex == 2)
				{
            ADC1->SQR3 = (2 << 0) ; // select (channel 2)
				}
				if (channelIndex == 3)
        {   
					  ADC1->SQR3 = (3 << 0) ; // select (channel 3)
					  channelIndex=0;
				}			
				ADC1->CR2 |= (1 << 30);  // Start ADC conversion
			  TIM6->SR &= 0x0000; // Interrupt has been handled (UIF=0)
    }
}

void EXTI0_IRQHandler(void)
{
    if((EXTI->PR & 0x1) != 0)    // Check if EXTI0 interrupt is pending
    {
        systemState++;
				if (systemState == 1)
				{
				SendTxt_usart3("SYSTEM BEGIN\n");
				TIM6->CR1 = 0x1;         // Start the TIM6 timer		
				}	
				if (systemState == 2)
				{
				SendTxt_usart3("SYSTEM STOP\n");
				TIM6->CR1 = 0;         // Stop the TIM6 timer		
				TIM6->CNT = 0;         // Reset the TIM6 counter to 0
				systemState=0;
				channelIndex = 0;   
				currentChannel = 0;   
				}	
				
        EXTI->PR |= 0x1;         // Clear the EXTI0 interrupt flag
    }
}

void ADC_IRQHandler(void)
{
    if (ADC1->SR & 0x2)          // Check End of Conversion (EOC) flag
    {
			  currentChannel++;  	
				if (currentChannel == 1)
				{
            ADC_Value[0] = ADC1->DR; // Store result for PA1 (channel 1)	  
				}
				
        if (currentChannel == 2)
				{
            ADC_Value[1] = ADC1->DR; // Store result for PA2 (channel 2)				
				}
				
				if (currentChannel == 3)
        {   
					  ADC_Value[2] = ADC1->DR; // Store result for PA3 (channel 3)
						temperature=DS1621_Read_Temp();
					  Transmit_string_to_HC06();
						currentChannel=0;		
				}			 	
        //ADC1->SR &= ~0x2;         // Clear EOC flag				
    }

}

void USART3_IRQHandler()
{
	if((USART3->SR & 0x20) != 0) 
	{
		i++;
		RX_Buffer[i-1]=USART3->DR;
	}
	if((USART3->SR & 0x10) != 0) 
	{
			SendTxt_usart3(RX_Buffer);
			// Command AVANCE+ to increase speed forward
			if (strstr(RX_Buffer, "AVANCE+"))
			{
					if (current_speed_forward + STEP_SPEED <= MAX_SPEED) {
							current_speed_forward += STEP_SPEED;  // Increase forward speed up to MAX_SPEED
					} else {
							current_speed_forward = MAX_SPEED;  // Don't exceed MAX_SPEED
					}
					TIM3->CCR1 = current_speed_forward;  
					TIM3->CCR2 = 0;  
					TIM3->CCR3 = 0;                     
					TIM3->CCR4 = current_speed_forward;    					
					sprintf(TX_Buffer,"Current_speed_forward=%d\n",current_speed_forward);
					SendTxt_usart3(TX_Buffer);
			}

			// Command ARRIERE+ to increase speed backward
			if (strstr(RX_Buffer, "ARRIERE+"))
			{
					if (current_speed_backward + STEP_SPEED <= MAX_SPEED) {
							current_speed_backward += STEP_SPEED;  // Increase backward speed up to MAX_SPEED
					} else {
							current_speed_backward = MAX_SPEED;  // Don't exceed MAX_SPEED
					}
					TIM3->CCR1 = 0;                       
					TIM3->CCR2 = current_speed_backward;                      
					TIM3->CCR3 = current_speed_backward;  
					TIM3->CCR4 = 0;  
					sprintf(TX_Buffer,"Current_speed_backward=%d\n",current_speed_backward);
					SendTxt_usart3(TX_Buffer);
					
			}

			// Command AVANCE- to decrease speed forward
			if (strstr(RX_Buffer, "AVANCE-"))
			{
					if (current_speed_forward - STEP_SPEED >= MIN_SPEED) {
							current_speed_forward -= STEP_SPEED;  // Decrease forward speed down to MIN_SPEED
					} else {
							current_speed_forward = MIN_SPEED;  // Don't go below MIN_SPEED
					}
					TIM3->CCR1 = current_speed_forward;  
					TIM3->CCR2 = 0;  
					TIM3->CCR3 = 0;                      
					TIM3->CCR4 = current_speed_forward;                      
					sprintf(TX_Buffer,"Current_speed_forward=%d\n",current_speed_forward);
					SendTxt_usart3(TX_Buffer);
			}

			// Command ARRIERE- to decrease speed backward
			if (strstr(RX_Buffer, "ARRIERE-"))
			{
					if (current_speed_backward - STEP_SPEED >= MIN_SPEED) {
							current_speed_backward -= STEP_SPEED;  // Decrease backward speed down to MIN_SPEED
					} else {
							current_speed_backward = MIN_SPEED;  // Don't go below MIN_SPEED
					}
					TIM3->CCR1 = 0;                       
					TIM3->CCR2 = current_speed_backward;  
					TIM3->CCR3 = current_speed_backward;  
					TIM3->CCR4 = 0;  
					sprintf(TX_Buffer,"Current_speed_backward=%d\n",current_speed_backward);
					SendTxt_usart3(TX_Buffer);
			}

			// Command GAUCHE to turn left
			if (strstr(RX_Buffer, "GAUCHE"))
			{
					TIM3->CCR1 = 0;  
					TIM3->CCR2 = 0;                    
					TIM3->CCR3 = 0;  
					TIM3->CCR4 = TURNING_SPEED;                      
					sprintf(TX_Buffer,"Turning_speed=%d\n",TURNING_SPEED);
					SendTxt_usart3(TX_Buffer);
			}

			// Command DROITE to turn right
			if (strstr(RX_Buffer, "DROITE"))
			{
					TIM3->CCR1 = TURNING_SPEED;                       
					TIM3->CCR2 = 0;   
					TIM3->CCR3 = 0;                      
					TIM3->CCR4 = 0;  
					sprintf(TX_Buffer,"Turning_speed=%d\n",TURNING_SPEED);
					SendTxt_usart3(TX_Buffer);
			}

			// Command STOP to stop all motors
			if (strstr(RX_Buffer, "STOP"))
			{
					TIM3->CCR1 = 0;  
					TIM3->CCR2 = 0;   
					TIM3->CCR3 = 0;   
					TIM3->CCR4 = 0;   
					current_speed_forward = MIN_SPEED;  // Reset forward speed
					current_speed_backward = MIN_SPEED; // Reset backward speed
					
					TIM6->CR1 = 0;         // Stop the TIM6 timer		
					TIM6->CNT = 0;         // Reset the TIM6 counter to 0
					systemState=0;
					channelIndex = 0;   
					currentChannel = 0;   
			}
		  //SendTxt_usart3(RX_Buffer);
		  ClearBuffer(RX_Buffer, sizeof(RX_Buffer));
			i=0;
			USART3->SR;
			USART3->DR;		
	}

}
