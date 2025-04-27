/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h> // Pour utiliser atoi()
#include "modbus_crc.h"
#include "modbusSlave.h"
#include "modbusmaster.h"
#include "EEPROM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//#define MAX_REGISTERS 32
//#define MAX_COILS      8

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

extern uint8_t Coils_Database[25];
extern uint8_t Inputs_Database[25];
extern uint16_t Input_Registers_Database[50];
extern uint16_t Holding_Registers_Database[50];

// Buffers pour Master transmission
char buffer[32];
uint8_t choice;
uint8_t slaveAddr;
uint16_t startAddr, numRegs, value;
uint16_t values[10];
uint8_t coils[10];
uint8_t TxData_M[8];

//uint8_t coils[MAX_COILS] = {0};  // Example for storing coil states
//uint8_t RxData[MAX_REGISTERS];

// Buffers for Slave reception and transmission
uint8_t RxData[256];
uint8_t TxData[256];
uint16_t Data[10];

// Buffers pour la réception et l'envoi
uint8_t rx_buffer[1];       // Pour lire un caractère
uint8_t tx_buffer[500];     // Pour envoyer des messages
char input_buffer[20];      // Pour les entrées utilisateur
uint8_t input_index = 0;    // Index pour gérer les entrées

// Mode : 0 = Slave, 1 = Master
uint8_t mode = 0; // 0 = Slave, 1 = Master

#define DEV_ADDR 0xA0 // Adresse I2C de l'EEPROM

// Déclarer les paramètres UART pour stocker les données
UART_Params uartParams = {115200,UART_PARITY_NONE,UART_WORDLENGTH_8B,UART_STOPBITS_1};  // Valeurs par défaut
// Déclarations pour lire et écrire dans l'EEPROM
uint8_t uartParamsData[sizeof(UART_Params)];

// Prototypes
void ShowMenu(void);
void ProcessOption(char option);
void ConfigureUART2(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (RxData[0] == SLAVE_ID)
	{
		switch (RxData[1]){
		case 0x03:
			readHoldingRegs();
			break;
		case 0x04:
			readInputRegs();
			break;
		case 0x01:
			readCoils();
			break;
		case 0x02:
			readInputs();
			break;
		case 0x06:
			writeSingleReg();
			break;
		case 0x10:
			writeHoldingRegs();
			break;
		case 0x05:
			writeSingleCoil();
			break;
		case 0x0F:
			writeMultiCoils();
			break;
		default:
			modbusException(ILLEGAL_FUNCTION);
			break;
		}
	}

	HAL_UARTEx_ReceiveToIdle_IT(&huart2, RxData, 256);
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        // Mise à jour des broches du Port D (sortie) depuis Coils_Database[0] et Coils_Database[1]
		uint8_t lower_byte_out = Coils_Database[0]; // PD0 à PD7
        uint8_t upper_byte_out = Coils_Database[1]; // PD8 à PD15

        for (int i = 0; i < 8; i++) {
            HAL_GPIO_WritePin(GPIOD, (1 << i), (lower_byte_out & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        for (int i = 0; i < 8; i++) {
            HAL_GPIO_WritePin(GPIOD, (1 << (i + 8)), (upper_byte_out & (1 << i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        // Lecture des broches du Port C (entrée) et mise à jour de Coils_Database[2] et Coils_Database[3]
        uint8_t lower_byte_in = 0; // PC0 à PC7
        uint8_t upper_byte_in = 0; // PC8 à PC15

        for (int i = 0; i < 8; i++) {
            if (HAL_GPIO_ReadPin(GPIOC, (1 << i)) == GPIO_PIN_SET) {
                lower_byte_in |= (1 << i); // Ajouter 1 au bit correspondant
            }
        }

        for (int i = 0; i < 8; i++) {
            if (HAL_GPIO_ReadPin(GPIOC, (1 << (i + 8))) == GPIO_PIN_SET) {
                upper_byte_in |= (1 << i); // Ajouter 1 au bit correspondant
            }
        }

        // Mise à jour de la base de données Coils
        Coils_Database[2] = lower_byte_in; // PC0 à PC7
        Coils_Database[3] = upper_byte_in; // PC8 à PC15
        Inputs_Database[0] = lower_byte_in; // PC0 à PC7 (même valeur que Coils_Database[2])
        Inputs_Database[1] = upper_byte_in; // PC8 à PC15 (même valeur que Coils_Database[3])

        Update_Input_Registers();
//Scenario  de test
       // Scenario 1: Lire l'état de PC0 et contrôler la LED sur PD0
        //une variable globale pour suivre si la LED a été allumée
        static uint8_t led_on = 0;

        if (Coils_Database[0] & 0x01) {
            // Si la LED est allumée
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_RESET) {
                // Si le bouton est pressé, éteindre la LED
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET); // Éteindre la LED
                Coils_Database[0] &= ~0x01; // Mettre à jour Coils_Database[0] pour éteindre la LED (mettre le premier bit à 0)
                led_on = 1;  // Indiquer que la LED a été allumée au moins une fois
            }
        } else if (led_on) { // Cette condition ne sera vraie que si la LED a été allumée une fois
            // Si la LED était éteinte dans Coils_Database[0] et si elle a été allumée au moins une fois
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0) == GPIO_PIN_SET) {
                // Rallumer la LED
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);
                Coils_Database[0] |= 0x01; // Mettre à jour Coils_Database[0] pour allumer la LED (mettre le premier bit à 1)
                led_on = 0; // Réinitialiser le flag après avoir rallumé la LED
            }
        }


        // Scenario 2: Ajuster la luminosité de la LED sur PD1 en fonction du potentiomètre
        // Utiliser la valeur du potentiomètre pour contrôler la luminosité de la LED via PWM PA5
           uint16_t pot_value = Holding_Registers_Database[0]; // Valeur du potentiomètre stockée dans le Holding registre
           uint32_t pwm_duty_cycle = (pot_value * 100) / 4095; // Convertir la valeur du potentiomètre en pourcentage (12 bits = 0 à 4095)

        // Appliquer le duty cycle PWM à la LED sur PA5 (par exemple via TIM1 PWM)
           __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pwm_duty_cycle); // Appliquer le PWM à la LED sur PA5

          ////////////////////////////////////////////////////////////////////////////////

           uint16_t pot_value2 = Holding_Registers_Database[2]; // Valeur du potentiomètre stockée dans le Holding registre
           uint32_t pwm_duty_cycle2 = (pot_value2 * 100) / 4095; // Convertir la valeur du potentiomètre en pourcentage (12 bits = 0 à 4095)

        // Appliquer le duty cycle PWM à la LED sur PA6 (par exemple via TIM3 ch1 PWM)
           __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm_duty_cycle2); // Appliquer le PWM à la LED sur PA6

}

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM6_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim6); // Démarrer le timer en mode interruption
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
  //HAL_UARTEx_ReceiveToIdle_IT(&huart2, RxData, 256);

  // Lire les paramètres UART au redémarrage
  EEPROM_Read_UART_Params(5, 0, &uartParams);
  ConfigureUART2();
  // Afficher le menu initial
  ShowMenu();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  if (mode == 0)
	  	  {
	 	  HAL_UARTEx_ReceiveToIdle_IT(&huart2, RxData, 256);
	 	  }
	  if (HAL_UART_Receive(&huart3, rx_buffer, 1, HAL_MAX_DELAY) == HAL_OK)
	  	  {
	 	  char option = rx_buffer[0];
	 	  ProcessOption(option); // Traiter l'option choisie
	 	  }
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// Afficher le menu
void ShowMenu(void) {
    sprintf((char *)tx_buffer,
    	"\n"
    	//"===========================\r\n"
        "\r\n=== Configuration Modbus ===\r\n"
    	"\n"
    	"0: Rafraichir le menu\r\n"
        "1: Choisir Mode (Slave ou Master)\r\n"
        "2: Choisir Baudrate\r\n"
        "3: Choisir Longueur des mots\r\n"
        "4: Choisir Parite\r\n"
        "5: Choisir Bits d'arret\r\n"
    	"6: Menu Modbus Master\r\n"
    	"\n"
    	"===========================\r\n"
    	"\n"
        "Entrez votre choix:");
    HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
}

// Traiter une option utilisateur
void ProcessOption(char option) {
    switch (option) {
        case '0': // Rafraichir le menu
               ShowMenu();
               break;
        case '1': // Configuration du Mode (Slave ou Master)
            sprintf((char *)tx_buffer,
            	"\n"
                "\r\nOptions Mode:\r\n"
                "0: Mode Slave\r\n"
                "1: Mode Master\r\n"
            	"Votre choix actuel : %s\r\n"
            	"Entrez votre choix: ", (mode == 0) ? "Slave" : "Master");
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, rx_buffer, 1, HAL_MAX_DELAY);

            if (rx_buffer[0] == '0') {
                mode = 0; // Slave
            } else if (rx_buffer[0] == '1') {
                mode = 1; // Master
            } else {
                sprintf((char *)tx_buffer, "Option invalide.\r\n");
                HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
                return;
            }
            sprintf((char *)tx_buffer, "Mode configure a %s.\r\n", (mode == 0) ? "Slave" : "Master");
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            ShowMenu(); // Rafraîchir le menu après la configuration
            break;

        case '2': // Configuration Baudrate
            sprintf((char *)tx_buffer,
            	"\n"
            	"\r\nOptions Baudrate:\r\n"
                "1: 9600\r\n"
                "2: 19200\r\n"
                "3: 38400\r\n"
                "4: 57600\r\n"
                "5: 115200\r\n"
                "6: Saisir manuellement\r\n"
            	"Votre choix actuel : %lu\r\n"
            	"Entrez votre choix: ", uartParams.baudrate);
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, rx_buffer, 1, HAL_MAX_DELAY);

            if (rx_buffer[0] >= '1' && rx_buffer[0] <= '5') {
                int baud_choices[] = {9600, 19200, 38400, 57600, 115200};
                uartParams.baudrate = baud_choices[rx_buffer[0] - '1'];
            } else if (rx_buffer[0] == '6') {
                sprintf((char *)tx_buffer, "Entrez le baudrate (par exemple 9600): ");
                HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);

                // Lire l'entrée complète de l'utilisateur jusqu'à "Entrée"
                memset(input_buffer, 0, sizeof(input_buffer));
                HAL_UART_Receive(&huart3, (uint8_t *)input_buffer, sizeof(input_buffer)-1, HAL_MAX_DELAY); // Lire jusqu'à ce que l'utilisateur appuie sur "Entrée"

                uartParams.baudrate = atoi(input_buffer);  // Convertir en entier
                if (uartParams.baudrate <= 0) {
                    sprintf((char *)tx_buffer, "Valeur de baudrate invalide.\r\n");
                    HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
                    return;
                }
            } else {
                sprintf((char *)tx_buffer, "Option invalide.\r\n");
                HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
                return;
            }
            // Sauvegarder ces paramètres dans l'EEPROM
            EEPROM_Write_UART_Params(5, 0, &uartParams);
            sprintf((char *)tx_buffer, "Baudrate configure a %lu.\r\n", uartParams.baudrate);
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            ShowMenu(); // Rafraîchir le menu après la configuration
            break;

        case '3': // Configuration Longueur des mots
            sprintf((char *)tx_buffer,
            	"\n"
            	"\r\nOptions Longueur des mots:\r\n"
                "1: 8 bits\r\n"
                "2: 9 bits\r\n"
            	"Votre choix actuel : %s\r\n"
            	"Entrez votre choix: ", (uartParams.wordLength == UART_WORDLENGTH_8B) ? "8 bits" : "9 bits");
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, rx_buffer, 1, HAL_MAX_DELAY);

            uartParams.wordLength = (rx_buffer[0] == '1') ? UART_WORDLENGTH_8B : UART_WORDLENGTH_9B;
            // Sauvegarder ces paramètres dans l'EEPROM
            EEPROM_Write_UART_Params(5, 0, &uartParams);
            sprintf((char *)tx_buffer, "Longueur des mots configuree.\r\n");
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            ShowMenu(); // Rafraîchir le menu après la configuration
            break;

        case '4': // Configuration Parite
            sprintf((char *)tx_buffer,
            	"\n"
            	"\r\nOptions Parite:\r\n"
                "1: Aucune\r\n"
                "2: Paire\r\n"
                "3: Impaire\r\n"
            	"Votre choix actuel : %s\r\n"
                "Entrez votre choix: ", (uartParams.parity == UART_PARITY_NONE) ? "Aucune" : (uartParams.parity == UART_PARITY_EVEN) ? "Paire" : "Impaire");
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, rx_buffer, 1, HAL_MAX_DELAY);

            if (rx_buffer[0] == '1') uartParams.parity = UART_PARITY_NONE;
            else if (rx_buffer[0] == '2') uartParams.parity = UART_PARITY_EVEN;
            else if (rx_buffer[0] == '3') uartParams.parity = UART_PARITY_ODD;
            // Sauvegarder ces paramètres dans l'EEPROM
            EEPROM_Write_UART_Params(5, 0, &uartParams);
            sprintf((char *)tx_buffer, "Parite configuree.\r\n");
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            ShowMenu(); // Rafraîchir le menu après la configuration
            break;

        case '5': // Configuration Bits d'arrêt
            sprintf((char *)tx_buffer,
            	"\n"
            	"\r\nOptions Bits d'arret:\r\n"
                "1: 1 bit\r\n"
                "2: 2 bits\r\n"
            	"Votre choix actuel : %s\r\n"
                "Entrez votre choix: ", (uartParams.stopBits == UART_STOPBITS_1) ? "1 bit" : "2 bits");
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, rx_buffer, 1, HAL_MAX_DELAY);

            uartParams.stopBits = (rx_buffer[0] == '1') ? UART_STOPBITS_1 : UART_STOPBITS_2;
            // Sauvegarder ces paramètres dans l'EEPROM
            EEPROM_Write_UART_Params(5, 0, &uartParams);
            sprintf((char *)tx_buffer, "Bits d'arret configurees.\r\n");
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            ShowMenu(); // Rafraîchir le menu après la configuration
            break;
        case '6': // Interaction avec le menu Modbus Master
                   if (mode == 1) { // Vérifier que le mode est "Master"
                       ModbusMenuInteraction();
                   } else {
                       sprintf((char *)tx_buffer, "Mode non valide. Configurez d'abord en mode Master.\r\n");
                       HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
                   }
                   ShowMenu();// Afficher à nouveau le menu après l'interaction
                   break;
        default:
            sprintf((char *)tx_buffer, "Option invalide.\r\n");
            HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
            ShowMenu(); // Rafraîchir le menu après la configuration
            break;
    }

    // Appliquer la configuration
    ConfigureUART2();
}

void ConfigureUART2(void) {

	// 1. Désactiver l'UART2 pour éviter toute interférence
    HAL_UART_DeInit(&huart2); // Désactive l'UART

    // 2. Appliquer la nouvelle configuration
    huart2.Init.BaudRate = uartParams.baudrate;
    huart2.Init.WordLength = uartParams.wordLength;
    huart2.Init.StopBits = uartParams.stopBits;
    huart2.Init.Parity = uartParams.parity;

    // 3. Réinitialiser l'UART pour appliquer les changements de configuration
    HAL_UART_Init(&huart2); // Appliquer la configuration

    // 4. Optionnel : Assurez-vous que l'UART est prêt à communiquer (si nécessaire)
    // Vérifiez si l'UART est correctement initialisé et prêt pour la communication
    if (HAL_UART_GetState(&huart2) != HAL_UART_STATE_READY) {
    // Si l'UART n'est pas prêt, vous pouvez gérer l'erreur ici
    // Par exemple, afficher un message d'erreur ou tenter une réinitialisation
        sprintf((char *)tx_buffer, "Erreur d'initialisation de l'UART.\r\n");
        HAL_UART_Transmit(&huart3, tx_buffer, strlen((char *)tx_buffer), HAL_MAX_DELAY);
    }
}
void ModbusMenuInteraction()
{
    char menu[] =
        "\r\n=== Modbus Master Menu ===\r\n"
        "1. Read Holding Registers (0x03)\r\n"
        "2. Read Input Registers (0x04)\r\n"
        "3. Read Coils (0x01)\r\n"
        "4. Read Discrete Inputs (0x02)\r\n"
        "5. Write Single Register (0x06)\r\n"
        "6. Write Multiple Registers (0x10)\r\n"
        "7. Write Single Coil (0x05)\r\n"
        "8. Write Multiple Coils (0x0F)\r\n"
        "Enter your choice: ";

    // Display the menu
    HAL_UART_Transmit(&huart3, (uint8_t*)menu, strlen(menu), HAL_MAX_DELAY);

    // Read user input (choice)
    HAL_UART_Receive(&huart3, (uint8_t*)buffer, 1, HAL_MAX_DELAY);
    choice = buffer[0] - '0';

    // Get slave address
    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Slave Address (1-247): ", 31, HAL_MAX_DELAY);
    HAL_UART_Receive(&huart3, (uint8_t*)buffer, 3, HAL_MAX_DELAY);
    slaveAddr = atoi(buffer);

    // Execute based on choice
    switch (choice)
    {
        case 1:  // Read Holding Registers
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Start Address: ", 23, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            startAddr = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Number of Registers: ", 29, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            numRegs = atoi(buffer);

            readHoldingRegisters_M(slaveAddr, startAddr, numRegs);
            break;

        case 2:  // Read Input Registers
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Start Address: ", 23, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            startAddr = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Number of Registers: ", 29, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            numRegs = atoi(buffer);

            readInputRegisters_M(slaveAddr, startAddr, numRegs);
            break;

        case 3:  // Read Coils
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Start Address: ", 23, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            startAddr = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Number of Coils: ", 24, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            numRegs = atoi(buffer);

            readCoils_M(slaveAddr, startAddr, numRegs);
            break;

        case 4:  // Read Discrete Inputs
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Start Address: ", 23, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            startAddr = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Number of Inputs: ", 26, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            numRegs = atoi(buffer);

            readDiscreteInputs_M(slaveAddr, startAddr, numRegs);
            break;

        case 5:  // Write Single Register
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Address: ", 17, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            startAddr = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Value: ", 15, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            value = atoi(buffer);

            writeSingleRegister_M(slaveAddr, startAddr, value);
            break;

        case 6:  // Write Multiple Registers
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Start Address: ", 23, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            startAddr = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Number of Registers: ", 29, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            numRegs = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Values (space separated): ", 34, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 32, HAL_MAX_DELAY);  // Example: "100 200 300"
            for (int i = 0; i < numRegs; i++) {
                values[i] = atoi(&buffer[i * 5]);  // Convert each value (space separated)
            }

            writeMultipleRegisters_M(slaveAddr, startAddr, numRegs, values);
            break;

        case 7:  // Write Single Coil
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Address: ", 17, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            startAddr = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Value (0 or 1): ", 24, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            value = atoi(buffer);

            if (value != 0 && value != 1) {
                HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nInvalid value. Only 0 or 1 allowed.", 37, HAL_MAX_DELAY);
            } else {
                writeSingleCoil_M(slaveAddr, startAddr, value);
            }
            break;

        case 8:  // Write Multiple Coils
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Start Address: ", 23, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            startAddr = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Number of Coils: ", 24, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 5, HAL_MAX_DELAY);
            numRegs = atoi(buffer);

            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nEnter Coil Values (space separated): ", 37, HAL_MAX_DELAY);
            HAL_UART_Receive(&huart3, (uint8_t*)buffer, 32, HAL_MAX_DELAY);  // Example: "1 0 1"
            for (int i = 0; i < (numRegs + 7) / 8; i++) {
                coils[i] = atoi(&buffer[i * 5]);  // Convert each value (space separated)
            }

            writeMultipleCoils_M(slaveAddr, startAddr, numRegs, coils);
            break;

        default:
            HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nInvalid choice.\r\n", 19, HAL_MAX_DELAY);
            break;
        }

    HAL_UART_Transmit(&huart3, (uint8_t*)"\r\nOperation complete.\r\n", 23, HAL_MAX_DELAY);
    //ShowMenu();
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8399;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Activer les horloges pour les Ports D et C
  	  __HAL_RCC_GPIOD_CLK_ENABLE();
  	  __HAL_RCC_GPIOC_CLK_ENABLE();

  	  // Configuration des broches du Port D (PD0 à PD15) en sortie
  	  GPIO_InitTypeDef GPIO_InitStruct = {0};
  	  GPIO_InitStruct.Pin = GPIO_PIN_All; // PD0 à PD15
  	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Mode sortie Push-Pull
  	  GPIO_InitStruct.Pull = GPIO_NOPULL; // Pas de résistance interne
  	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Vitesse basse
  	  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  	  // Configuration des broches du Port C (PC0 à PC15) en entrée
  	  GPIO_InitStruct.Pin = GPIO_PIN_All; // PC0 à PC15
  	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT; // Mode entrée
  	  GPIO_InitStruct.Pull = GPIO_PULLUP; // Résistance Pull-Up pour éviter des états flottants
  	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
