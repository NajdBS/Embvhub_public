/*
 * modbusmaster.h
 *
 *  Created on: Dec 15, 2024
 *      Author: NajdBS
 */

#ifndef INC_MODBUSMASTER_H_
#define INC_MODBUSMASTER_H_

#include <stdint.h> // Pour les types uint8_t, uint16_t
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart2;
extern uint8_t TxData_M[8];


extern char buffer[32];
extern uint8_t choice;
extern uint8_t slaveAddr;
extern uint16_t startAddr, numRegs, value;
extern uint16_t values[10];
extern uint8_t coils[10];


// Prototype pour lire les registres de maintien (0x03)
void readHoldingRegisters_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numRegs);

// Prototype pour lire les registres d'entrée (0x04)
void readInputRegisters_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numRegs);

// Prototype pour lire les bobines (0x01)
void readCoils_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numCoils);

// Prototype pour lire les entrées discrètes (0x02)
void readDiscreteInputs_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numInputs);

// Prototype pour écrire dans un registre unique (0x06)
void writeSingleRegister_M(uint8_t slaveAddr, uint16_t addr, uint16_t value);

// Prototype pour écrire dans plusieurs registres (0x10)
void writeMultipleRegisters_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numRegs, uint16_t* values);

// Prototype pour écrire dans une seule bobine (0x05)
void writeSingleCoil_M(uint8_t slaveAddr, uint16_t addr, uint8_t value);

// Prototype pour écrire dans plusieurs bobines (0x0F)
void writeMultipleCoils_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numCoils, uint8_t* values);


#endif /* INC_MODBUSMASTER_H_ */
