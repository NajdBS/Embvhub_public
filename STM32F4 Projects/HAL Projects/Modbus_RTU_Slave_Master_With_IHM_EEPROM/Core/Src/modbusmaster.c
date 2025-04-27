/*
 * modbusmaster.c
 *
 *  Created on: Dec 15, 2024
 *      Author: NajdBS
 */
#include "modbusmaster.h"

void readHoldingRegisters_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numRegs)
{
  TxData_M[0] = slaveAddr;
  TxData_M[1] = 0x03;  // Function code for Read Holding Registers
  TxData_M[2] = (startAddr >> 8) & 0xFF;
  TxData_M[3] = startAddr & 0xFF;
  TxData_M[4] = (numRegs >> 8) & 0xFF;
  TxData_M[5] = numRegs & 0xFF;

  uint16_t crc = crc16(TxData_M, 6);
  TxData_M[6] = crc & 0xFF;
  TxData_M[7] = (crc >> 8) & 0xFF;

  HAL_UART_Transmit(&huart2, TxData_M, 8, 1000);
}

/**
  * @brief  Reads Input Registers (0x04)
  */
void readInputRegisters_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numRegs)
{
  TxData_M[0] = slaveAddr;
  TxData_M[1] = 0x04;  // Function code for Read Input Registers
  TxData_M[2] = (startAddr >> 8) & 0xFF;
  TxData_M[3] = startAddr & 0xFF;
  TxData_M[4] = (numRegs >> 8) & 0xFF;
  TxData_M[5] = numRegs & 0xFF;

  uint16_t crc = crc16(TxData_M, 6);
  TxData_M[6] = crc & 0xFF;
  TxData_M[7] = (crc >> 8) & 0xFF;

  HAL_UART_Transmit(&huart2, TxData_M, 8, 1000);
}

/**
  * @brief  Reads Coils (0x01)
  */
void readCoils_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numCoils)
{
  TxData_M[0] = slaveAddr;
  TxData_M[1] = 0x01;  // Function code for Read Coils
  TxData_M[2] = (startAddr >> 8) & 0xFF;
  TxData_M[3] = startAddr & 0xFF;
  TxData_M[4] = (numCoils >> 8) & 0xFF;
  TxData_M[5] = numCoils & 0xFF;

  uint16_t crc = crc16(TxData_M, 6);
  TxData_M[6] = crc & 0xFF;
  TxData_M[7] = (crc >> 8) & 0xFF;

  HAL_UART_Transmit(&huart2, TxData_M, 8, 1000);
}

/**
  * @brief  Reads Discrete Inputs (0x02)
  */
void readDiscreteInputs_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numInputs)
{
  TxData_M[0] = slaveAddr;
  TxData_M[1] = 0x02;  // Function code for Read Discrete Inputs
  TxData_M[2] = (startAddr >> 8) & 0xFF;
  TxData_M[3] = startAddr & 0xFF;
  TxData_M[4] = (numInputs >> 8) & 0xFF;
  TxData_M[5] = numInputs & 0xFF;

  uint16_t crc = crc16(TxData_M, 6);
  TxData_M[6] = crc & 0xFF;
  TxData_M[7] = (crc >> 8) & 0xFF;

  HAL_UART_Transmit(&huart2, TxData_M, 8, 1000);
}

/**
  * @brief  Writes Single Register (0x06)
  */
void writeSingleRegister_M(uint8_t slaveAddr, uint16_t addr, uint16_t value)
{
  TxData_M[0] = slaveAddr;
  TxData_M[1] = 0x06;  // Function code for Write Single Register
  TxData_M[2] = (addr >> 8) & 0xFF;
  TxData_M[3] = addr & 0xFF;
  TxData_M[4] = (value >> 8) & 0xFF;
  TxData_M[5] = value & 0xFF;

  uint16_t crc = crc16(TxData_M, 6);
  TxData_M[6] = crc & 0xFF;
  TxData_M[7] = (crc >> 8) & 0xFF;

  HAL_UART_Transmit(&huart2, TxData_M, 8, 1000);
}

/**
  * @brief  Writes Multiple Registers (0x10)
  */
void writeMultipleRegisters_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numRegs, uint16_t* values)
{
  TxData_M[0] = slaveAddr;
  TxData_M[1] = 0x10;  // Function code for Write Multiple Registers
  TxData_M[2] = (startAddr >> 8) & 0xFF;
  TxData_M[3] = startAddr & 0xFF;
  TxData_M[4] = (numRegs >> 8) & 0xFF;
  TxData_M[5] = numRegs & 0xFF;
  TxData_M[6] = numRegs * 2;  // Number of bytes to write

  // Add values to TxData
  for (int i = 0; i < numRegs; i++) {
	TxData_M[7 + 2 * i] = (values[i] >> 8) & 0xFF;
	TxData_M[8 + 2 * i] = values[i] & 0xFF;
  }

  uint16_t crc = crc16(TxData_M, 6 + numRegs * 2);
  TxData_M[6 + numRegs * 2] = crc & 0xFF;
  TxData_M[7 + numRegs * 2] = (crc >> 8) & 0xFF;

  HAL_UART_Transmit(&huart2, TxData_M, 8 + numRegs * 2, 1000);
}

/**
  * @brief  Writes Single Coil (0x05)
  */
void writeSingleCoil_M(uint8_t slaveAddr, uint16_t addr, uint8_t value)
{
  TxData_M[0] = slaveAddr;
  TxData_M[1] = 0x05;  // Function code for Write Single Coil
  TxData_M[2] = (addr >> 8) & 0xFF;
  TxData_M[3] = addr & 0xFF;
  TxData_M[4] = (value == 0) ? 0x00 : 0xFF;
  TxData_M[5] = 0x00;

  uint16_t crc = crc16(TxData_M, 6);
  TxData_M[6] = crc & 0xFF;
  TxData_M[7] = (crc >> 8) & 0xFF;

  HAL_UART_Transmit(&huart2, TxData_M, 8, 1000);
}

/**
  * @brief  Writes Multiple Coils (0x0F)
  */
void writeMultipleCoils_M(uint8_t slaveAddr, uint16_t startAddr, uint16_t numCoils, uint8_t* values)
{
  TxData_M[0] = slaveAddr;
  TxData_M[1] = 0x0F;  // Function code for Write Multiple Coils
  TxData_M[2] = (startAddr >> 8) & 0xFF;
  TxData_M[3] = startAddr & 0xFF;
  TxData_M[4] = (numCoils >> 8) & 0xFF;
  TxData_M[5] = numCoils & 0xFF;
  TxData_M[6] = (numCoils + 7) / 8;  // Number of bytes needed to represent coils

  // Add coil values to TxData
  for (int i = 0; i < (numCoils + 7) / 8; i++) {
	  TxData_M[7 + i] = values[i];
  }

  uint16_t crc = crc16(TxData_M, 6 + (numCoils + 7) / 8);
  TxData_M[7 + (numCoils + 7) / 8] = crc & 0xFF;
  TxData_M[8 + (numCoils + 7) / 8] = (crc >> 8) & 0xFF;

  HAL_UART_Transmit(&huart2, TxData_M, 9 + (numCoils + 7) / 8, 1000);
}
