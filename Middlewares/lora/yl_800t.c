#include "yl_800t.h"
#include "string.h"
#include "usart.h"

#define PARAMETER_INDEX 8

#define COMMAND_WRITE_PARAMETERS 0x01
#define COMMAND_READ_PARAMETERS 0x02
#define PARAMETER_LENGTH_READ_WRITE 14

#define COMMAND_READ_SIGNAL_STRENGTH 0x06
#define PARAMETER_LENGTH_SIGNAL_STRENGTH 2

// Write message except for the parameter. Returns the total message length.
uint8_t writeMessage(uint8_t command, uint8_t parameterLength, uint8_t *message);
// Verifies the given received message.
int receiveMessage(uint8_t command, uint8_t parameterLength, const uint8_t *message);
uint8_t checksum(const uint8_t *message, uint8_t length);

void yl800tInit()
{
    RCC->APB2ENR |= 1<<2;    //使能PORTA时钟
    RCC->APB2ENR |= 1<<3;    //使能PORTB时钟

	GPIOA->CRH &= 0XFFFFFFF0;
	GPIOA->CRH |= 0X00000003;
    GPIOA->ODR |= 0x1<<8;
	GPIOB->CRH &= 0X00FFFFFF;
	GPIOB->CRH |= 0X83000000;
    GPIOB->ODR |= 0x3<<14;

    uart_init(72, 9600);
}

uint8_t yl800tSendReadAllParameters(uint8_t *messageOut)
{
    memset(&messageOut[PARAMETER_INDEX], 0, PARAMETER_LENGTH_READ_WRITE);
    return writeMessage(COMMAND_READ_PARAMETERS, PARAMETER_LENGTH_READ_WRITE, messageOut);
}

int yl800tReceiveReadAllParameters(const uint8_t *message, YL800TReadWriteAllParameters *parametersOut)
{
    int result = receiveMessage(COMMAND_READ_PARAMETERS, PARAMETER_LENGTH_READ_WRITE, message);
    if (result != 0) {
        return result;
    }
    const uint8_t *parameter         = &message[PARAMETER_INDEX];
    parametersOut->serialBaudRate    = parameter[0];
    parametersOut->serialParity      = parameter[1];
    parametersOut->rfFrequency       = (uint32_t)parameter[2] << 16 | (uint32_t)parameter[3] << 8 | parameter[4];
    parametersOut->rfSpreadingFactor = parameter[5];
    parametersOut->mode              = parameter[6];
    parametersOut->rfBandwidth       = parameter[7];
    parametersOut->nodeId            = parameter[8] << 8 | parameter[9];
    parametersOut->netId             = parameter[10];
    parametersOut->rfTransmitPower   = parameter[11];
    parametersOut->breathCycle       = parameter[12];
    parametersOut->breathTime        = parameter[13];
    return 0;
}

uint8_t yl800tSendWriteAllParameters(const YL800TReadWriteAllParameters *parameters, uint8_t *messageOut)
{
    uint8_t *parameter = &messageOut[PARAMETER_INDEX];
    parameter[0]       = parameters->serialBaudRate;
    parameter[1]       = parameters->serialParity;
    parameter[2]       = parameters->rfFrequency >> 16;
    parameter[3]       = parameters->rfFrequency >> 8;
    parameter[4]       = parameters->rfFrequency;
    parameter[5]       = parameters->rfSpreadingFactor;
    parameter[6]       = parameters->mode;
    parameter[7]       = parameters->rfBandwidth;
    parameter[8]       = parameters->nodeId >> 8;
    parameter[9]       = parameters->nodeId;
    parameter[10]      = parameters->netId;
    parameter[11]      = parameters->rfTransmitPower;
    parameter[12]      = parameters->breathCycle;
    parameter[13]      = parameters->breathTime;
    return writeMessage(COMMAND_WRITE_PARAMETERS, PARAMETER_LENGTH_READ_WRITE, messageOut);
}

int yl800tReceiveWriteAllParameters(const uint8_t *message)
{
    return receiveMessage(COMMAND_WRITE_PARAMETERS, PARAMETER_LENGTH_READ_WRITE, message);
}

uint8_t yl800tSendReadSignalStrength(uint8_t *messageOut)
{
    messageOut[PARAMETER_INDEX]     = 0;
    messageOut[PARAMETER_INDEX + 1] = 0;
    return writeMessage(COMMAND_READ_SIGNAL_STRENGTH, PARAMETER_LENGTH_SIGNAL_STRENGTH, messageOut);
}

int yl800tReceiveReadSignalStrength(const uint8_t *message, uint8_t *strengthOut)
{
    int result = receiveMessage(COMMAND_READ_SIGNAL_STRENGTH, PARAMETER_LENGTH_SIGNAL_STRENGTH, message);
    if (result != 0) {
        return result;
    }
    *strengthOut = message[PARAMETER_INDEX];
    return result;
}

uint8_t writeMessage(uint8_t command, uint8_t parameterLength, uint8_t *messageOut)
{
    messageOut[0]                  = 0xAF;
    messageOut[1]                  = 0xAF;
    messageOut[2]                  = 0x00;
    messageOut[3]                  = 0x00;
    messageOut[4]                  = 0xAF;
    messageOut[5]                  = 0x80;
    messageOut[6]                  = command;
    messageOut[7]                  = parameterLength;
    uint8_t lengthBeforeCs         = PARAMETER_INDEX + parameterLength;
    messageOut[lengthBeforeCs]     = checksum(messageOut, lengthBeforeCs);
    messageOut[lengthBeforeCs + 1] = 0x0D;
    messageOut[lengthBeforeCs + 2] = 0x0A;

    return lengthBeforeCs + 3;
}

int receiveMessage(uint8_t command, uint8_t parameterLength, const uint8_t *message)
{
    uint8_t lengthBeforeCs = PARAMETER_INDEX + parameterLength;
    if (
        message[0] != 0xAF || message[1] != 0xAF || message[2] != 0x00 || message[3] != 0x00 ||
        message[4] != 0xAF || message[5] != 0x00 || message[6] != command || message[7] != parameterLength ||
        message[lengthBeforeCs] != checksum(message, lengthBeforeCs) ||
        message[lengthBeforeCs + 1] != 0x0D || message[lengthBeforeCs + 2] != 0x0A) {
        return 1;
    } else {
        return 0;
    }
}

uint8_t checksum(const uint8_t *message, uint8_t length)
{
    uint8_t result = 0;
    for (uint8_t i = 0; i < length; ++i) {
        result += message[i];
    }
    return result;
}
