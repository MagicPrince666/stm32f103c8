/*
 * Functions and type definitions to help with building and parsing messages for the YL-800T module.
 */

#ifndef _YL_800T_H
#define _YL_800T_H

#include "sys.h"
#include <inttypes.h>

#define LORA_AUX PBin(15)
#define LORA_EN PAout(8)
#define LORA_SET PBout(14)

typedef enum YL800TSerialBaudRate {
    YL_800T_BAUD_RATE_1200   = 0x01,
    YL_800T_BAUD_RATE_2400   = 0x02,
    YL_800T_BAUD_RATE_4800   = 0x03,
    YL_800T_BAUD_RATE_9600   = 0x04,
    YL_800T_BAUD_RATE_19200  = 0x05,
    YL_800T_BAUD_RATE_38400  = 0x06,
    YL_800T_BAUD_RATE_57600  = 0x07,
    YL_800T_BAUD_RATE_115200 = 0x08
} YL800TSerialBaudRate;

typedef enum YL800TSerialParity {
    YL_800T_PARITY_NONE = 0x00,
    YL_800T_PARITY_ODD  = 0x01,
    YL_800T_PARITY_EVEN = 0x02
} YL800TSerialParity;

typedef enum YL800TRFSpreadingFactor {
    YL_800T_RF_SPREADING_FACTOR_128  = 0x07,
    YL_800T_RF_SPREADING_FACTOR_256  = 0x08,
    YL_800T_RF_SPREADING_FACTOR_512  = 0x09,
    YL_800T_RF_SPREADING_FACTOR_1024 = 0x0A,
    YL_800T_RF_SPREADING_FACTOR_2048 = 0x0B,
    YL_800T_RF_SPREADING_FACTOR_4096 = 0x0C
} YL800TRFSpreadingFactor;

typedef enum YL800TMode {
    YL_800T_RF_MODE_STANDARD = 0x00,
    YL_800T_RF_MODE_CENTRAL  = 0x01,
    YL_800T_RF_MODE_NODE     = 0x02
} YL800TMode;

typedef enum YL800TRFBandwidth {
    YL_800T_RF_BANDWIDTH_62_5K = 0x06,
    YL_800T_RF_BANDWIDTH_125K  = 0x07,
    YL_800T_RF_BANDWIDTH_250K  = 0x08,
    YL_800T_RF_BANDWIDTH_500K  = 0x09
} YL800TRFBandwidth;

typedef enum YL800TBreathCycle {
    YL_800T_BREATH_CYCLE_2S  = 0x00,
    YL_800T_BREATH_CYCLE_4S  = 0x01,
    YL_800T_BREATH_CYCLE_6S  = 0x02,
    YL_800T_BREATH_CYCLE_8S  = 0x03,
    YL_800T_BREATH_CYCLE_10S = 0x04
} YL800TBreathCycle;

typedef enum YL800TBreathTime {
    YL_800T_BREATH_TIME_2MS  = 0x00,
    YL_800T_BREATH_TIME_4MS  = 0x01,
    YL_800T_BREATH_TIME_8MS  = 0x02,
    YL_800T_BREATH_TIME_16MS = 0x03,
    YL_800T_BREATH_TIME_32MS = 0x04,
    YL_800T_BREATH_TIME_64MS = 0x05
} YL800TBreathTime;

typedef struct YL800TReadWriteAllParameters {
    /* The speed of serial communication over the UART interface. */
    uint8_t serialBaudRate;
    /* The serial partity check bit option. */
    uint8_t serialParity;
    /*
     * The RF frequency in 1/2^14 MHz units. E.g. 334 MHz is 334*2^14=5472256
     *
     * Valid values are 420-510 MHz for Sx1278 based modules, or 850-930 MHz for Sx1276 based modules.
     */
    uint32_t rfFrequency;
    /*
     * The spread spectrum factor as a power of 2 between 2^7 and 2^12.
     *
     * A higher factor will increase receive sensitivity but decrease transmission speed.
     * Only valid for standard mode.
     */
    uint8_t rfSpreadingFactor;
    uint8_t mode;
    /*
     * The spread spectrum bandwidth. Determines the frequency of signal modulation.
     *
     * A lower value will increase receive sensitivity but decrease transmission speed. A value of 125K is recommended.
     * Only valid for standard mode.
     */
    uint8_t rfBandwidth;
    /*
     * The ID or address of a node module.
     *
     * The first two bytes sent to the central module determine the id of the node which should receive the data.
     * Data from the central module with node id 0xFFFF will be broadcasted to all nodes regardless of this value.
     * With node id set to 0x0000 all transmissions from the central module are received.
     *
     * It is sent with most significant byte first.
     *
     * Only valid for node mode.
     */
    uint16_t nodeId;
    /*
     * The network id.
     *
     * The value must match between any two modules in order for them to communicate.
     */
    uint8_t netId;
    /*
     * The RF transmission power. Valid values are from 1 through to 7.
     *
     * The current consumed during transmission will depend on this setting.
     * | Value  | Power (dBm) | Current (mA) |
     * | ------ | ----------- | ------------ |
     * | 1      | 5.5-6.5     | 30-40        |
     * | 2      | 5.5-6.5     | 30-40        |
     * | 3      | 8.5-9.8     | 40-45        |
     * | 4      | 11.5-12.5   | 45-55        |
     * | 5      | 14.5-15.5   | 60-70        |
     * | 6      | 17.5-18     | 90-100       |
     * | 7      | 19.5-20     | 110-120      |
     */
    uint8_t rfTransmitPower;
    /*
     * The sleep or dormant time for star network communication.
     * The time that the node module is asleep between each breath.
     *
     * Only valid for central and node mode.
     */
    uint8_t breathCycle;
    /*
     * The wake time for star network communication.
     *
     * With a longer wake time, receive sensitivity is increased, power consumption is higher, and transmission speed is lower.
     *
     * Only valid for central and node mode.
     */
    uint8_t breathTime;
} YL800TReadWriteAllParameters;

void yl800tInit();

/*
 * Build message to request the current value for all parameters.
 * The message is written to buffer `messageOut`. `messageOut` must be at least 25 bytes long.
 * Returns the length of the message in bytes.
 */
uint8_t yl800tSendReadAllParameters(uint8_t *messageOut);

/*
 * Parse response to request for the current value of all parameters.
 * Returns 0 on success.
 */
int yl800tReceiveReadAllParameters(const uint8_t *message, YL800TReadWriteAllParameters *parametersOut);

/*
 * Build message to update all parameter values.
 * The message is written to buffer `messageOut`. `messageOut` must be at least 25 bytes long.
 * Returns the length of the message in bytes.
 */
uint8_t yl800tSendWriteAllParameters(const YL800TReadWriteAllParameters *parameters, uint8_t *messageOut);

/*
 * Parse response to request to update the value of all parameters.
 * Returns 0 on success.
 */
int yl800tReceiveWriteAllParameters(const uint8_t *message);

/*
 * Build message to read the received signal strength for the last packet.
 * The message is written to buffer `messageOut`. `messageOut` must be at least 13 bytes long.
 * Returns the length of the message in bytes.
 */
uint8_t yl800tSendReadSignalStrength(uint8_t *messageOut);

/*
 * Parse response to request to read the received signal strength for the last packet.
 * The signal strength in dBm + 164 is written to strengthOut.
 * Returns 0 on success.
 */
int yl800tReceiveReadSignalStrength(const uint8_t *message, uint8_t *strengthOut);
#endif
