#pragma once

/*!
 * \file      sx126x.h
 *
 * \brief     SX126x driver implementation
 *
 * \copyright Revised BSD License, see file LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#ifndef __SX126x_H__
#define __SX126x_H__

#define SX1261 1
#define SX1262 2

/*!
     * Radio complete Wake-up Time with TCXO stabilisation time
     */
#define RADIO_TCXO_SETUP_TIME 50 // [ms]

/*!
 * Radio complete Wake-up Time with margin for temperature compensation
 */
#define RADIO_WAKEUP_TIME 3 // [ms]

/*!
 * \brief Compensation delay for SetAutoTx/Rx functions in 15.625 microseconds
 */
#define AUTO_RX_TX_OFFSET 2

/*!
 * \brief LFSR initial value to compute IBM type CRC
 */
#define CRC_IBM_SEED 0xFFFF

/*!
 * \brief LFSR initial value to compute CCIT type CRC
 */
#define CRC_CCITT_SEED 0x1D0F

/*!
 * \brief Polynomial used to compute IBM CRC
 */
#define CRC_POLYNOMIAL_IBM 0x8005

/*!
 * \brief Polynomial used to compute CCIT CRC
 */
#define CRC_POLYNOMIAL_CCITT 0x1021

/*!
 * \brief The address of the register holding the first byte defining the CRC seed
 *
 */
#define REG_LR_CRCSEEDBASEADDR 0x06BC

/*!
 * \brief The address of the register holding the first byte defining the CRC polynomial
 */
#define REG_LR_CRCPOLYBASEADDR 0x06BE

/*!
 * \brief The address of the register holding the first byte defining the whitening seed
 */
#define REG_LR_WHITSEEDBASEADDR_MSB 0x06B8
#define REG_LR_WHITSEEDBASEADDR_LSB 0x06B9

/*!
 * \brief The address of the register holding the packet configuration
 */
#define REG_LR_PACKETPARAMS 0x0704

/*!
 * \brief The address of the register holding the payload size
 */
#define REG_LR_PAYLOADLENGTH 0x0702

/*!
 * \brief The addresses of the registers holding SyncWords values
 */
#define REG_LR_SYNCWORDBASEADDRESS 0x06C0

/*!
 * \brief The addresses of the register holding LoRa Modem SyncWord value
 */
#define REG_LR_SYNCWORD 0x0740

/*!
 * Syncword for Private LoRa networks
 */
#define LORA_MAC_PRIVATE_SYNCWORD 0x1424

/*!
 * Syncword for Public LoRa networks
 */
#define LORA_MAC_PUBLIC_SYNCWORD 0x3444

/*!
 * The address of the register giving a 4 bytes random number
 */
#define RANDOM_NUMBER_GENERATORBASEADDR 0x0819

/*!
 * The address of the register holding RX Gain value (0x94: power saving, 0x96: rx boosted)
 */
#define REG_RX_GAIN 0x08AC

/*!
 * Change the value on the device internal trimming capacitor
 */
#define REG_XTA_TRIM 0x0911

/*!
 * Set the current max value in the over current protection
 */
#define REG_OCP 0x08E7

/*!
 * \brief Structure describing the radio status
 */
typedef union RadioStatus_u
{
	uint8_t Value;
	struct
	{						   //bit order is lsb -> msb
		uint8_t Reserved : 1;  //!< Reserved
		uint8_t CmdStatus : 3; //!< Command status
		uint8_t ChipMode : 3;  //!< Chip mode
		uint8_t CpuBusy : 1;   //!< Flag for CPU radio busy
	} Fields;
} SX126X_RadioStatus_t;

/*!
 * \brief Structure describing the error codes for callback functions
 */
typedef enum
{
	SX126X_IRQ_HEADER_ERROR_CODE = 0x01,
	SX126X_IRQ_SYNCWORD_ERROR_CODE = 0x02,
	SX126X_IRQ_CRC_ERROR_CODE = 0x04,
} IrqErrorCode_t;

enum SX126X_IrqPblSyncHeaderCode_t
{
	SX126X_IRQ_PBL_DETECT_CODE = 0x01,
	SX126X_IRQ_SYNCWORD_VALID_CODE = 0x02,
	SX126X_IRQ_HEADER_VALID_CODE = 0x04,
};

/*!
 * \brief Represents the operating mode the radio is actually running
 */
typedef enum
{
	SX126X_MODE_SLEEP = 0x00, //! The radio is in sleep mode
	SX126X_MODE_STDBY_RC,	   //! The radio is in standby mode with RC oscillator
	SX126X_MODE_STDBY_XOSC,   //! The radio is in standby mode with XOSC oscillator
	SX126X_MODE_FS,		   //! The radio is in frequency synthesis mode
	SX126X_MODE_TX,		   //! The radio is in transmit mode
	SX126X_MODE_RX,		   //! The radio is in receive mode
	SX126X_MODE_RX_DC,		   //! The radio is in receive duty cycle mode
	SX126X_MODE_CAD		   //! The radio is in channel activity detection mode
} SX126X_RadioOperatingModes_t;

/*!
 * \brief Declares the oscillator in use while in standby mode
 *
 * Using the STDBY_RC standby mode allow to reduce the energy consumption
 * STDBY_XOSC should be used for time critical applications
 */
typedef enum
{
	SX126X_STDBY_RC = 0x00,
	SX126X_STDBY_XOSC = 0x01,
} SX126X_RadioStandbyModes_t;

/*!
 * \brief Declares the power regulation used to power the device
 *
 * This command allows the user to specify if DC-DC or LDO is used for power regulation.
 * Using only LDO implies that the Rx or Tx current is doubled
 */
typedef enum
{
	SX126X_USE_LDO = 0x00, // default
	SX126X_USE_DCDC = 0x01,
} SX126X_RadioRegulatorMode_t;

/*!
 * \brief Represents the possible packet type (i.e. modem) used
 */
typedef enum
{
	SX126X_PACKET_TYPE_GFSK = 0x00,
	SX126X_PACKET_TYPE_LORA = 0x01,
	SX126X_PACKET_TYPE_NONE = 0x0F,
} SX126X_RadioPacketTypes_t;

/*!
 * \brief Represents the ramping time for power amplifier
 */
typedef enum
{
	SX126X_RADIO_RAMP_10_US = 0x00,
	SX126X_RADIO_RAMP_20_US = 0x01,
	SX126X_RADIO_RAMP_40_US = 0x02,
	SX126X_RADIO_RAMP_80_US = 0x03,
	SX126X_RADIO_RAMP_200_US = 0x04,
	SX126X_RADIO_RAMP_800_US = 0x05,
	SX126X_RADIO_RAMP_1700_US = 0x06,
	SX126X_RADIO_RAMP_3400_US = 0x07,
} SX126X_RadioRampTimes_t;

/*!
 * \brief Represents the number of symbols to be used for channel activity detection operation
 */
typedef enum
{
	SX126X_LORA_CAD_01_SYMBOL = 0x00,
	SX126X_LORA_CAD_02_SYMBOL = 0x01,
	SX126X_LORA_CAD_04_SYMBOL = 0x02,
	SX126X_LORA_CAD_08_SYMBOL = 0x03,
	SX126X_LORA_CAD_16_SYMBOL = 0x04,
} SX126X_RadioLoRaCadSymbols_t;

/*!
 * \brief Represents the Channel Activity Detection actions after the CAD operation is finished
 */
typedef enum
{
	SX126X_LORA_CAD_ONLY = 0x00,
	SX126X_LORA_CAD_RX = 0x01,
	SX126X_LORA_CAD_LBT = 0x10,
} SX126X_RadioCadExitModes_t;

/*!
 * \brief Represents the modulation shaping parameter
 */
typedef enum
{
	SX126X_MOD_SHAPING_OFF = 0x00,
	SX126X_MOD_SHAPING_G_BT_03 = 0x08,
	SX126X_MOD_SHAPING_G_BT_05 = 0x09,
	SX126X_MOD_SHAPING_G_BT_07 = 0x0A,
	SX126X_MOD_SHAPING_G_BT_1 = 0x0B,
} SX126X_RadioModShapings_t;

/*!
 * \brief Represents the modulation shaping parameter
 */
typedef enum
{
	SX126X_RX_BW_4800 = 0x1F,
	SX126X_RX_BW_5800 = 0x17,
	SX126X_RX_BW_7300 = 0x0F,
	SX126X_RX_BW_9700 = 0x1E,
	SX126X_RX_BW_11700 = 0x16,
	SX126X_RX_BW_14600 = 0x0E,
	SX126X_RX_BW_19500 = 0x1D,
	SX126X_RX_BW_23400 = 0x15,
	SX126X_RX_BW_29300 = 0x0D,
	SX126X_RX_BW_39000 = 0x1C,
	SX126X_RX_BW_46900 = 0x14,
	SX126X_RX_BW_58600 = 0x0C,
	SX126X_RX_BW_78200 = 0x1B,
	SX126X_RX_BW_93800 = 0x13,
	SX126X_RX_BW_117300 = 0x0B,
	SX126X_RX_BW_156200 = 0x1A,
	SX126X_RX_BW_187200 = 0x12,
	SX126X_RX_BW_234300 = 0x0A,
	SX126X_RX_BW_312000 = 0x19,
	SX126X_RX_BW_373600 = 0x11,
	SX126X_RX_BW_467000 = 0x09,
} SX126X_RadioRxBandwidth_t;

/*!
 * \brief Represents the possible spreading factor values in LoRa packet types
 */
typedef enum
{
	SX126X_LORA_SF5 = 0x05,
	SX126X_LORA_SF6 = 0x06,
	SX126X_LORA_SF7 = 0x07,
	SX126X_LORA_SF8 = 0x08,
	SX126X_LORA_SF9 = 0x09,
	SX126X_LORA_SF10 = 0x0A,
	SX126X_LORA_SF11 = 0x0B,
	SX126X_LORA_SF12 = 0x0C,
} SX126X_RadioLoRaSpreadingFactors_t;

/*!
 * \brief Represents the bandwidth values for LoRa packet type
 */
typedef enum
{
	SX126X_LORA_BW_500 = 6,
	SX126X_LORA_BW_250 = 5,
	SX126X_LORA_BW_125 = 4,
	SX126X_LORA_BW_062 = 3,
	SX126X_LORA_BW_041 = 10,
	SX126X_LORA_BW_031 = 2,
	SX126X_LORA_BW_020 = 9,
	SX126X_LORA_BW_015 = 1,
	SX126X_LORA_BW_010 = 8,
	SX126X_LORA_BW_007 = 0,
} SX126X_RadioLoRaBandwidths_t;//RadioLoRaBandwidths_t;

/*!
 * \brief Represents the coding rate values for LoRa packet type
 */
typedef enum
{
	SX126X_LORA_CR_4_5 = 0x01,
	SX126X_LORA_CR_4_6 = 0x02,
	SX126X_LORA_CR_4_7 = 0x03,
	SX126X_LORA_CR_4_8 = 0x04,
} SX126X_RadioLoRaCodingRates_t;

/*!
 * \brief Represents the preamble length used to detect the packet on Rx side
 */
typedef enum
{
	SX126X_RADIO_PREAMBLE_DETECTOR_OFF = 0x00,		//!< Preamble detection length off
	SX126X_RADIO_PREAMBLE_DETECTOR_08_BITS = 0x04, //!< Preamble detection length 8 bits
	SX126X_RADIO_PREAMBLE_DETECTOR_16_BITS = 0x05, //!< Preamble detection length 16 bits
	SX126X_RADIO_PREAMBLE_DETECTOR_24_BITS = 0x06, //!< Preamble detection length 24 bits
	SX126X_RADIO_PREAMBLE_DETECTOR_32_BITS = 0x07, //!< Preamble detection length 32 bit
} SX126X_RadioPreambleDetection_t;

/*!
 * \brief Represents the possible combinations of SyncWord correlators activated
 */
typedef enum
{
	SX126X_RADIO_ADDRESSCOMP_FILT_OFF = 0x00, //!< No correlator turned on, i.e. do not search for SyncWord
	SX126X_RADIO_ADDRESSCOMP_FILT_NODE = 0x01,
	SX126X_RADIO_ADDRESSCOMP_FILT_NODE_BROAD = 0x02,
} SX126X_RadioAddressComp_t;

/*!
 *  \brief Radio GFSK packet length mode
 */
typedef enum
{
	SX126X_RADIO_PACKET_FIXED_LENGTH = 0x00,	 //!< The packet is known on both sides, no header included in the packet
	SX126X_RADIO_PACKET_VARIABLE_LENGTH = 0x01, //!< The packet is on variable size, header included
} SX126X_RadioPacketLengthModes_t;

/*!
 * \brief Represents the CRC length
 */
typedef enum
{
	SX126X_RADIO_CRC_OFF = 0x01, //!< No CRC in use
	SX126X_RADIO_CRC_1_BYTES = 0x00,
	SX126X_RADIO_CRC_2_BYTES = 0x02,
	SX126X_RADIO_CRC_1_BYTES_INV = 0x04,
	SX126X_RADIO_CRC_2_BYTES_INV = 0x06,
	SX126X_RADIO_CRC_2_BYTES_IBM = 0xF1,
	SX126X_RADIO_CRC_2_BYTES_CCIT = 0xF2,
} SX126X_RadioCrcTypes_t;

/*!
 * \brief Radio whitening mode activated or deactivated
 */
typedef enum
{
	SX126X_RADIO_DC_FREE_OFF = 0x00,
	SX126X_RADIO_DC_FREEWHITENING = 0x01,
} SX126X_RadioDcFree_t;

/*!
 * \brief Holds the Radio lengths mode for the LoRa packet type
 */
typedef enum
{
	SX126X_LORA_PACKET_VARIABLE_LENGTH = 0x00, //!< The packet is on variable size, header included
	SX126X_LORA_PACKET_FIXED_LENGTH = 0x01,	//!< The packet is known on both sides, no header included in the packet
	SX126X_LORA_PACKET_EXPLICIT = SX126X_LORA_PACKET_VARIABLE_LENGTH,
	SX126X_LORA_PACKET_IMPLICIT = SX126X_LORA_PACKET_FIXED_LENGTH,
} SX126X_RadioLoRaPacketLengthMode_t;

/*!
 * \brief Represents the CRC mode for LoRa packet type
 */
typedef enum
{
	SX126X_LORA_CRC_ON = 0x01,	 //!< CRC activated
	SX126X_LORA_CRC_OFF = 0x00, //!< CRC not used
} SX126X_RadioLoRaCrcModes_t;

/*!
 * \brief Represents the IQ mode for LoRa packet type
 */
typedef enum
{
	SX126X_LORA_IQ_NORMAL = 0x00,
	SX126X_LORA_IQ_INVERTED = 0x01,
} SX126X_RadioLoRaIQModes_t;

/*!
 * \brief Represents the voltage used to control the TCXO on/off from DIO3
 */
typedef enum
{
	SX126X_TCXO_CTRL_1_6V = 0x00,
	SX126X_TCXO_CTRL_1_7V = 0x01,
	SX126X_TCXO_CTRL_1_8V = 0x02,
	SX126X_TCXO_CTRL_2_2V = 0x03,
	SX126X_TCXO_CTRL_2_4V = 0x04,
	SX126X_TCXO_CTRL_2_7V = 0x05,
	SX126X_TCXO_CTRL_3_0V = 0x06,
	SX126X_TCXO_CTRL_3_3V = 0x07,
} SX126X_RadioTcxoCtrlVoltage_t;

/*!
 * \brief Represents the interruption masks available for the radio
 *
 * \remark Note that not all these interruptions are available for all packet types
 */
typedef enum
{
	SX126X_IRQ_RADIO_NONE = 0x0000,
	SX126X_IRQ_TX_DONE = 0x0001,
	SX126X_IRQ_RX_DONE = 0x0002,
	SX126X_IRQ_PREAMBLE_DETECTED = 0x0004,
	SX126X_IRQ_SYNCWORD_VALID = 0x0008,
	SX126X_IRQ_HEADER_VALID = 0x0010,
	SX126X_IRQ_HEADER_ERROR = 0x0020,
	SX126X_IRQ_CRC_ERROR = 0x0040,
	SX126X_IRQ_CAD_DONE = 0x0080,
	SX126X_IRQ_CAD_ACTIVITY_DETECTED = 0x0100,
	SX126X_IRQ_RX_TX_TIMEOUT = 0x0200,
	SX126X_IRQ_RADIO_ALL = 0xFFFF,
} SX126X_RadioIrqMasks_t;

/*!
 * \brief Represents all possible opcode understood by the radio
 */
typedef enum RadioCommands_e
{
	SX126X_RADIO_GET_STATUS = 0xC0,
	SX126X_RADIO_WRITE_REGISTER = 0x0D,
	SX126X_RADIO_READ_REGISTER = 0x1D,
	Q2W4SX126X_RADIO_WRITE_BUFFER = 0x0E,
	SX126X_RADIO_READ_BUFFER = 0x1E,
	SX126X_RADIO_SET_SLEEP = 0x84,
	SX126X_RADIO_SET_STANDBY = 0x80,
	SX126X_RADIO_SET_FS = 0xC1,
	SX126X_RADIO_SET_TX = 0x83,
	SX126X_RADIO_SET_RX = 0x82,
	SX126X_RADIO_SET_RXDUTYCYCLE = 0x94,
	SX126X_RADIO_SET_CAD = 0xC5,
	SX126X_RADIO_SET_TXCONTINUOUSWAVE = 0xD1,
	SX126X_RADIO_SET_TXCONTINUOUSPREAMBLE = 0xD2,
	SX126X_RADIO_SET_PACKETTYPE = 0x8A,
	SX126X_RADIO_GET_PACKETTYPE = 0x11,
	SX126X_RADIO_SET_RFFREQUENCY = 0x86,
	SX126X_RADIO_SET_TXPARAMS = 0x8E,
	SX126X_RADIO_SET_PACONFIG = 0x95,
	SX126X_RADIO_SET_CADPARAMS = 0x88,
	SX126X_RADIO_SET_BUFFERBASEADDRESS = 0x8F,
	SX126X_RADIO_SET_MODULATIONPARAMS = 0x8B,
	SX126X_RADIO_SET_PACKETPARAMS = 0x8C,
	SX126X_RADIO_GET_RXBUFFERSTATUS = 0x13,
	SX126X_RADIO_GET_PACKETSTATUS = 0x14,
	SX126X_RADIO_GET_RSSIINST = 0x15,
	SX126X_RADIO_GET_STATS = 0x10,
	SX126X_RADIO_RESET_STATS = 0x00,
	SX126X_RADIO_CFG_DIOIRQ = 0x08,
	SX126X_RADIO_GET_IRQSTATUS = 0x12,
	SX126X_RADIO_CLR_IRQSTATUS = 0x02,
	SX126X_RADIO_CALIBRATE = 0x89,
	SX126X_RADIO_CALIBRATEIMAGE = 0x98,
	SX126X_RADIO_SET_REGULATORMODE = 0x96,
	SX126X_RADIO_GET_ERROR = 0x17,
	SX126X_RADIO_CLR_ERROR = 0x07,
	SX126X_RADIO_SET_TCXOMODE = 0x97,
	SX126X_RADIO_SET_TXFALLBACKMODE = 0x93,
	SX126X_RADIO_SET_RFSWITCHMODE = 0x9D,
	SX126X_RADIO_SET_STOPRXTIMERONPREAMBLE = 0x9F,
	SX126X_RADIO_SET_LORASYMBTIMEOUT = 0xA0,
} SX126X_RadioCommands_t;






#endif // __SX126x_H__
