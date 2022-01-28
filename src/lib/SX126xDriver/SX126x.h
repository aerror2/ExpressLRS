#pragma once

#include "targets.h"
#include "SX126X_Regs.h"
#include "SX126X_hal.h"

class SX126XDriver
{
public:
    ///////Callback Function Pointers/////
    static void ICACHE_RAM_ATTR nullCallback(void);

    void (*RXdoneCallback)() = &nullCallback; //function pointer for callback
    void (*TXdoneCallback)() = &nullCallback; //function pointer for callback

    static void (*TXtimeout)(); //function pointer for callback
    static void (*RXtimeout)(); //function pointer for callback

    ///////////Radio Variables////////
    #define TXRXBuffSize 16
    volatile uint8_t TXdataBuffer[TXRXBuffSize];
    volatile uint8_t RXdataBuffer[TXRXBuffSize];

    uint8_t PayloadLength = 8; // Dummy default value which is overwritten during setup.

    static uint8_t _syncWord;

    SX126X_RadioLoRaBandwidths_t currBW = SX126X_LORA_BW_0800;
    SX126X_RadioLoRaSpreadingFactors_t currSF = SX126X_LORA_SF6;
    SX126X_RadioLoRaCodingRates_t currCR = SX126X_LORA_CR_4_7;
    uint32_t currFreq = 433000000;
    SX126X_RadioOperatingModes_t currOpmode = SX126X_MODE_SLEEP;
    bool IQinverted = false;
    bool ImageCalibrated = false;
    // static uint8_t currPWR;
    // static uint8_t maxPWR;

    ///////////////////////////////////

    /////////////Packet Stats//////////
    int8_t LastPacketRSSI = 0;
    int8_t LastPacketSNR = 0;
    int8_t LastSignalRssiPkt = 0;
    volatile uint8_t NonceTX = 0;
    volatile uint8_t NonceRX = 0;
    static uint32_t TotalTime;
    static uint32_t TimeOnAir;
    static uint32_t TXstartMicros;
    static uint32_t TXspiTime;
    static uint32_t HeadRoom;
    static uint32_t TXdoneMicros;
    /////////////////////////////////

    //// Local Variables //// Copy of values for SPI speed optimisation
    static uint8_t CURR_REG_PAYLOAD_LENGTH;
    static uint8_t CURR_REG_DIO_MAPPING_1;
    static uint8_t CURR_REG_FIFO_ADDR_PTR;

    ////////////////Configuration Functions/////////////
    SX126XDriver();
    static SX126XDriver *instance;
    bool Begin();
    void End();
    void SetMode(SX126X_RadioOperatingModes_t OPmode);
    void SetTxIdleMode() { SetMode(SX126X_MODE_FS); }; // set Idle mode used when switching from RX to TX
    void Config(SX126X_RadioLoRaBandwidths_t bw, SX126X_RadioLoRaSpreadingFactors_t sf, SX126X_RadioLoRaCodingRates_t cr, uint32_t freq, uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength);
    void ConfigLoRaModParams(SX126X_RadioLoRaBandwidths_t bw, SX126X_RadioLoRaSpreadingFactors_t sf, SX126X_RadioLoRaCodingRates_t cr,uint8_t LowDatarateOptimize);
    void SetPacketParams(uint8_t PreambleLength, SX126X_RadioLoRaPacketLengthMode_t HeaderType, uint8_t PayloadLength, SX126X_RadioLoRaCrcModes_t crc, SX126X_RadioLoRaIQModes_t InvertIQ);
    void ICACHE_RAM_ATTR SetFrequencyHz(uint32_t freq);
    void ICACHE_RAM_ATTR SetFrequencyReg(uint32_t freq);
    void ICACHE_RAM_ATTR SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr);
    void SetOutputPower(int8_t power);
    void SetOutputPowerMax() { SetOutputPower(13); };
    void SetPaConfig(uint8_t paDutyCycle, uint8_t hpMax, uint8_t deviceSel, uint8_t paLut);
    int32_t ICACHE_RAM_ATTR GetFrequencyError();

    void TXnb();
    void RXnb();

    uint16_t GetIrqStatus();
    void ClearIrqStatus(uint16_t irqMask);

    void GetStatus();

    void SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask);
    
    bool GetFrequencyErrorbool();
    uint8_t GetRxBufferAddr();
    void GetLastPacketStats();
    void CalibrateImage(uint32_t freq);
private:
    static void ICACHE_RAM_ATTR IsrCallback();
    void RXnbISR(); // ISR for non-blocking RX routine
    void TXnbISR(); // ISR for non-blocking TX routine
};
