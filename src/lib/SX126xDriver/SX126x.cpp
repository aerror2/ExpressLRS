#include "sx126x/sx126x.h"
#include "SX126X_Regs.h"
#include "SX126X_hal.h"
#include "SX126X.h"
#include "logging.h"

SX126XHal hal;
SX126XDriver *SX126XDriver::instance = NULL;

//DEBUG_SX126X_OTA_TIMING

/* Steps for startup

1. If not in STDBY_RC mode, then go to this mode by sending the command:
SetStandby(STDBY_RC)

2. Define the LoRa® packet type by sending the command:
SetPacketType(PACKET_TYPE_LORA)

3. Define the RF frequency by sending the command:
SetRfFrequency(rfFrequency)
The LSB of rfFrequency is equal to the PLL step i.e. 52e6/2^18 Hz. SetRfFrequency() defines the Tx frequency.

4. Indicate the addresses where the packet handler will read (txBaseAddress in Tx) or write (rxBaseAddress in Rx) the first
byte of the data payload by sending the command:
SetBufferBaseAddress(txBaseAddress, rxBaseAddress)
Note:
txBaseAddress and rxBaseAddress are offset relative to the beginning of the data memory map.

5. Define the modulation parameter signal BW SF CR
*/

#if defined(DEBUG_SX126X_OTA_TIMING)
static uint32_t beginTX;
static uint32_t endTX;
#endif

void ICACHE_RAM_ATTR SX126XDriver::nullCallback(void) {}

SX126XDriver::SX126XDriver()
{
    instance = this;
}

void SX126XDriver::End()
{
    SetMode(SX126X_MODE_SLEEP);
    hal.end();
    TXdoneCallback = &nullCallback; // remove callbacks
    RXdoneCallback = &nullCallback;
}

bool SX126XDriver::Begin()
{
    hal.init();
    hal.IsrCallback = &SX126XDriver::IsrCallback;

    hal.reset();
    DBGLN("SX126X Begin");
    delay(100);
    uint16_t firmwareRev = (((hal.ReadRegister(REG_LR_FIRMWARE_VERSION_MSB)) << 8) | (hal.ReadRegister(REG_LR_FIRMWARE_VERSION_MSB + 1)));
    DBGLN("Read Vers: %d", firmwareRev);
    if ((firmwareRev == 0) || (firmwareRev == 65535))
    {
        // SPI communication failed, just return without configuration
        return false;
    }

    SetMode(SX126X_MODE_STDBY_RC);                                                                                                //Put in STDBY_RC mode
    hal.WriteCommand(RADIO_SET_PACKETTYPE, PACKET_TYPE_LORA);                                                       //Set packet type to LoRa
    ConfigLoRaModParams(currBW, currSF, currCR, 1);                                                                                  //Configure Modulation Params

    //hal.WriteCommand(RADIO_SET_FS, 0x01);                                                                              //Enable auto fs  ,,126x seems no auto fs


   // hal.WriteRegister(0x0891, (hal.ReadRegister(0x0891) | 0xC0));                                                                 //default is low power mode, switch to high sensitivity instead

    SetPacketParams(12, SX126X_LORA_PACKET_IMPLICIT, 8, SX126X_LORA_CRC_OFF, SX126X_LORA_IQ_NORMAL);                              //default params
    SetFrequencyReg(currFreq);                                                                                                    //Set Freq
    SetFIFOaddr(0x00, 0x00);                                                                                                      //Config FIFO addr
    SetDioIrqParams(SX126X_IRQ_RADIO_ALL, SX126X_IRQ_TX_DONE | SX126X_IRQ_RX_DONE, SX126X_IRQ_RADIO_NONE, SX126X_IRQ_RADIO_NONE); //set IRQ to both RXdone/TXdone on DIO1
#if defined(USE_SX126X_DCDC)
    hal.WriteCommand(SX126X_RADIO_SET_REGULATORMODE, SX126X_USE_DCDC);     // Enable DCDC converter instead of LDO
#endif
    return true;
}

void SX126XDriver::Config(SX126X_RadioLoRaBandwidths_t bw, SX126X_RadioLoRaSpreadingFactors_t sf, SX126X_RadioLoRaCodingRates_t cr, uint32_t freq, uint8_t PreambleLength, bool InvertIQ, uint8_t PayloadLength)
{
    PayloadLength = PayloadLength;
    IQinverted = InvertIQ;
    SetMode(SX126X_MODE_STDBY_XOSC);
    ConfigLoRaModParams(bw, sf, cr);
    SetPacketParams(PreambleLength, SX126X_LORA_PACKET_IMPLICIT, PayloadLength, SX126X_LORA_CRC_OFF, (SX126X_RadioLoRaIQModes_t)((uint8_t)!IQinverted << 6)); // TODO don't make static etc. LORA_IQ_STD = 0x40, LORA_IQ_INVERTED = 0x00
    SetFrequencyReg(freq);
}

void SX126XDriver::SetOutputPower(int8_t power)
{
    // if (power < -18) power = -18;
    // else if (13 < power) power = 13;
    // uint8_t buf[2] = {(uint8_t)(power + 18), (uint8_t)SX126X_RADIO_RAMP_04_US};
    // hal.WriteCommand(SX126X_RADIO_SET_TXPARAMS, buf, sizeof(buf));
    // DBGLN("SetPower: %d", buf[0]);
    // return;
    SX126X_RadioRampTimes_t rampTime =  SX126X_RADIO_RAMP_40_US ;

    uint8_t buf[2];

    // if (SX126xGetPaSelect(0) == SX1261)
    // {
    //     if (power == 15)
    //     {
    //         SX126xSetPaConfig(0x06, 0x00, 0x01, 0x01);
    //     }
    //     else
    //     {
    //         SX126xSetPaConfig(0x04, 0x00, 0x01, 0x01);
    //     }
    //     if (power >= 14)
    //     {
    //         power = 14;
    //     }
    //     else if (power < -17)
    //     {
    //         power = -17;
    //     }
    //     SX126xWriteRegister(REG_OCP, 0x18); // current max is 80 mA for the whole device
    // }
    // else // sx1262
    {
        // WORKAROUND - Better Resistance of the SX1262 Tx to Antenna Mismatch, see DS_SX1261-2_V1.2 datasheet chapter 15.2
        // RegTxClampConfig = @address 0x08D8
        SX126xWriteRegister(0x08D8, SX126xReadRegister(0x08D8) | (0x0F << 1));
        // WORKAROUND END

        SX126xSetPaConfig(0x04, 0x07, 0x00, 0x01);
        if (power > 22)
        {
            power = 22;
        }
        else if (power < -9)
        {
            power = -9;
        }
        SX126xWriteRegister(REG_OCP, 0x38); // current max 160mA for the whole device
    }
    buf[0] = power;
    buf[1] = (uint8_t)rampTime;
    SX126xWriteCommand(SX126X_RADIO_SET_TXPARAMS, buf, 2);

}

void SX126XDriver::SetPacketParams(uint8_t PreambleLength, SX126X_RadioLoRaPacketLengthMode_t HeaderType, uint8_t PayloadLength, SX126X_RadioLoRaCrcModes_t crc, SX126X_RadioLoRaIQModes_t InvertIQ)
{
    uint8_t buf[7];

    buf[0] = PreambleLength;
    buf[1] = HeaderType;
    buf[2] = PayloadLength;
    buf[3] = crc;
    buf[4] = InvertIQ;
    buf[5] = 0x00;
    buf[6] = 0x00;

    hal.WriteCommand(SX126X_RADIO_SET_PACKETPARAMS, buf, sizeof(buf));
}

void SX126XDriver::SetMode(SX126X_RadioOperatingModes_t OPmode)
{

    if (OPmode == currOpmode)
    {
       return;
    }

    WORD_ALIGNED_ATTR uint8_t buf[3];
    uint32_t switchDelay = 0;

    switch (OPmode)
    {

    case SX126X_MODE_SLEEP:
        hal.WriteCommand(RADIO_SET_SLEEP, 0x01);
        break;

    case SX126X_MODE_CALIBRATION:
        break;

    case SX126X_MODE_STDBY_RC:
        hal.WriteCommand(RADIO_SET_STANDBY, SX126X_STDBY_RC);
        switchDelay = 1500;
        break;

    case SX126X_MODE_STDBY_XOSC:
        hal.WriteCommand(RADIO_SET_STANDBY, SX126X_STDBY_XOSC);
        switchDelay = 50;
        break;

    case SX126X_MODE_FS:
        hal.WriteCommand(RADIO_SET_FS, 0x00);
        switchDelay = 70;
        break;

    case SX126X_MODE_RX:
        buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet, set to FF for cont RX
        buf[1] = 0xFF;
        buf[2] = 0xFF;
        hal.WriteCommand(RADIO_SET_RX, buf, sizeof(buf));
        switchDelay = 100;
        break;

    case SX126X_MODE_TX:
        //uses timeout Time-out duration = periodBase * periodBaseCount
        buf[0] = 0x00; // periodBase = 1ms, page 71 datasheet
        buf[1] = 0xFF; // no timeout set for now
        buf[2] = 0xFF; // TODO dynamic timeout based on expected onairtime
        hal.WriteCommand(RADIO_SET_TX, buf, sizeof(buf));
        switchDelay = 100;
        break;

    case SX126X_MODE_CAD:

        hal.WriteCommand(RADIO_SET_CAD, 0, 0);
        break;

    default:
        break;
    }
    hal.BusyDelay(switchDelay);

    currOpmode = OPmode;
}

void SX126XDriver::ConfigLoRaModParams(SX126X_RadioLoRaBandwidths_t bw, SX126X_RadioLoRaSpreadingFactors_t sf, SX126X_RadioLoRaCodingRates_t cr,uint8_t LowDatarateOptimize)
{
    // Care must therefore be taken to ensure that modulation parameters are set using the command
    // SetModulationParam() only after defining the packet type SetPacketType() to be used


     


    WORD_ALIGNED_ATTR uint8_t rfparams[3] = {0};

    rfparams[0] = (uint8_t)sf;
    rfparams[1] = (uint8_t)bw;
    rfparams[2] = (uint8_t)cr;
    rfparams[3] = (uint8_t)LowDatarateOptimize;

    hal.WriteCommand(RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams));


}




void ICACHE_RAM_ATTR SX126XDriver::CalibrateImage(uint32_t freq)
{
    uint8_t calFreq[2];

    if (freq > 900000000)
    {
        calFreq[0] = 0xE1;
        calFreq[1] = 0xE9;
    }
    else if (freq > 850000000)
    {
        calFreq[0] = 0xD7;
        calFreq[1] = 0xDB;
    }
    else if (freq > 770000000)
    {
        calFreq[0] = 0xC1;
        calFreq[1] = 0xC5;
    }
    else if (freq > 460000000)
    {
        calFreq[0] = 0x75;
        calFreq[1] = 0x81;
    }
    else if (freq > 425000000)
    {
        calFreq[0] = 0x6B;
        calFreq[1] = 0x6F;
    }
    hal.WriteCommand(SX126X_RADIO_CALIBRATEIMAGE, calFreq, 2);
}


void ICACHE_RAM_ATTR SX126XDriver::SetFrequencyHz(uint32_t frequency)
{
    // WORD_ALIGNED_ATTR uint8_t buf[3] = {0};

    // uint32_t freq = (uint32_t)((double)Reqfreq / (double)FREQ_STEP);
    // buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    // buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    // buf[2] = (uint8_t)(freq & 0xFF);

    // hal.WriteCommand(SX126X_RADIO_SET_RFFREQUENCY, buf, sizeof(buf));
    // currFreq = Reqfreq;

    WORD_ALIGNED_ATTR uint8_t buf[4];
    uint32_t freq = 0;

    if (ImageCalibrated == false)
    {
        CalibrateImage(frequency);
        ImageCalibrated = true;
    }

    freq = (uint32_t)((double)frequency / (double)FREQ_STEP);
    buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);
    hal.WriteCommand(SX126X_RADIO_SET_RFFREQUENCY, buf, 4);
    currFreq = frequency;
}

void ICACHE_RAM_ATTR SX126XDriver::SetFrequencyReg(uint32_t freq)
{
    WORD_ALIGNED_ATTR uint8_t buf[4] = {0};

    if (ImageCalibrated == false)
    {
        CalibrateImage(freq);
        ImageCalibrated = true;
    }


   buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);
    hal.WriteCommand(SX126X_RADIO_SET_RFFREQUENCY, buf, sizeof(buf));
    currFreq = freq;
}

int32_t ICACHE_RAM_ATTR SX126XDriver::GetFrequencyError()
{
    WORD_ALIGNED_ATTR uint8_t efeRaw[3] = {0};
    uint32_t efe = 0;
    double efeHz = 0.0;

    efeRaw[0] = hal.ReadRegister(SX126X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
    efeRaw[1] = hal.ReadRegister(SX126X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1);
    efeRaw[2] = hal.ReadRegister(SX126X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2);
    efe = (efeRaw[0] << 16) | (efeRaw[1] << 8) | efeRaw[2];

    efe &= SX126X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

    //efeHz = 1.55 * (double)complement2(efe, 20) / (1600.0 / (double)GetLoRaBandwidth() * 1000.0);
    return efeHz;
}

void SX126XDriver::SetFIFOaddr(uint8_t txBaseAddr, uint8_t rxBaseAddr)
{
    uint8_t buf[2];

    buf[0] = txBaseAddr;
    buf[1] = rxBaseAddr;
    hal.WriteCommand(SX126X_RADIO_SET_BUFFERBASEADDRESS, buf, sizeof(buf));
}

void SX126XDriver::SetDioIrqParams(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irqMask & 0x00FF);
    buf[2] = (uint8_t)((dio1Mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1Mask & 0x00FF);
    buf[4] = (uint8_t)((dio2Mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2Mask & 0x00FF);
    buf[6] = (uint8_t)((dio3Mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3Mask & 0x00FF);

    hal.WriteCommand(SX126X_RADIO_CFG_DIOIRQ, buf, sizeof(buf));
}

uint16_t ICACHE_RAM_ATTR SX126XDriver::GetIrqStatus()
{
    uint8_t status[2];

    hal.ReadCommand(SX126X_RADIO_GET_IRQSTATUS, status, 2);
    return status[0] << 8 | status[1];
}

void ICACHE_RAM_ATTR SX126XDriver::ClearIrqStatus(uint16_t irqMask)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irqMask >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irqMask & 0x00FF);

    hal.WriteCommand(SX126X_RADIO_CLR_IRQSTATUS, buf, sizeof(buf));
}

void ICACHE_RAM_ATTR SX126XDriver::TXnbISR()
{
    //TODO
    currOpmode = SX126X_MODE_FS; // radio goes to FS after TX
// #ifdef DEBUG_SX126X_OTA_TIMING
//     endTX = micros();
//     DBGLN("TOA: %d", endTX - beginTX);
// #endif
    TXdoneCallback();
}

uint8_t FIFOaddr = 0;

void ICACHE_RAM_ATTR SX126XDriver::TXnb()
{
    if (currOpmode == SX126X_MODE_TX) //catch TX timeout
    {
        //DBGLN("Timeout!");
        SetMode(SX126X_MODE_FS);
        TXnbISR();
        return;
    }
    hal.TXenable();                      // do first to allow PA stablise
    hal.WriteBuffer(0x00, TXdataBuffer, PayloadLength); //todo fix offset to equal fifo addr
    instance->SetMode(SX126X_MODE_TX);
#ifdef DEBUG_SX126X_OTA_TIMING
    beginTX = micros();
#endif
}

void ICACHE_RAM_ATTR SX126XDriver::RXnbISR()
{
    // In continuous receive mode, the device stays in Rx mode
    //currOpmode = SX126X_MODE_FS;
    uint8_t FIFOaddr = GetRxBufferAddr();
    hal.ReadBuffer(FIFOaddr, RXdataBuffer, PayloadLength);
    GetLastPacketStats();
    RXdoneCallback();
}

void ICACHE_RAM_ATTR SX126XDriver::RXnb()
{
    hal.RXenable();
    SetMode(SX126X_MODE_RX);
}

uint8_t ICACHE_RAM_ATTR SX126XDriver::GetRxBufferAddr()
{
    WORD_ALIGNED_ATTR uint8_t status[2] = {0};
    hal.ReadCommand(SX126X_RADIO_GET_RXBUFFERSTATUS, status, 2);
    return status[1];
}

void ICACHE_RAM_ATTR SX126XDriver::GetStatus()
{
    uint8_t status = 0;
    hal.ReadCommand(SX126X_RADIO_GET_STATUS, (uint8_t *)&status, 1);
    DBGLN("Status: %x, %x, %x", (0b11100000 & status) >> 5, (0b00011100 & status) >> 2, 0b00000001 & status);
}

bool ICACHE_RAM_ATTR SX126XDriver::GetFrequencyErrorbool()
{
    //TODO:
    
    // uint8_t regEFI[3];

    // hal.ReadRegister(SX126X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB, regEFI, sizeof(regEFI));

    // DBGLN("%d %d %d", regEFI[0], regEFI[1], regEFI[2]);

    //bool result = (val & 0b00001000) >> 3;
    //return result; // returns true if pos freq error, neg if false
    return 0;
}

void ICACHE_RAM_ATTR SX126XDriver::GetLastPacketStats()
{
    uint8_t status[2];

    hal.ReadCommand(SX126X_RADIO_GET_PACKETSTATUS, status, 2);
    LastPacketRSSI = -(int8_t)(status[0] / 2);
    LastPacketSNR = (int8_t)status[1] / 4;
    // https://www.mouser.com/datasheet/2/761/DS_SX126X-1_V2.2-1511144.pdf
    // need to subtract SNR from RSSI when SNR <= 0;
    int8_t negOffset = (LastPacketSNR < 0) ? LastPacketSNR : 0; 
    LastPacketRSSI += negOffset;
}

void ICACHE_RAM_ATTR SX126XDriver::IsrCallback()
{
    uint16_t irqStatus = instance->GetIrqStatus();
    instance->ClearIrqStatus(SX126X_IRQ_RADIO_ALL);
    if ((irqStatus & SX126X_IRQ_TX_DONE))
        instance->TXnbISR();
    else if ((irqStatus & SX126X_IRQ_RX_DONE))
        instance->RXnbISR();
}
