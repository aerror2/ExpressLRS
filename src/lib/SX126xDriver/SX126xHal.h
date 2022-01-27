
#include "SX126X_Regs.h"
#include "SX126X.h"

enum SX126X_BusyState_
{
    SX126X_NOT_BUSY = true,
    SX126X_BUSY = false,
};

class SX126XHal
{
public:
    static SX126XHal *instance;

    SX126XHal();

    void init();
    void end();
    void reset();

    void ICACHE_RAM_ATTR WriteCommand(SX126X_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR WriteCommand(SX126X_RadioCommands_t command, uint8_t val);
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR WriteRegister(uint16_t address, uint8_t value);

    void ICACHE_RAM_ATTR ReadCommand(SX126X_RadioCommands_t opcode, uint8_t *buffer, uint8_t size);
    void ICACHE_RAM_ATTR ReadRegister(uint16_t address, uint8_t *buffer, uint8_t size);
    uint8_t ICACHE_RAM_ATTR ReadRegister(uint16_t address);

    void ICACHE_RAM_ATTR WriteBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size); // Writes and Reads to FIFO
    void ICACHE_RAM_ATTR ReadBuffer(uint8_t offset, volatile uint8_t *buffer, uint8_t size);

    bool ICACHE_RAM_ATTR WaitOnBusy();
    
    void ICACHE_RAM_ATTR TXenable();
    void ICACHE_RAM_ATTR RXenable();
    void ICACHE_RAM_ATTR TXRXdisable();

    static ICACHE_RAM_ATTR void dioISR();
    void (*IsrCallback)(); //function pointer for callback

#if defined(GPIO_PIN_BUSY) && (GPIO_PIN_BUSY != UNDEF_PIN)
    void BusyDelay(uint32_t duration) const { (void)duration; };
#else
    uint32_t BusyDelayStart;
    uint32_t BusyDelayDuration;
    void BusyDelay(uint32_t duration)
    {
        BusyDelayStart = micros();
        BusyDelayDuration = duration;
    }
#endif
};