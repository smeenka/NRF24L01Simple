#ifndef _NRF24L01SIMPLE__h_
#define _NRF24L01SIMPLE__h_

#include <Arduino.h>
#include <nRF24L01.h>

#define RESULT_OK 0
#define RESULT_TX_FULL -1
#define RESULT_RX_FULL -2
#define RESULT_MAX_RETRY -3

typedef struct ReceivePacket // Any packet up to 32 bytes can be sent.
{
    uint8_t pipe;
    uint8_t length;
    uint8_t data[32];
}ReceivePacket_t;

typedef void (*WriteDoneCallback) (void); 

// all interupts enabled, 1 CRC byte, CRC enbled;
//#define  CONFIG_RX_MODE  (_BV(PWR_UP) | _BV(EN_CRC) |_BV(PRIM_RX))
//#define  CONFIG_TX_MODE  (_BV(PWR_UP) | _BV(EN_CRC))

#define  CONFIG_RX_MODE  (_BV(PWR_UP) | 0 |_BV(PRIM_RX))
#define  CONFIG_TX_MODE  (_BV(PWR_UP) | 0)


class NRF24L01Simple {

  public:

    // Constructors
    NRF24L01Simple(uint8_t cePin, uint8_t csnPin); 
    
    enum Bitrate { BITRATE2MBPS = 0x08, BITRATE1MBPS =0x00, BITRATE250KBPS = 0x20};
    enum SendType { ACK, NO_ACK };
    enum Role { PRX = 0x1, PTX = 0x0};
    enum PowerMode {POWER_XLOW = 0b000, POWER_LOW=0b010, POWER_MEDIUM=0b100, POWER_HIGH=0b110};
    enum Checksum {CHECKSUM_OFF, CHECKSUM_1_BYTE, CHECKSUM_2_BYTE};
    
    // Methods for receivers and transmitters.
    // init       = Turns the radio on and puts it into receiving mode.  Returns 0 if it cannot communicate with the radio.
    //              Channel can be 0-125 and sets the exact frequency of the radio between 2400 - 2525 MHz.
    // return true if communication with spi registers is correct, false if communication is not correct
    bool begin( Bitrate bitrate = BITRATE1MBPS);

    void powerDown();

    // power up radio in given power mode wtih given role (PTX or PRX)
    void powerUp( PowerMode mode = POWER_MEDIUM , Checksum cs = CHECKSUM_1_BYTE) ;
 
    // Set the tx address. The address is a pointer to an array of default 5 bytes
    void setTxPipe(uint8_t* address, uint8_t len = 5); 
    // Set the Rx address for the given channel
    // For sending and receiving make tx pipe address equal to rx pipe 0
    // The pipenr is 0 ..5 else nothing will happen
    // For pipenr 0 and 1 a 5 length data array is required, for channel 2 ..5 an one byte array
    // if nrBytes is 0, use the dynamic lenght of the chip. If nrBytes > 0, use static length
    void setRxPipe(uint8_t pipenr, uint8_t* address, uint8_t len = 5, bool enableAutoAck = true, uint8_t nrBytes = 0); 

    void setChannel(uint8_t channel = 100);

    // Check if data can be put in send fifo
    // return RESULT_TX_FULL if no tx fifo is available for send
    // return RESULT_RX_FULL if tx is avaible, but no RX fifo in case of send with ACK
    // return RESULT_OK if a fifo is available for send
    int8_t checkSend(SendType sendType = ACK);

    // Send data to the file. no check  is done in this function, must be done with checkSendFifo before calling this funtion
    void send(void* data, uint8_t length, SendType sendType = ACK);

    void queueAckData(uint8_t pipe, void *data, uint8_t length);

    // this function must be called inside the ISR
    // or by polling with the fastest rate possible
    void checkStatus(void);

    // printDetails  = Prints many of the radio registers.  Requires a serial object in the constructor, e.g. NRFSimple _radio(Serial);
    void printDetails();

    // printChannels = Prints a graph showing received signals across all available channels.  Requires a serial object in the constructor.
    void printChannels();

    uint8_t countRetransmit();

    // switch the radio in listening mode. Wail fail if radio is busy with sending
    bool startRx();

    // check the dataAvailable flag
    bool hasData();

    // return the receive pacet and reset the flag dataAvailable
    ReceivePacket_t* getData(); 

  protected:
    virtual void writeDoneCallback(void) {}
    const static uint8_t MAX_NRF_CHANNEL = 125; // Valid channel range is 2400 - 2525 MHz, in 1 MHz increments.
    const static uint8_t OFF_TO_POWERDOWN_MILLIS = 100;     // Vcc > 1.9V power on reset time.
    const static uint8_t POWERDOWN_TO_STANDBY_MILLIS = 5; // 4500uS to Standby + 130uS to RX or TX mode, so 5ms is enough.
    const static uint8_t STANDBY_TO_RXTX_MODE_MICROS = 130; // 130uS from Standby  RX or TX mode, so 5ms is enough.
    const static uint8_t CE_TRANSMISSION_MICROS = 10;       // Time to initiate data transmission.

    enum SpiTransferType { READ_OPERATION, WRITE_OPERATION };

    uint8_t _cePin, _csnPin;
   
    ReceivePacket_t receivePacket;

    // next flag is set in the interrupt and reset in funtion getReceivePacket
    bool    dataAvailable;
    // next flag is set when start sending data, and reset in interrupt when sending is completed
    bool    busySending;

    bool    initRadio(Bitrate bitrate);
    uint8_t readRegister(uint8_t regName);
    void    readRegister(uint8_t regName, void* data, uint8_t length);
    void    writeRegister(uint8_t regName, uint8_t data);
    void    writeRegister(uint8_t regName, void* data, uint8_t length);
    void    spiTransfer(SpiTransferType transferType, uint8_t regName, void* data, uint8_t length);

    void    printRegister(const char name[], uint8_t regName);
    void    handleRx(void);
};

#endif
