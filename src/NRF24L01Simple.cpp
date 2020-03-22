#include <NRF24L01Simple.h>
#include <Arduino.h>
#include <SPI.h> 

#ifdef  NRF24L01SIMPLE_DEBUG
#define debug(input)   { Serial.print(input);    }
#define debughex(input){ Serial.print(input,HEX);}
#define debugln(input) { Serial.println(input);  }
#else
#define debug(input)   
#define debughex(input)
#define debugln(input) 
#endif


////////////////////
// Public methods //
////////////////////

NRF24L01Simple::NRF24L01Simple(uint8_t cePin, uint8_t csnPin){
    _cePin = cePin;
    _csnPin= csnPin;
    // Default states for the radio pins.  When CSN is LOW the radio listens to SPI communication,
    // so we operate most of the time with CSN HIGH.
    pinMode(      cePin, OUTPUT);
    pinMode(     csnPin, OUTPUT);
    digitalWrite(csnPin, HIGH);
    digitalWrite( cePin, LOW);
}


bool NRF24L01Simple::begin(Bitrate bitrate)
{
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV4); // decided to slow down from DIV2     
    busySending = false;
    dataAvailable = false;

    return  initRadio(bitrate);
}

bool NRF24L01Simple::initRadio(Bitrate bitrate)
{
    powerUp();

    // Transmission speed, retry times, and output power setup.
    // For 2 Mbps or 1 Mbps operation, a 500 uS retry time is necessary to support the max ACK packet size.
    // For 250 Kbps operation, a 1500 uS retry time is necessary.
    if (bitrate == BITRATE2MBPS)
    {
        writeRegister(RF_SETUP,   B00001010);   // 2 Mbps, -12 dBm output power
        writeRegister(SETUP_RETR, 0x1F); // 1 =  500 uS between retries, 1111 = 15 retries
    }
    else if (bitrate == BITRATE1MBPS)
    {
        writeRegister(RF_SETUP, B00000010);   // 1 Mbps, -12 dBm output power
        writeRegister(SETUP_RETR, 0x2F); // 2 =  750 uS between retries, 1111 = 15 retries
    }
    else
    {
        writeRegister(RF_SETUP, B00100010);   // 250 Kbps, -12 dBm output power
        writeRegister(SETUP_RETR, 0x5F); // 5 = 1500 uS between retries, 1111 = 15 retries
    }
    // autoacknolidge is disabled here, enabled while setting a RX address
    writeRegister(EN_AA, 0);
    writeRegister(EN_RXADDR, 0);


    // Enable dynamically sized packets on the 2 RX pipes we use, 0 and 1.
    // RX pipe address 1 is used to for normal packets from radios that send us data.
    // RX pipe address 0 is used to for auto-acknowledgment packets from radios we transmit to.

    // Enable dynamically sized payloads, ACK payloads, and TX support with or without an ACK request.
    //writeRegister(FEATURE,  _BV(EN_ACK_PAY) | _BV(EN_DYN_ACK));

    // for the RFNANO bit EN_DYN_ACK seems to break ack packets.
    writeRegister(FEATURE,  _BV(EN_ACK_PAY) );

    // Ensure RX and TX buffers are empty.  Each buffer can hold 3 packets.
    spiTransfer(WRITE_OPERATION, FLUSH_RX, NULL, 0);
    spiTransfer(WRITE_OPERATION, FLUSH_TX, NULL, 0);

    // Clear any interrupts.
    writeRegister(STATUS_NRF, _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));

    // check if the SPI registers can be read
    uint8_t reg = readRegister( CONFIG);
    return reg == (CONFIG_RX_MODE | _BV(EN_CRC) );
}

// Interrupt handlers - Not intended to be called externally
/* ======================= interrupt stuff --------------------- */

void NRF24L01Simple::setChannel(uint8_t channel){
    // Valid channel range is 2400 - 2525 MHz, in 1 MHz increments.
    if (channel > MAX_NRF_CHANNEL) { 
        channel = MAX_NRF_CHANNEL; 
    }
    writeRegister(RF_CH, channel);
}

void NRF24L01Simple::powerDown()
{
    digitalWrite(_cePin, LOW);
    delayMicroseconds(CE_TRANSMISSION_MICROS);
    // Turn off the radio.
    writeRegister(CONFIG, readRegister(CONFIG) & ~_BV(PWR_UP));
}
// power up radio in RX mode
void NRF24L01Simple::powerUp(PowerMode mode, Checksum cs)
{
    digitalWrite(_cePin, LOW);
    delayMicroseconds(CE_TRANSMISSION_MICROS);
    
    uint8_t config = CONFIG_RX_MODE;
    switch (cs) {
    case CHECKSUM_OFF:
      break;
    case CHECKSUM_1_BYTE:
        config |= _BV(EN_CRC);
      break;
    case CHECKSUM_2_BYTE:
        config |= _BV(EN_CRC);
        config |= _BV(CRCO);
      break;
    default:
      break;        
    }
    // Turn on the radio into standby into the given mode
    writeRegister( CONFIG, config);

    uint8_t reg = readRegister( RF_SETUP);
    reg &=  ~0b110; // clear lowest bits
    reg |= mode;
    writeRegister( RF_SETUP, reg);
    delay(POWERDOWN_TO_STANDBY_MILLIS);
}

// Set the tx address. The address is a pointer to an array of 5 bytes
void NRF24L01Simple::setTxPipe(uint8_t* address, uint8_t len ){
    writeRegister(TX_ADDR, address, len);
    writeRegister(SETUP_AW, len -2);
} 
// Set the Rx address for the given pipenr
// The pipenr is 0 ..5 else nothing will happen
// For pipenr 0 and 1 a 5 length data array is required, for pipenr 2 ..5 an one byte array
void NRF24L01Simple::setRxPipe(uint8_t pipenr, uint8_t* address, uint8_t len , bool enableAutoAck, uint8_t nrBytes){

    switch (pipenr) {
    case 0:
      writeRegister(RX_ADDR_P0, address, len);
      writeRegister(RX_PW_P0,   nrBytes);
      break;
    case 1:
      writeRegister(RX_ADDR_P1, address, len);
      writeRegister(RX_PW_P1,   nrBytes);
      break;
    case 2:
      writeRegister(RX_ADDR_P2, address, 1);
      writeRegister(RX_PW_P2,   nrBytes);
      break;
    case 3:
      writeRegister(RX_ADDR_P3, address, 1);
      writeRegister(RX_PW_P3,   nrBytes);
      break;
    case 4:
      writeRegister(RX_ADDR_P4, address, 1);
      writeRegister(RX_PW_P4,   nrBytes);
      break;
    case 5:
      writeRegister(RX_ADDR_P5, address, 1);
      writeRegister(RX_PW_P5,   nrBytes);
      break;
    default:
      break;        
    }
    // set the length of the address
    writeRegister(SETUP_AW, len -2);

    // handle auto acknolidge
    uint8_t reg = readRegister(EN_AA);
    if (enableAutoAck)
        writeRegister(EN_AA, reg |  _BV(pipenr) );
    else
        writeRegister(EN_AA, reg & ~_BV(pipenr) );

    // enable this pipenr
    reg = readRegister(EN_RXADDR);
    writeRegister(EN_RXADDR, reg |_BV(pipenr) );

    // enable dynamic payload for this pipenr if nrBytes is zero
    if (nrBytes == 0 ){
       reg = readRegister(FEATURE);
        writeRegister(FEATURE, reg |_BV(EN_DPL));

        reg = readRegister(DYNPD);
        writeRegister(DYNPD, reg | _BV(pipenr) );
    }
} 



    // Check if data can be put in senf fifo
    // return RESULT_OK if a fifo is available for send
    // return RESULT_TX_FULL if nof tx fifo is available for send
    // return RESULT_RX_FULL if tx is avaible, but no RX fifo in case of send with ACK
int8_t NRF24L01Simple::checkSend(SendType sendType){
    uint8_t fifoReg = readRegister(FIFO_STATUS);
    uint8_t txBufferIsFull = fifoReg & _BV(FIFO_FULL);
    if (txBufferIsFull)
    {  
        // Put radio into active mode so it will start sending even after a retry
        digitalWrite(_cePin, HIGH);
        return RESULT_TX_FULL;
    }
    // If RX buffer is full and we require an ACK, clear it so we can receive the ACK response.
    uint8_t rxBufferIsFull = fifoReg & _BV(RX_FULL);
    if (sendType == ACK && rxBufferIsFull)
    {
        return RESULT_RX_FULL;
    }
    return RESULT_OK;
}

    // Send data. 
void NRF24L01Simple::send(void *data, uint8_t length, SendType sendType){
    busySending = true;
    digitalWrite(_cePin, LOW);
    uint8_t configReg = readRegister(CONFIG);
    // reset bit 0
    writeRegister( CONFIG, configReg & ~1);

    // Add data to the TX buffer, with or without an ACK request.
    if (sendType == NO_ACK) { spiTransfer(WRITE_OPERATION, W_TX_PAYLOAD_NO_ACK, data, length); }
    else                    { spiTransfer(WRITE_OPERATION, W_TX_PAYLOAD       , data, length); }

    // Put radio into active mode so it will start sending
    digitalWrite(_cePin, HIGH);
}

void NRF24L01Simple::queueAckData(uint8_t pipe, void *data, uint8_t length)
{
   
    // Add the packet to the TX buffer for pipe 1, the pipe used to receive packets from radios that
    // send us data.  When we receive the next transmission from a radio, we'll provide this ACK data in the
    // auto-acknowledgment packet that goes back.
    spiTransfer(WRITE_OPERATION, (W_ACK_PAYLOAD | pipe), data, length);
}

// this function must be called inside the ISR
void NRF24L01Simple::checkStatus(void){
    uint8_t statusReg = readRegister(STATUS_NRF);
    
    if (statusReg & _BV(TX_DS)){
        // ok we are done with sending, put radio in standby
        digitalWrite(_cePin, LOW);
        writeRegister(STATUS_NRF,  _BV(TX_DS ) );
        busySending = false;
        debugln("tI");    
        writeDoneCallback();
    }
    if (statusReg & _BV(MAX_RT)){
        // ok radio is ready with send task, with error
        busySending = false;
        digitalWrite(_cePin, LOW);
        writeRegister(STATUS_NRF,  _BV(MAX_RT) );
        debugln("eI");
    }
    if (statusReg & _BV(RX_DR)){
        // radio did receive a packet of data
        digitalWrite(_cePin, LOW);
        writeRegister(STATUS_NRF,  _BV(RX_DR) );
        handleRx();
    }
}

// only to be called form checkISR function
void NRF24L01Simple::handleRx(void){
    //     110 = Not Used
    //     111 = RX FIFO Empty
    uint8_t pipenr = (readRegister(STATUS_NRF) & B1110) >> 1;
    if (pipenr > 5)  {
        return;
    }
    receivePacket.pipe = pipenr;

    uint8_t dataLength;
    spiTransfer(READ_OPERATION, R_RX_PL_WID, &dataLength, 1);

    // Verify the data length is valid (0 - 32 bytes).
    if (dataLength > 32)
    {
        spiTransfer(WRITE_OPERATION, FLUSH_RX, NULL, 0); // Clear invalid data in the RX buffer.
        return ;
    }
    receivePacket.length = dataLength;

    spiTransfer(READ_OPERATION, R_RX_PAYLOAD, receivePacket.data, dataLength);

    dataAvailable = true;
}


// check the dataAVailable flag
bool NRF24L01Simple::hasData(){
    return dataAvailable;
}



// return the receive pacet and reset the flag dataAvailable
ReceivePacket_t* NRF24L01Simple::getData(){
    dataAvailable = false;
    return &receivePacket;
}

bool NRF24L01Simple::startRx()
{
    if (busySending) {
        return false;
    }
    digitalWrite(_cePin, LOW);
    uint8_t configReg = readRegister(CONFIG);
    // set bit 0
    writeRegister( CONFIG, configReg | 1);

    // Start listening for packets.
    digitalWrite(_cePin, HIGH);
    return true;
}

void NRF24L01Simple::printDetails()
{
    printRegister("CONFIG", readRegister(CONFIG));
    printRegister("EN_AA", readRegister(EN_AA));
    printRegister("EN_RXADDR", readRegister(EN_RXADDR));
    printRegister("SETUP_AW", readRegister(SETUP_AW));
    printRegister("SETUP_RETR", readRegister(SETUP_RETR));
    printRegister("RF_CH", readRegister(RF_CH));
    printRegister("RF_SETUP", readRegister(RF_SETUP));
    printRegister("STATUS", readRegister(STATUS_NRF));
    printRegister("OBSERVE_TX", readRegister(OBSERVE_TX));
    printRegister("RX_PW_P0", readRegister(RX_PW_P0));
    printRegister("RX_PW_P1", readRegister(RX_PW_P1));
    printRegister("FIFO_STATUS", readRegister(FIFO_STATUS));
    printRegister("DYNPD", readRegister(DYNPD));
    printRegister("FEATURE", readRegister(FEATURE));
    
    uint8_t len = 5;

    uint8_t data[len];
    
    Serial.print("TX_ADDR ");
    readRegister(TX_ADDR, &data, len);
    for (uint8_t i = 0; i < len; i++) {
        Serial.print( data[i],HEX ); 
        Serial.print(","); 
     }
    Serial.println("");
    Serial.print("RX_ADDR_P0 ");
    readRegister(RX_ADDR_P0, &data, len);
    for (uint8_t i = 0; i < len; i++) { 
        Serial.print( data[i] , HEX); 
        Serial.print(","); 
    }
    Serial.println("");
    Serial.print("RX_ADDR_P1 ");
    readRegister(RX_ADDR_P1, &data, len);
    for (uint8_t i = 0; i < len; i++) { 
        Serial.print( data[i], HEX ) ; 
        Serial.print(","); 
    }
    Serial.println("");
}

void NRF24L01Simple::printChannels()
{
    uint8_t signalStrength[MAX_NRF_CHANNEL + 1] = { 0 };  // Holds carrier detect counts for each channel.
    uint8_t originalChannelReg = readRegister(RF_CH);
    
    // Put radio into Standby-I mode.
    while (!startRx());
    digitalWrite(_cePin, LOW);
    
    // Loop through each channel.
    for (uint8_t channelNumber = 0; channelNumber <= MAX_NRF_CHANNEL; channelNumber++)
    {
        // Set the channel.
        writeRegister(RF_CH, channelNumber);

        // Take a bunch of measurements.
        for (uint8_t measurementCount = 0; measurementCount < 200; measurementCount++)
        {
            // Put the radio into RX mode and wait a little time for a signal to be received.
            digitalWrite(_cePin, HIGH);
            delayMicroseconds(400);
            digitalWrite(_cePin, LOW);

            uint8_t signalWasReceived = readRegister(CD);
            if (signalWasReceived)
            {
                signalStrength[channelNumber]++;
            }
        }

        // Build the message about the channel, e.g. 'Channel 125 XXXXXXXXX'
        String channelMsg = "Channel ";
        
        if      (channelNumber < 10 ) { channelMsg += "  "; } // Right-align
        else if (channelNumber < 100) { channelMsg += " ";  } // the channel
        channelMsg += channelNumber;                          // number.

        channelMsg += " ";
        uint8_t strength = signalStrength[channelNumber];

        while (strength--)
        {
            channelMsg += "X";
        }

        // Print the message.
        Serial.println(channelMsg);
    }
    digitalWrite(_cePin, LOW);
    writeRegister(RF_CH, originalChannelReg); // Set the radio back to the original channel.
}

/////////////////////
// Private methods //
/////////////////////





/*
SPI related functions
*/
uint8_t NRF24L01Simple::readRegister(uint8_t regName)
{
    uint8_t data;
    readRegister(regName, &data, 1);
    return data;
}

void NRF24L01Simple::readRegister(uint8_t regName, void *data, uint8_t length)
{
    spiTransfer(READ_OPERATION, (R_REGISTER | (REGISTER_MASK & regName)), data, length);
}

void NRF24L01Simple::writeRegister(uint8_t regName, uint8_t data)
{
    writeRegister(regName, &data, 1);
}
uint8_t NRF24L01Simple::countRetransmit()
{
    return (readRegister(OBSERVE_TX) & 0xF);
}

void NRF24L01Simple::writeRegister(uint8_t regName, void *data, uint8_t length)
{
    spiTransfer(WRITE_OPERATION, (W_REGISTER | (REGISTER_MASK & regName)), data, length);
}

void NRF24L01Simple::spiTransfer(SpiTransferType transferType, uint8_t regName, void *data, uint8_t length)
{
    uint8_t* intData = reinterpret_cast<uint8_t*>(data);

    noInterrupts(); // Prevent an interrupt from interferring with the communication.

    digitalWrite(_csnPin, LOW); // Signal radio to list on the SPI bus.
    SPI.transfer(regName);

    for (uint8_t i = 0; i < length; ++i) {
        uint8_t newData = SPI.transfer(intData[i]);
        if (transferType == READ_OPERATION) { 
            intData[i] = newData; 
        }
    }
    digitalWrite(_csnPin, HIGH); // Stop radio from listening to the SPI bus.
    interrupts();
}

void NRF24L01Simple::printRegister(const char name[], uint8_t reg)
{
    String msg = name;
    msg += " ";

    uint8_t i = 8;
    do
    {
        msg += bitRead(reg, --i);
    }
    while (i);

    Serial.println(msg);
}

