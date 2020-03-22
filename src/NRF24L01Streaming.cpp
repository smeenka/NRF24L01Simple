#include <NRF24L01Streaming.h>
#include <Arduino.h>
#include <SPI.h>

#ifdef  NRF24L01SIMPLE_DEBUG
#define debug(input)   { Serial.print(input);  }
#define debugln(input) { Serial.println(input); }
#else
#define debug(input)   
#define debugln(input) 
#endif


////////////////////
// Public methods //
////////////////////

NRF24L01Streaming::NRF24L01Streaming(uint8_t cePin, uint8_t csnPin) : NRF24L01Simple(cePin, csnPin) {
}


void NRF24L01Streaming::writeDoneCallback(void) {
    uint8_t count = availableTx();

    if (count == 0) {
        busySending = false;
    } else {
        if (count > 31) {
            count = 31;
        } 
        // do the SPI transfer in the most efficient way
        digitalWrite(_csnPin, LOW); // Signal radio to list on the SPI bus.
        //SPI.transfer(W_TX_PAYLOAD_NO_ACK);
        SPI.transfer(W_TX_PAYLOAD);
        SPI.transfer('P');  // send streaming print command
        for (uint8_t i = 0; i < count; i++){
            // send actual streaming data
            uint8_t d = readTx();
            SPI.transfer(d);
        }
        digitalWrite(_csnPin, HIGH); // Stop radio from listening to the SPI bus.

        delayMicroseconds(CE_TRANSMISSION_MICROS);
        // resume sending
        digitalWrite(_cePin, HIGH);
    }    
}   


/////////////////////
// Private methods //
/////////////////////

// streaming implementation
int NRF24L01Streaming::availableForWrite(void)
{
    if (tx_head >= tx_tail) 
        return NRF_TX_BUFFER_SIZE + tx_tail  - tx_head - 1;
    else
        return tx_tail - tx_head - 1;
}
uint8_t NRF24L01Streaming::availableTx(void)
{
    if (tx_head >= tx_tail) 
        return tx_head - tx_tail;
    else
        return NRF_TX_BUFFER_SIZE + tx_head - tx_tail;
}

void NRF24L01Streaming::flush()
{
    // If we have never written a byte, no need to flush. This special
    // case is needed since there is no way to force the TXCIF (transmit
    // complete) bit to 1 during initialization
    tx_tail = 0;
    tx_head = 0;
}

size_t NRF24L01Streaming::write(uint8_t c)
{
    tx_buffer[tx_head] = c;
    tx_head += 1;
    tx_head %= NRF_TX_BUFFER_SIZE;
    return 1;
}

inline uint8_t NRF24L01Streaming::readTx(void) {  // read a character from the TX buffer
    uint8_t result = tx_buffer[tx_tail];
    tx_tail += 1;
    tx_tail %= NRF_TX_BUFFER_SIZE; 
    return result;
}
