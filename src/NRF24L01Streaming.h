#ifndef _NRFStreaming_h_
#define _NRFStreaming_h_

#include <Arduino.h>
#include <NRF24L01Simple.h>

#define RESULT_OK                      0
#define RESULT_TX_FULL                -1
#define RESULT_RX_FULL                -2
#define RESULT_MAX_RETRY              -3


#define  NRF_TX_BUFFER_SIZE 200




class NRF24L01Streaming : public Stream, public NRF24L01Simple{

  public:

    // Constructor
    NRF24L01Streaming(uint8_t cePin, uint8_t csnPin);

    // Streaming interface
    virtual int availableForWrite(void);
    virtual void flush(void);
    virtual size_t write(uint8_t);
    virtual int available() { return 0; }
    virtual int read() { return 0; }
    virtual int peek() { return 0; }

    inline size_t write(unsigned long n) { return write((uint8_t)n); }
    inline size_t write(long n) { return write((uint8_t)n); }
    inline size_t write(unsigned int n) { return write((uint8_t)n); }
    inline size_t write(int n) { return write((uint8_t)n); }

    using Print::write; // pull in write(str) and write(buf, size) from Print

  protected:
    void writeDoneCallback(void);
  private:

    // streaming interface
    volatile uint8_t tx_head;
    volatile uint8_t tx_tail;    
    unsigned char tx_buffer[NRF_TX_BUFFER_SIZE];
    inline uint8_t availableTx(void);  // amount of characters in stream buffer to send to other side
    inline uint8_t readTx(void);  // read a character from the TX buffer

};

#endif
