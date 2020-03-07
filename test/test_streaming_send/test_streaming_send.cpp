
/*

Demonstrates simple RX and TX operation.
Any of the Basic_RX examples can be used as a receiver.
Please read through 'radioStreaming.h' for a description of all the methods available in the library.

Radio    Arduino
CE    -> 9
CSN   -> 10 (Hardware SPI SS)
MOSI  -> 11 (Hardware SPI MOSI)
MISO  -> 12 (Hardware SPI MISO)
SCK   -> 13 (Hardware SPI SCK)
IRQ   -> No connection
VCC   -> No more than 3.6 volts
GND   -> GND

*/

#include <NRF24L01Streaming.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN     2
#define LED_COUNT  16

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define NRF24L01SIMPLE_DEBUG 1
const static uint8_t TX_ID = 1; // Id of the radio with sender role.
const static uint8_t RX_ID = 2; // Id of the radio we will transmit to.

NRF24L01Streaming radio(10,9);         // Arduino nano


typedef struct RadioPacket // Any packet up to 32 bytes can be sent.
{
    uint8_t  command;
    uint8_t  fromId;
    uint32_t deltaMicros;
    uint32_t retry;
}RadioPacket_t;

RadioPacket_t radioPacket;


ReceivePacket_t* dataP;

long nextSendMillis = millis() + 1000;
long counter = 0;

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(uint8_t i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}
void setup()
{
    counter = 0;
    Serial.begin(115200);
    Serial.println("Testing streaming interface with nlrL24+ wireless TX role");

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();            // Turn OFF all pixels ASAP
    strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
    colorWipe(strip.Color(255,   0,   0)     , 50); // Red
    colorWipe(strip.Color(5,   5,   5)       , 50); // almost off    


    
    // By default, 'begin' configures the radio to use a 2MBPS bitrate on channel 100 (channels 0-125 are valid).
    // Both the RX and TX radios must have the same bitrate and channel to communicate with each other.
    
    if (!radio.begin(NRF24L01Simple::BITRATE1MBPS))
    {
        Serial.println("Cannot communicate with radio");
        while (1); // Wait here forever.
    }
    radio.powerUp(NRF24L01Simple::POWER_LOW , NRF24L01Simple::CHECKSUM_1_BYTE);
    radio.setChannel(10);

    radioPacket.command = 'I';
    radioPacket.fromId = TX_ID;

    uint8_t address[5] = { 1, 2, 3, 4, RX_ID };

    radio.setTxPipe(address);
    radio.setRxPipe(0, address);

    Serial.println("INIT Done");
    radio.printDetails();
}

void loop()
{
    // By default, 'send' transmits data and waits for an acknowledgement.  If no acknowledgement is received,
    // it will try again up to 16 times.  You can also perform a NO_ACK send that does not request an acknowledgement.
    // The data packet will only be transmitted a single time so there is no guarantee it will be successful.  Any random
    // electromagnetic interference can sporatically cause packets to be lost, so NO_ACK sends are only suited for certain
    // types of situations, such as streaming real-time data where performance is more important than reliability.
    //   radio.send(DESTINATIONradio_ID, &radioPacket, sizeof(radioPacket), radioStreaming::NO_ACK)
    //   radio.send(DESTINATIONradio_ID, &radioPacket, sizeof(radioPacket), radioStreaming::REQUIRE_ACK) // THE DEFAULT


    long now = millis();
    if (nextSendMillis < now) { 
        counter++;
        radioPacket.retry = radio.countRetransmit();
        radioPacket.deltaMicros = counter;

        nextSendMillis = now + 1000;
        radio.print("Hello World. ");
        radio.print ("I");
        radio.print(" am printing from streaming interface. Now:");
        radio.print(now);
        radio.print(" Counting: ");
        radio.print(counter);
        radio.print(" hex: ");
        radio.print(counter,HEX);
        radio.print(" bin: ");
        radio.print(counter,BIN);
        radio.print(" retries: ");
        radio.print(radio.countRetransmit());
        radio.print(" newline\n");

        colorWipe(strip.Color(  0,0,5), 10); 
        Serial.print("Sending ");
        Serial.print(counter);

        int8_t status = 0;
        do  {
            status = radio.checkSend(NRF24L01Simple::ACK);
            if (status == RESULT_TX_FULL){
                colorWipe(strip.Color(  0,0,50), 25); 
                colorWipe(strip.Color(  50,0,0), 25); 
            }
            else if (status == RESULT_MAX_RETRY){
                colorWipe(strip.Color(  0,0,50),   25); 
                colorWipe(strip.Color(  50,50,50), 50); 
            }
            // must be called to clear the MAX_RT flag in case rx is down
            radio.checkStatus();
        }
        while (status != RESULT_OK);

        radio.send(&radioPacket, sizeof(radioPacket), NRF24L01Simple::ACK); // Note how '&' must be placed in front of the variable name.
        colorWipe(strip.Color(  0,50,0), 50); 
    }
    radio.checkStatus();
}
