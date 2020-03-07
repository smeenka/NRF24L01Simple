
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


long counter;

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
long sendStartMicros = 0;

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
    Serial.println("Testing streaming interface with nlrL24+ wireless RX role");

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();            // Turn OFF all pixels ASAP
    strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
    colorWipe(strip.Color(255,   0,   0)     , 50); // Red
    colorWipe(strip.Color(5,   5,   5)       , 50); // almost off    Serial.begin(115200);

    
    // By default, 'init' configures the radio to use a 2MBPS bitrate on channel 100 (channels 0-125 are valid).
    // Both the RX and TX radios must have the same bitrate and channel to communicate with each other.
    // You can run the 'ChannelScanner' example to help select the best channel for your environment.
    // You can assign a different bitrate and channel as shown below.
    //   radio.init(RADIO_ID, PINradio_CE, PINradio_CSN, radioStreaming::BITRATE250KBPS, 0)
    //   radio.init(RADIO_ID, PINradio_CE, PINradio_CSN, radioStreaming::BITRATE1MBPS, 75)
    //   radio.init(RADIO_ID, PINradio_CE, PINradio_CSN, radioStreaming::BITRATE2MBPS, 100) // THE DEFAULT
    
    if (!radio.begin(NRF24L01Simple::BITRATE1MBPS))
    {
        Serial.println("Cannot communicate with radio");
        while (1); // Wait here forever.
    }
    radio.powerUp(NRF24L01Simple::POWER_MEDIUM , NRF24L01Simple::CHECKSUM_1_BYTE);
    radio.setChannel(10);

    radioPacket.command = 'I';

    uint8_t address[5] = { 1, 2, 3, 4, RX_ID };
    radio.setRxPipe(0, address);
    radio.startRx();
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
    radio.checkStatus();

    if (radio.hasData() ) {
        radioPacket.deltaMicros = micros() - sendStartMicros;
        radioPacket.retry = radio.countRetransmit();

        dataP = radio.getData();
        Serial.print("Packet len:");
        Serial.print(dataP->length);
        Serial.print(" pipe:");
        Serial.print(dataP->pipe);
        Serial.print(" ");

        if (dataP->data[0] == 'P'){
            // ok it is a streaming packet
            for (int i = 1; i < dataP->length; i++){
                char x = (char) dataP->data[i];
                Serial.print(x);
            }
            Serial.println("");
        } else {
            // non streaming packet
            // cast received bytes to the data fromat we use
            RadioPacket* packet = (RadioPacket*) dataP->data; 
            Serial.print("command:");
            Serial.print(packet->command);
            Serial.print(" from:");
            Serial.print(packet->fromId);
            Serial.print(" retries:");
            Serial.println(packet->retry);        
        }
        radio.startRx();
        colorWipe(strip.Color(  0,50,0), 50); 
    }
}
